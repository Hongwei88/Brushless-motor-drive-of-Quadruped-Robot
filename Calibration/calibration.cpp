/*
* 确定位置传感器偏移、相位排序和位置传感器线性化的校准程序
*/

#include "calibration.h"
#include "foc.h"
#include "PreferenceWriter.h"
#include "user_config.h"
#include "motor_config.h"
#include "current_controller_config.h"

//检查并设置相位顺序
///检查相序命令，以确保正Q电流在位置传感器正方向产生电流///
void order_phases(PositionSensor *ps, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs)
	{   
   //初始化电机FOC控制的变量  
    printf("\n\r Checking phase ordering\n\r");
    float theta_ref = 0;					//定义一个参考的theta角度
    float theta_actual = 0;				//定义一个实际的theta角度
    float v_d = V_CAL;  					//把所有的电压放在D轴上
    float v_q = 0.0f;							//把q轴的电压置0
    float v_u, v_v, v_w = 0;			//把三相振幅置0（标量）
    float dtc_u, dtc_v, dtc_w = .5f;//把三相的矢量电压置0
    int sample_counter = 0;				//把采用的计数值置0
   ///////////////////////////////////////////////////////////////////////////////////////////////////	
		
		
		

   
    ///将电压角设置为零，即设置转动校正前电机的位置，一次就好
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);          //电压反dq0变换，通过上面给定的dq坐标系下的电压和角度，计算出三相的振幅
    svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);     //空间矢量调制，通过电源电压和三相振幅，计算出三相对应的电压矢量 
		for(int i = 0; i<20000; i++)												 //设置初始占空比，开始控制电机了，这一瞬间电机是锁死的
			{
			TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);                                
			TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
			TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
			wait_us(100);
			}
		//////
    ps->Sample(DT); 																		//通过磁编码器对现在的	转子位置进行采样
    wait_us(1000);
		float theta_start;	 //磁编码器获取绝对的机械角度，最开始的时候，定义的时候旧默认为0了
    //float theta_start = ps->GetMechPositionFixed();                                  //获取初始化转子位置
		//////	
    controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);    //根据ADC的读数计算相电流
    controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);
    controller->i_a = -controller->i_b - controller->i_c;					//根据基尔霍夫电流定律，知道两相电流可以求第三相电流
	 
			//电流dq0变换，得到dq轴上的两个电流分量
    dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);   
    float current = sqrt(pow(controller->i_d, 2) + pow(controller->i_q, 2));	//计算出dq轴上的两个电流分量合成的总电流
    printf("\n\rCurrent\n\r");
    printf("%f    %f   %f\n\r\n\r", controller->i_d, controller->i_q, current);
		///////////////////////////////////////////////////////////////////////////////////////////////////





			
    /// 控制旋转电压角度，持续让三相电压矢量沿着正方向转动，边转动，边计算和输出实际的机械角度
    while(theta_ref < 4*PI)//旋转四圈，实际上电角度旋转6圈，机械角才旋转一圈的
			{                                                    //旋转两个电周期
        abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                             //电压反dq0变换
        svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);                        //空间矢量调制
        wait_us(100);
        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);                                        //设置占空比
        TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
        TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
       ps->Sample(DT);                                                            //采集位置传感器
       theta_actual = ps->GetMechPositionFixed();
       if(theta_ref==0){theta_start = theta_actual;}
       if(sample_counter > 200)
				{
           sample_counter = 0 ;
        printf("%.4f   %.4f\n\r", theta_ref/(NPP), theta_actual);
        }
        sample_counter++;
       theta_ref += 0.001f;
       }
		///////////////////////////////////////////////////////////////////////////////////////////////////	
			 
			 
			 
			 
    float theta_end = ps->GetMechPositionFixed(); //磁编码器获取绝对的机械角度，最后的时候	 
    int direction = (theta_end - theta_start)>0;  //判断电机转动的方向，这里通过判断运算符，输出的是布尔值
    printf("Theta Start:   %f    Theta End:  %f\n\r", theta_start, theta_end);
    printf("Direction:  %d\n\r", direction);
    if(direction){printf("Phasing correct\n\r");}
    else if(!direction){printf("Phasing incorrect.  Swapping phases V and W\n\r");}
    PHASE_ORDER = direction;      //更改相位方向顺序
    }
    
//校正程序
///测量位置传感器的电角度偏移，并（将来） 校正由于位置传感器偏心而引起的非线性/// 		
void calibrate(PositionSensor *ps, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs)
	{
		///定义校正程序的变量//////////////////////////////////////////////////////////////////////////////////////
    printf("Starting calibration procedure\n\r");
    float * error_f;															  //正旋转记录的旋转角偏移量
    float * error_b;																//反旋转记录的旋转角偏移量
    int * lut;																			//查表用的变量
    int * raw_f;																		//前进方向转子的角度
    int * raw_b;																		//后退方向转子的角度
    float * error;																	//旋转量总偏差
    float * error_filt; 														//校正后旋转量总偏差	
    const int n = 128*NPP;                          // 每次机械旋转角度采样的位置数量
    const int n2 = 40;                              // 保存采样之间的增量（用于平滑运动）
    float delta = 2*PI*NPP/(n*n2);                  // 采样数据的角度变化
    error_f = new float[n]();                       // 赋值误差矢量向前旋转
    error_b = new float[n]();                       // 赋值误差矢量向后旋转
    const int  n_lut = 128;													//赋值查表遍历变量	
    lut = new int[n_lut]();                         // 在开始前，清除所有就得查阅表格
    error = new float[n]();
    const int window = 128;
    error_filt = new float[n]();
    ps->WriteLUT(lut); 
    raw_f = new int[n]();
    raw_b = new int[n]();
    float theta_ref = 0;														//参考的theta角
    float theta_actual = 0;													//真实的theta角
    float v_d = V_CAL;                              //把所有的电压放在D轴上
    float v_q = 0.0f;																//清零Q轴电压
    float v_u, v_v, v_w = 0;												//三相线电压
    float dtc_u, dtc_v, dtc_w = .5f;								//三相线电流（不清楚）
    
    ///将电压角设置为零，等待转子位置稳定，仅仅做一次/////////////////////////////////////////////////////////////////////////////////////////////////////
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);      //电压反dq0变换
    svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); //空间矢量调制
    for(int i = 0; i<40000; i++)										 //设置占空比 
		{
			TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);        //给定V相的PWM                        
			if(PHASE_ORDER)																 //之前已经检查了UV顺序的，按着确定的顺序给电压信号
				{                                   
					TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);		 //给定V相的PWM
					TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);		 //给定W相的PWM
				}
			else
				{
					TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_v);		 //给定V相的PWM
					TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_w);    //给定W相的PWM
				}
			wait_us(100);
		}
    ps->Sample(DT); 																//磁编码器  开始采样
    controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);     //根据ADC的读数计算相电流B,通过（实时ADC采集驱动IC的电流计算-校正后的电流计数）*单位计算对应的电流�=实际的电流
    controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);		 //根据ADC的读数计算相电流C
    controller->i_a = -controller->i_b - controller->i_c;																	 //根据ADC的读数计算相电流A
    dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);   //电流dq0变换
    float current = sqrt(pow(controller->i_d, 2) + pow(controller->i_q, 2));							 //计算总电流
    printf(" Current Angle : Rotor Angle : Raw Encoder \n\r\n\r");
		
		
		
		///向前旋转//////////////////////////////////////////////////////////////////////////////////////////////////////
    for(int i = 0; i<n; i++)		 												// 每次机械旋转角度采样的位置数量
		{                                                   
       for(int j = 0; j<n2; j++) 												// 向前旋转，发那么多次PWM是为了控制响应平滑一点
			{   
       theta_ref += delta;			 												// 参考的电角度按增量递增
       abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);			//电压反dq0变换
       svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); //空间矢量调制
        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);
        if(PHASE_ORDER){                                //检查相序顺序 
            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);     // 设置占空比
            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
            }
        else{
            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_v);
            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_w);
            }
            wait_us(100);
            ps->Sample(DT);
        }
       ps->Sample(DT);
       theta_actual = ps->GetMechPositionFixed();
       error_f[i] = theta_ref/NPP - theta_actual;		//计算给定角度和旋转角度的偏差，确定位置传感器的偏移
       raw_f[i] = ps->GetRawPosition();							//编码器记录的位置
        printf("%.4f   %.4f    %d\n\r", theta_ref/(NPP), theta_actual, raw_f[i]);//打印给定的机械角度
    }

	///向后旋转获取///////////////////////////////////////////////////////////////////////////////////////////////////////////	
    for(int i = 0; i<n; i++)														// 每次机械旋转角度采样的位置数量							
		{                                                   
			 for(int j = 0; j<n2; j++)								  			// 向后旋转
			{
			 theta_ref -= delta;														  // 参考的电角度按增量递增
			 abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);      //电压反dq0变换
			 svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); //空间矢量调制
				TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);
				if(PHASE_ORDER)
						{
						TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
						TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
						}
				else{
						TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_v);
						TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_w);
						}
						wait_us(100);
						ps->Sample(DT);
				}
			 ps->Sample(DT);                                  //采用位置传感器
			 theta_actual = ps->GetMechPositionFixed();       // 获取机械角度
			 error_b[i] = theta_ref/NPP - theta_actual;				//计算给定角度和旋转角度的偏差，确定位置传感器的偏移
			 raw_b[i] = ps->GetRawPosition();
			 printf("%.4f   %.4f    %d\n\r", theta_ref/(NPP), theta_actual, raw_b[i]);
			 //theta_ref -= delta;
		}    
 	////计算平均位置传感器偏移并设置磁编码器的偏移寄存器//////////////////////////////////////////////////////////////////////////////////////////////////////////	
        float offset = 0;                                  
        for(int i = 0; i<n; i++)
					{
					offset += (error_f[i] + error_b[n-1-i])/(2.0f*n);                   // 计算平均位置传感器偏移
					}
        offset = fmod(offset*NPP, 2*PI);                                    // 将机械角度转换为电气角度   
        ps->SetElecOffset(offset);                                          // 设置位置传感器偏移
        __float_reg[0] = offset;
        E_OFFSET = offset;
        
	/// 以线性化位置传感器偏心率执行滤波
	/// n位采样平均值，其中n=一个电循环中得样品数量
	///该滤波器在点频率下增益为0，且均为整倍数，所以齿槽效应应该回避完全滤掉
        float mean = 0;
        for (int i = 0; i<n; i++)   //平均前后两个方向
					{                                              
            error[i] = 0.5f*(error_f[i] + error_b[n-i-1]);
          }
					
				//滤波	
        for (int i = 0; i<n; i++)
					{
            for(int j = 0; j<window; j++)
						{
                int ind = -window/2 + j + i;                                    // Indexes from -window/2 to + window/2
                if(ind<0)
									{ind += n;}                                                  //移动平均绕线 
                else if(ind > n-1) 
									{ind -= n;}
                error_filt[i] += error[ind]/(float)window;
            }
            if(i<window){
//                cogging_current[i] = current*sinf((error[i] - error_filt[i])*NPP);
                }
            //printf("%.4f   %4f    %.4f   %.4f\n\r", error[i], error_filt[i], error_f[i], error_b[i]);
            mean += error_filt[i]/n;
           }
        int raw_offset = (raw_f[0] + raw_b[n-1])/2;                             //对这个方向错误不敏感，所以两分就足够了
        
       // //生成线性化查找表
        printf("\n\r Encoder non-linearity compensation table\n\r");
        printf(" Sample Number : Lookup Index : Lookup Value\n\r\n\r");
        for (int i = 0; i<n_lut; i++)
					 {                                          //生成查找表
            int ind = (raw_offset>>7) + i;
            if(ind > (n_lut-1)){ 
                ind -= n_lut;
                }
            lut[ind] = (int) ((error_filt[i*NPP] - mean)*(float)(ps->GetCPR())/(2.0f*PI));
            printf("%d   %d   %d \n\r", i, ind, lut[ind]);
            wait(.001);
            }
            
        ps->WriteLUT(lut);                                                      // 将查找表写入位置传感器对象
        memcpy(&ENCODER_LUT, lut, sizeof(lut));                                 // 将查找表复制到闪存阵列
        printf("\n\rEncoder Electrical Offset (rad) %f\n\r",  offset);
        //保存到闪存
        if (!prefs->ready()) prefs->open();
        prefs->flush();                                                         // 将偏移量和查找表写入闪存
        prefs->close();
        
        delete[] error_f;       //必须释放闪存
        delete[] error_b;
        delete[] lut;
        delete[] raw_f;
        delete[] raw_b;

    }
