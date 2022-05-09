
#include "foc.h"
using namespace FastMath;

/// 逆DQ0变换 (逆park变换+逆clark变换) ///
/// 相电流幅值=dq矢量长度 ///
///i.e. iq = 1, id = 0, 峰值相电流=1
///输入的变量是DQ轴上的角度theta，D轴上的电压值，Q轴上的电压值
///输出的是三相上各自的电压，这三个电压使输入给驱动器IC的
///实质就是公式变换而已
void abc( float theta, float d, float q, float *a, float *b, float *c)
{
    float cf = FastCos(theta);
    float sf = FastSin(theta);
    
    *a = cf*d - sf*q;                // Faster Inverse DQ0 transform
    *b = (0.86602540378f*sf-.5f*cf)*d - (-0.86602540378f*cf-.5f*sf)*q;
    *c = (-0.86602540378f*sf-.5f*cf)*d - (0.86602540378f*cf-.5f*sf)*q;
}
    
/// DQ0变换 (park变换+clark变换)///
/// 相电流幅值=dq矢量长度 ///
///i.e. iq = 1, id = 0, 峰值相电流=1
///输入的是三相电流，三相电流是先经过驱动IC转换成电压信号后，我们用单片机ADC进行采集换算出来的
///输出的是dq轴上的电流分量
void dq0(float theta, float a, float b, float c, float *d, float *q)
{    
    float cf = FastCos(theta);
    float sf = FastSin(theta);
    
    *d = 0.6666667f*(cf*a + (0.86602540378f*sf-.5f*cf)*b + (-0.86602540378f*sf-.5f*cf)*c);   ///Faster DQ0 Transform
    *q = 0.6666667f*(-sf*a - (-0.86602540378f*cf-.5f*sf)*b - (0.86602540378f*cf-.5f*sf)*c);     
}

/// 空间矢量调制 ，这个使正弦波转换成PWM波的///
/// u,v,w 振幅 = v_bus全调制深度 
/// 输入的是电源电压，三相的振幅
/// 输出的是三相的电压矢量
/// 实质也是公式转换而已（理论）
void svm(float v_bus, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w)
{
    float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w))*0.5f;
    
    *dtc_u = fminf(fmaxf(((u -v_offset)/v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_v = fminf(fmaxf(((v -v_offset)/v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_w = fminf(fmaxf(((w -v_offset)/v_bus + .5f), DTC_MIN), DTC_MAX); 
    
    /*
    sinusoidal pwm
    *dtc_u = fminf(fmaxf((u/v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_v = fminf(fmaxf((v/v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_w = fminf(fmaxf((w/v_bus + .5f), DTC_MIN), DTC_MAX);
    */    
}


/// 使逆变器的输出线性化，对于小占空比而言，输出不是线性化的 ///
void linearize_dtc(float *dtc)
{
    float sgn = 1.0f-(2.0f*(dtc<0));
    if(abs(*dtc) >= .01f)
    {
        *dtc = *dtc*.986f+.014f*sgn;
    }
    else
    {
        *dtc = 2.5f*(*dtc);
    }
}
    
/// 测量电流传感器的零点偏移 /// 
void zero_current(int *offset_1, int *offset_2)     
{
    int adc1_offset = 0;
    int adc2_offset = 0;
    int n = 1024;
    for (int i = 0; i<n; i++)
    {                                               // Average n samples of the ADC
        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f);           // Write duty cycles
        TIM1->CCR2 = (PWM_ARR>>1)*(1.0f);
        TIM1->CCR1 = (PWM_ARR>>1)*(1.0f);
        ADC1->CR2  |= ADC_CR2_SWSTART;              // Begin sample and conversion
        wait(0.001);
        adc2_offset += ADC2->DR;
        adc1_offset += ADC1->DR;
    }
    *offset_1 = adc1_offset/n;
    *offset_2 = adc2_offset/n;
}

////初始化电机FOC五个控制参数
void init_controller_params(ControllerStruct *controller)
{
    controller->ki_d = KI_D;
    controller->ki_q = KI_Q;
    controller->k_d = K_SCALE*I_BW;
    controller->k_q = K_SCALE*I_BW;
    controller->alpha = 1.0f - 1.0f/(1.0f - DT*I_BW*2.0f*PI);    
}

//复位清零FOC参数,即把dq轴的数据清零就好了
void reset_foc(ControllerStruct *controller)
{
    TIM1->CCR3 = (PWM_ARR>>1)*(0.5f);
    TIM1->CCR1 = (PWM_ARR>>1)*(0.5f);
    TIM1->CCR2 = (PWM_ARR>>1)*(0.5f);
    controller->i_d_ref = 0;
    controller->i_q_ref = 0;
    controller->i_d = 0;
    controller->i_q = 0;
    controller->i_q_filt = 0;
    controller->q_int = 0;
    controller->d_int = 0;
    controller->v_q = 0;
    controller->v_d = 0;
}

//复位温度观测器，即复位温度值
void reset_observer(ObserverStruct *observer)
{
    observer->temperature = 25.0f;
    observer->resistance = .1f;
}

//参考限制电流，电流限幅
void limit_current_ref (ControllerStruct *controller)
{
    float i_q_max_limit = (0.5774f*controller->v_bus - controller->dtheta_elec*WB)/R_PHASE;
    float i_q_min_limit = (-0.5774f*controller->v_bus - controller->dtheta_elec*WB)/R_PHASE;
    controller->i_q_ref = fmaxf(fminf(i_q_max_limit, controller->i_q_ref), i_q_min_limit);
}

//电流闭环程序
void commutate(ControllerStruct *controller, ObserverStruct *observer, GPIOStruct *gpio, float theta)
	{       
        /// 更新温度观测器估计值 ///////////////////////////////////////////////////////////////////////////////////////////////////////
        // 温度观测器//
        float t_rise = (float)observer->temperature - 25.0f;			//25度是标准参考温度，温升=ADC通过温阻线计算的温度-标准参考温度
        float q_th_in = (1.0f + .00393f*t_rise)*(controller->i_d*controller->i_d*R_PHASE*SQRT3 + controller->i_q*controller->i_q*R_PHASE*SQRT3);
        float q_th_out = t_rise*R_TH;
        observer->temperature += INV_M_TH*DT*(q_th_in-q_th_out);
				// 温阻抗阻抗观测器 //
        observer->resistance = (controller->v_q - SQRT3*controller->dtheta_elec*(WB))/controller->i_q;
        if(isnan(observer->resistance)){observer->resistance = R_PHASE;}
        observer->temperature2 = (double)(25.0f + ((observer->resistance*6.0606f)-1.0f)*275.5f);
        double e = observer->temperature - observer->temperature2;
        observer->temperature -= .001*e;

				
				
       ///电流闭环//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       controller->loop_count ++;   
	/*（第一步）测量三相电流*/
       if(PHASE_ORDER)  //正向时， 检查电流传感器的命令，确定VU相位顺序
				   {                                                                        
           controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);    // 根据ADC的读数计算B相电流
           controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);		 // 根据ADC的读数计算C相电流
           }
        else					 //反向时
					 {
            controller->i_b = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);    
            controller->i_c = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);
           }
						controller->i_a = -controller->i_b - controller->i_c;    															 	//根据基尔霍夫计算A相电流   
       
	/*（第二步）【重要】DQ0变换 (park变换+clark变换)*/	 
       float s = FastSin(theta); 
       float c = FastCos(theta);                            
       dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //电流dq变换（park变换+clark变换）
				
					 
	/*（第三步）计算参考的dq轴电流i_d_ref、i_q_ref*/	
        controller->i_q_filt = 0.95f*controller->i_q_filt + 0.05f*controller->i_q;
        controller->i_d_filt = 0.95f*controller->i_d_filt + 0.05f*controller->i_d;
        //加入alpha电流角度
        controller->i_d_ref_filt = (1.0f-controller->alpha)*controller->i_d_ref_filt + controller->alpha*controller->i_d_ref;
        controller->i_q_ref_filt = (1.0f-controller->alpha)*controller->i_q_ref_filt + controller->alpha*controller->i_q_ref;
       /// 弱磁中 ///
       controller->fw_int += .001f*(0.5f*OVERMODULATION*controller->v_bus - controller->v_ref);
       controller->fw_int = fmaxf(fminf(controller->fw_int, 0.0f), -I_FW_MAX);
       controller->i_d_ref = controller->fw_int;
       limit_norm(&controller->i_d_ref, &controller->i_q_ref, I_MAX);//电流限幅输出，目的是使其输出值在电源电流I_MAX为半径的圆圈内
       
       
       
 /*（第四步）PI 控制器，输入i_d_ref和i_d，输出v_d、v_q*/
			 //（1）计算PI控制器误差
       float i_d_error = controller->i_d_ref - controller->i_d;
       float i_q_error = controller->i_q_ref - controller->i_q;//  + cogging_current;
	  	 //计算前馈电压，前馈电压=相电流*相电阻//
			 // float v_d_ff = SQRT3*(1.0f*controller->i_d_ref*R_PHASE  - controller->dtheta_elec*L_Q*controller->i_q);   //feed-forward voltages
			 // float v_q_ff =  SQRT3*(1.0f*controller->i_q_ref*R_PHASE +  controller->dtheta_elec*(L_D*controller->i_d + 1.0f*WB));
       
       //（2）计算PI控制器增益P、积分误差I//
       controller->d_int += controller->k_d*controller->ki_d*i_d_error;  //ki_d认为是d轴电流增益，k_d才是电流环增益
       controller->q_int += controller->k_q*controller->ki_q*i_q_error;	 //ki_q认为使q轴电流增益，k_q才是电流环增益
       controller->d_int = fmaxf(fminf(controller->d_int, OVERMODULATION*controller->v_bus), - OVERMODULATION*controller->v_bus);
       controller->q_int = fmaxf(fminf(controller->q_int, OVERMODULATION*controller->v_bus), - OVERMODULATION*controller->v_bus); 
       controller->v_d = controller->k_d*i_d_error + controller->d_int ;//+ v_d_ff;  
       controller->v_q = controller->k_q*i_q_error + controller->q_int ;//+ v_q_ff; 
			 
       // （3）计算参考电压，对v_d、v_q进行线性化
       controller->v_ref = sqrt(controller->v_d*controller->v_d + controller->v_q*controller->v_q);
       limit_norm(&controller->v_d, &controller->v_q, OVERMODULATION*controller->v_bus);       //电压限幅输出，目的是使其输出值在电源电压v_bus为半径的圆圈内
       float dtc_d = controller->v_d/controller->v_bus;//dtc_d仅仅使一个比值，用于线性化
       float dtc_q = controller->v_q/controller->v_bus;
       linearize_dtc(&dtc_d);													 //使逆变器的输出线性化
       linearize_dtc(&dtc_q);
       controller->v_d = dtc_d*controller->v_bus;			 //d轴电压
       controller->v_q = dtc_q*controller->v_bus;			 //q轴电压
			 
 /*（第五步）【重要】逆DQ0变换 (逆park变换+逆clark变换)*/			 			 
       abc(controller->theta_elec + 0.0f*DT*controller->dtheta_elec, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //电压逆dq变换
 /*（第六步） 空间矢量调制 ，这个使正弦波转换成PWM波的*/			 
       svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //空间矢量调制
       if(PHASE_ORDER)													 							// 检查UV相顺序
				 {                                                        
            TIM1->CCR3 = (PWM_ARR)*(1.0f-controller->dtc_u);  //写PWM的占空比
            TIM1->CCR2 = (PWM_ARR)*(1.0f-controller->dtc_v);
            TIM1->CCR1 = (PWM_ARR)*(1.0f-controller->dtc_w);
        }
        else{
            TIM1->CCR3 = (PWM_ARR)*(1.0f-controller->dtc_u);
            TIM1->CCR1 = (PWM_ARR)*(1.0f-controller->dtc_v);
            TIM1->CCR2 =  (PWM_ARR)*(1.0f-controller->dtc_w);
        }
       controller->theta_elec = theta;   //记录电角度                                       
    }
    
 //进行力矩控制 
void torque_control(ControllerStruct *controller)
{
	//生成参考的力矩：参考力矩值=Kp(用户期望的位置-编码器的机械位置)+用户设置的力矩值+Kd(用户期望的速度-编码器的速度)
    float torque_ref = controller->kp*(controller->p_des - controller->theta_mech) + controller->t_ff + controller->kd*(controller->v_des - controller->dtheta_mech);
  //给定q轴的相电流，用参考的力矩给定 
		controller->i_q_ref = torque_ref/KT_OUT;    //i_q_ref与转矩有关，生成q轴的相电流=参考力矩/dq轴等效的电流
  //给定d轴的相电流    
		controller->i_d_ref = 0.0f;									//i_d_ref与磁通有关
}


