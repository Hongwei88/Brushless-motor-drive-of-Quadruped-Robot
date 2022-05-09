/*
* ȷ��λ�ô�����ƫ�ơ���λ�����λ�ô��������Ի���У׼����
*/

#include "calibration.h"
#include "foc.h"
#include "PreferenceWriter.h"
#include "user_config.h"
#include "motor_config.h"
#include "current_controller_config.h"

//��鲢������λ˳��
///������������ȷ����Q������λ�ô������������������///
void order_phases(PositionSensor *ps, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs)
	{   
   //��ʼ�����FOC���Ƶı���  
    printf("\n\r Checking phase ordering\n\r");
    float theta_ref = 0;					//����һ���ο���theta�Ƕ�
    float theta_actual = 0;				//����һ��ʵ�ʵ�theta�Ƕ�
    float v_d = V_CAL;  					//�����еĵ�ѹ����D����
    float v_q = 0.0f;							//��q��ĵ�ѹ��0
    float v_u, v_v, v_w = 0;			//�����������0��������
    float dtc_u, dtc_v, dtc_w = .5f;//�������ʸ����ѹ��0
    int sample_counter = 0;				//�Ѳ��õļ���ֵ��0
   ///////////////////////////////////////////////////////////////////////////////////////////////////	
		
		
		

   
    ///����ѹ������Ϊ�㣬������ת��У��ǰ�����λ�ã�һ�ξͺ�
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);          //��ѹ��dq0�任��ͨ�����������dq����ϵ�µĵ�ѹ�ͽǶȣ��������������
    svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);     //�ռ�ʸ�����ƣ�ͨ����Դ��ѹ���������������������Ӧ�ĵ�ѹʸ�� 
		for(int i = 0; i<20000; i++)												 //���ó�ʼռ�ձȣ���ʼ���Ƶ���ˣ���һ˲������������
			{
			TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);                                
			TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
			TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
			wait_us(100);
			}
		//////
    ps->Sample(DT); 																		//ͨ���ű����������ڵ�	ת��λ�ý��в���
    wait_us(1000);
		float theta_start;	 //�ű�������ȡ���ԵĻ�е�Ƕȣ��ʼ��ʱ�򣬶����ʱ���Ĭ��Ϊ0��
    //float theta_start = ps->GetMechPositionFixed();                                  //��ȡ��ʼ��ת��λ��
		//////	
    controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);    //����ADC�Ķ������������
    controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);
    controller->i_a = -controller->i_b - controller->i_c;					//���ݻ�������������ɣ�֪�����������������������
	 
			//����dq0�任���õ�dq���ϵ�������������
    dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);   
    float current = sqrt(pow(controller->i_d, 2) + pow(controller->i_q, 2));	//�����dq���ϵ��������������ϳɵ��ܵ���
    printf("\n\rCurrent\n\r");
    printf("%f    %f   %f\n\r\n\r", controller->i_d, controller->i_q, current);
		///////////////////////////////////////////////////////////////////////////////////////////////////





			
    /// ������ת��ѹ�Ƕȣ������������ѹʸ������������ת������ת�����߼�������ʵ�ʵĻ�е�Ƕ�
    while(theta_ref < 4*PI)//��ת��Ȧ��ʵ���ϵ�Ƕ���ת6Ȧ����е�ǲ���תһȦ��
			{                                                    //��ת����������
        abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                             //��ѹ��dq0�任
        svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);                        //�ռ�ʸ������
        wait_us(100);
        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);                                        //����ռ�ձ�
        TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
        TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
       ps->Sample(DT);                                                            //�ɼ�λ�ô�����
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
			 
			 
			 
			 
    float theta_end = ps->GetMechPositionFixed(); //�ű�������ȡ���ԵĻ�е�Ƕȣ�����ʱ��	 
    int direction = (theta_end - theta_start)>0;  //�жϵ��ת���ķ�������ͨ���ж��������������ǲ���ֵ
    printf("Theta Start:   %f    Theta End:  %f\n\r", theta_start, theta_end);
    printf("Direction:  %d\n\r", direction);
    if(direction){printf("Phasing correct\n\r");}
    else if(!direction){printf("Phasing incorrect.  Swapping phases V and W\n\r");}
    PHASE_ORDER = direction;      //������λ����˳��
    }
    
//У������
///����λ�ô������ĵ�Ƕ�ƫ�ƣ����������� У������λ�ô�����ƫ�Ķ�����ķ�����/// 		
void calibrate(PositionSensor *ps, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs)
	{
		///����У������ı���//////////////////////////////////////////////////////////////////////////////////////
    printf("Starting calibration procedure\n\r");
    float * error_f;															  //����ת��¼����ת��ƫ����
    float * error_b;																//����ת��¼����ת��ƫ����
    int * lut;																			//����õı���
    int * raw_f;																		//ǰ������ת�ӵĽǶ�
    int * raw_b;																		//���˷���ת�ӵĽǶ�
    float * error;																	//��ת����ƫ��
    float * error_filt; 														//У������ת����ƫ��	
    const int n = 128*NPP;                          // ÿ�λ�е��ת�ǶȲ�����λ������
    const int n2 = 40;                              // �������֮�������������ƽ���˶���
    float delta = 2*PI*NPP/(n*n2);                  // �������ݵĽǶȱ仯
    error_f = new float[n]();                       // ��ֵ���ʸ����ǰ��ת
    error_b = new float[n]();                       // ��ֵ���ʸ�������ת
    const int  n_lut = 128;													//��ֵ����������	
    lut = new int[n_lut]();                         // �ڿ�ʼǰ��������о͵ò��ı��
    error = new float[n]();
    const int window = 128;
    error_filt = new float[n]();
    ps->WriteLUT(lut); 
    raw_f = new int[n]();
    raw_b = new int[n]();
    float theta_ref = 0;														//�ο���theta��
    float theta_actual = 0;													//��ʵ��theta��
    float v_d = V_CAL;                              //�����еĵ�ѹ����D����
    float v_q = 0.0f;																//����Q���ѹ
    float v_u, v_v, v_w = 0;												//�����ߵ�ѹ
    float dtc_u, dtc_v, dtc_w = .5f;								//�����ߵ������������
    
    ///����ѹ������Ϊ�㣬�ȴ�ת��λ���ȶ���������һ��/////////////////////////////////////////////////////////////////////////////////////////////////////
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);      //��ѹ��dq0�任
    svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); //�ռ�ʸ������
    for(int i = 0; i<40000; i++)										 //����ռ�ձ� 
		{
			TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);        //����V���PWM                        
			if(PHASE_ORDER)																 //֮ǰ�Ѿ������UV˳��ģ�����ȷ����˳�����ѹ�ź�
				{                                   
					TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);		 //����V���PWM
					TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);		 //����W���PWM
				}
			else
				{
					TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_v);		 //����V���PWM
					TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_w);    //����W���PWM
				}
			wait_us(100);
		}
    ps->Sample(DT); 																//�ű�����  ��ʼ����
    controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);     //����ADC�Ķ������������B,ͨ����ʵʱADC�ɼ�����IC�ĵ�������-У����ĵ���������*��λ�����Ӧ�ĵ����=ʵ�ʵĵ���
    controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);		 //����ADC�Ķ������������C
    controller->i_a = -controller->i_b - controller->i_c;																	 //����ADC�Ķ������������A
    dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);   //����dq0�任
    float current = sqrt(pow(controller->i_d, 2) + pow(controller->i_q, 2));							 //�����ܵ���
    printf(" Current Angle : Rotor Angle : Raw Encoder \n\r\n\r");
		
		
		
		///��ǰ��ת//////////////////////////////////////////////////////////////////////////////////////////////////////
    for(int i = 0; i<n; i++)		 												// ÿ�λ�е��ת�ǶȲ�����λ������
		{                                                   
       for(int j = 0; j<n2; j++) 												// ��ǰ��ת������ô���PWM��Ϊ�˿�����Ӧƽ��һ��
			{   
       theta_ref += delta;			 												// �ο��ĵ�ǶȰ���������
       abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);			//��ѹ��dq0�任
       svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); //�ռ�ʸ������
        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);
        if(PHASE_ORDER){                                //�������˳�� 
            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);     // ����ռ�ձ�
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
       error_f[i] = theta_ref/NPP - theta_actual;		//��������ǶȺ���ת�Ƕȵ�ƫ�ȷ��λ�ô�������ƫ��
       raw_f[i] = ps->GetRawPosition();							//��������¼��λ��
        printf("%.4f   %.4f    %d\n\r", theta_ref/(NPP), theta_actual, raw_f[i]);//��ӡ�����Ļ�е�Ƕ�
    }

	///�����ת��ȡ///////////////////////////////////////////////////////////////////////////////////////////////////////////	
    for(int i = 0; i<n; i++)														// ÿ�λ�е��ת�ǶȲ�����λ������							
		{                                                   
			 for(int j = 0; j<n2; j++)								  			// �����ת
			{
			 theta_ref -= delta;														  // �ο��ĵ�ǶȰ���������
			 abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);      //��ѹ��dq0�任
			 svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); //�ռ�ʸ������
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
			 ps->Sample(DT);                                  //����λ�ô�����
			 theta_actual = ps->GetMechPositionFixed();       // ��ȡ��е�Ƕ�
			 error_b[i] = theta_ref/NPP - theta_actual;				//��������ǶȺ���ת�Ƕȵ�ƫ�ȷ��λ�ô�������ƫ��
			 raw_b[i] = ps->GetRawPosition();
			 printf("%.4f   %.4f    %d\n\r", theta_ref/(NPP), theta_actual, raw_b[i]);
			 //theta_ref -= delta;
		}    
 	////����ƽ��λ�ô�����ƫ�Ʋ����ôű�������ƫ�ƼĴ���//////////////////////////////////////////////////////////////////////////////////////////////////////////	
        float offset = 0;                                  
        for(int i = 0; i<n; i++)
					{
					offset += (error_f[i] + error_b[n-1-i])/(2.0f*n);                   // ����ƽ��λ�ô�����ƫ��
					}
        offset = fmod(offset*NPP, 2*PI);                                    // ����е�Ƕ�ת��Ϊ�����Ƕ�   
        ps->SetElecOffset(offset);                                          // ����λ�ô�����ƫ��
        __float_reg[0] = offset;
        E_OFFSET = offset;
        
	/// �����Ի�λ�ô�����ƫ����ִ���˲�
	/// nλ����ƽ��ֵ������n=һ����ѭ���е���Ʒ����
	///���˲����ڵ�Ƶ��������Ϊ0���Ҿ�Ϊ�����������Գݲ�ЧӦӦ�ûر���ȫ�˵�
        float mean = 0;
        for (int i = 0; i<n; i++)   //ƽ��ǰ����������
					{                                              
            error[i] = 0.5f*(error_f[i] + error_b[n-i-1]);
          }
					
				//�˲�	
        for (int i = 0; i<n; i++)
					{
            for(int j = 0; j<window; j++)
						{
                int ind = -window/2 + j + i;                                    // Indexes from -window/2 to + window/2
                if(ind<0)
									{ind += n;}                                                  //�ƶ�ƽ������ 
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
        int raw_offset = (raw_f[0] + raw_b[n-1])/2;                             //���������������У��������־��㹻��
        
       // //�������Ի����ұ�
        printf("\n\r Encoder non-linearity compensation table\n\r");
        printf(" Sample Number : Lookup Index : Lookup Value\n\r\n\r");
        for (int i = 0; i<n_lut; i++)
					 {                                          //���ɲ��ұ�
            int ind = (raw_offset>>7) + i;
            if(ind > (n_lut-1)){ 
                ind -= n_lut;
                }
            lut[ind] = (int) ((error_filt[i*NPP] - mean)*(float)(ps->GetCPR())/(2.0f*PI));
            printf("%d   %d   %d \n\r", i, ind, lut[ind]);
            wait(.001);
            }
            
        ps->WriteLUT(lut);                                                      // �����ұ�д��λ�ô���������
        memcpy(&ENCODER_LUT, lut, sizeof(lut));                                 // �����ұ��Ƶ���������
        printf("\n\rEncoder Electrical Offset (rad) %f\n\r",  offset);
        //���浽����
        if (!prefs->ready()) prefs->open();
        prefs->flush();                                                         // ��ƫ�����Ͳ��ұ�д������
        prefs->close();
        
        delete[] error_f;       //�����ͷ�����
        delete[] error_b;
        delete[] lut;
        delete[] raw_f;
        delete[] raw_b;

    }
