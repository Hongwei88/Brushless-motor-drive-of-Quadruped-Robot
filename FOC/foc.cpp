
#include "foc.h"
using namespace FastMath;

/// ��DQ0�任 (��park�任+��clark�任) ///
/// �������ֵ=dqʸ������ ///
///i.e. iq = 1, id = 0, ��ֵ�����=1
///����ı�����DQ���ϵĽǶ�theta��D���ϵĵ�ѹֵ��Q���ϵĵ�ѹֵ
///������������ϸ��Եĵ�ѹ����������ѹʹ�����������IC��
///ʵ�ʾ��ǹ�ʽ�任����
void abc( float theta, float d, float q, float *a, float *b, float *c)
{
    float cf = FastCos(theta);
    float sf = FastSin(theta);
    
    *a = cf*d - sf*q;                // Faster Inverse DQ0 transform
    *b = (0.86602540378f*sf-.5f*cf)*d - (-0.86602540378f*cf-.5f*sf)*q;
    *c = (-0.86602540378f*sf-.5f*cf)*d - (0.86602540378f*cf-.5f*sf)*q;
}
    
/// DQ0�任 (park�任+clark�任)///
/// �������ֵ=dqʸ������ ///
///i.e. iq = 1, id = 0, ��ֵ�����=1
///��������������������������Ⱦ�������ICת���ɵ�ѹ�źź������õ�Ƭ��ADC���вɼ����������
///�������dq���ϵĵ�������
void dq0(float theta, float a, float b, float c, float *d, float *q)
{    
    float cf = FastCos(theta);
    float sf = FastSin(theta);
    
    *d = 0.6666667f*(cf*a + (0.86602540378f*sf-.5f*cf)*b + (-0.86602540378f*sf-.5f*cf)*c);   ///Faster DQ0 Transform
    *q = 0.6666667f*(-sf*a - (-0.86602540378f*cf-.5f*sf)*b - (0.86602540378f*cf-.5f*sf)*c);     
}

/// �ռ�ʸ������ �����ʹ���Ҳ�ת����PWM����///
/// u,v,w ��� = v_busȫ������� 
/// ������ǵ�Դ��ѹ����������
/// �����������ĵ�ѹʸ��
/// ʵ��Ҳ�ǹ�ʽת�����ѣ����ۣ�
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


/// ʹ�������������Ի�������Сռ�ձȶ��ԣ�����������Ի��� ///
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
    
/// �������������������ƫ�� /// 
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

////��ʼ�����FOC������Ʋ���
void init_controller_params(ControllerStruct *controller)
{
    controller->ki_d = KI_D;
    controller->ki_q = KI_Q;
    controller->k_d = K_SCALE*I_BW;
    controller->k_q = K_SCALE*I_BW;
    controller->alpha = 1.0f - 1.0f/(1.0f - DT*I_BW*2.0f*PI);    
}

//��λ����FOC����,����dq�����������ͺ���
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

//��λ�¶ȹ۲���������λ�¶�ֵ
void reset_observer(ObserverStruct *observer)
{
    observer->temperature = 25.0f;
    observer->resistance = .1f;
}

//�ο����Ƶ����������޷�
void limit_current_ref (ControllerStruct *controller)
{
    float i_q_max_limit = (0.5774f*controller->v_bus - controller->dtheta_elec*WB)/R_PHASE;
    float i_q_min_limit = (-0.5774f*controller->v_bus - controller->dtheta_elec*WB)/R_PHASE;
    controller->i_q_ref = fmaxf(fminf(i_q_max_limit, controller->i_q_ref), i_q_min_limit);
}

//�����ջ�����
void commutate(ControllerStruct *controller, ObserverStruct *observer, GPIOStruct *gpio, float theta)
	{       
        /// �����¶ȹ۲�������ֵ ///////////////////////////////////////////////////////////////////////////////////////////////////////
        // �¶ȹ۲���//
        float t_rise = (float)observer->temperature - 25.0f;			//25���Ǳ�׼�ο��¶ȣ�����=ADCͨ�������߼�����¶�-��׼�ο��¶�
        float q_th_in = (1.0f + .00393f*t_rise)*(controller->i_d*controller->i_d*R_PHASE*SQRT3 + controller->i_q*controller->i_q*R_PHASE*SQRT3);
        float q_th_out = t_rise*R_TH;
        observer->temperature += INV_M_TH*DT*(q_th_in-q_th_out);
				// ���迹�迹�۲��� //
        observer->resistance = (controller->v_q - SQRT3*controller->dtheta_elec*(WB))/controller->i_q;
        if(isnan(observer->resistance)){observer->resistance = R_PHASE;}
        observer->temperature2 = (double)(25.0f + ((observer->resistance*6.0606f)-1.0f)*275.5f);
        double e = observer->temperature - observer->temperature2;
        observer->temperature -= .001*e;

				
				
       ///�����ջ�//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       controller->loop_count ++;   
	/*����һ���������������*/
       if(PHASE_ORDER)  //����ʱ�� �����������������ȷ��VU��λ˳��
				   {                                                                        
           controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);    // ����ADC�Ķ�������B�����
           controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);		 // ����ADC�Ķ�������C�����
           }
        else					 //����ʱ
					 {
            controller->i_b = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);    
            controller->i_c = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);
           }
						controller->i_a = -controller->i_b - controller->i_c;    															 	//���ݻ����������A�����   
       
	/*���ڶ���������Ҫ��DQ0�任 (park�任+clark�任)*/	 
       float s = FastSin(theta); 
       float c = FastCos(theta);                            
       dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //����dq�任��park�任+clark�任��
				
					 
	/*��������������ο���dq�����i_d_ref��i_q_ref*/	
        controller->i_q_filt = 0.95f*controller->i_q_filt + 0.05f*controller->i_q;
        controller->i_d_filt = 0.95f*controller->i_d_filt + 0.05f*controller->i_d;
        //����alpha�����Ƕ�
        controller->i_d_ref_filt = (1.0f-controller->alpha)*controller->i_d_ref_filt + controller->alpha*controller->i_d_ref;
        controller->i_q_ref_filt = (1.0f-controller->alpha)*controller->i_q_ref_filt + controller->alpha*controller->i_q_ref;
       /// ������ ///
       controller->fw_int += .001f*(0.5f*OVERMODULATION*controller->v_bus - controller->v_ref);
       controller->fw_int = fmaxf(fminf(controller->fw_int, 0.0f), -I_FW_MAX);
       controller->i_d_ref = controller->fw_int;
       limit_norm(&controller->i_d_ref, &controller->i_q_ref, I_MAX);//�����޷������Ŀ����ʹ�����ֵ�ڵ�Դ����I_MAXΪ�뾶��ԲȦ��
       
       
       
 /*�����Ĳ���PI ������������i_d_ref��i_d�����v_d��v_q*/
			 //��1������PI���������
       float i_d_error = controller->i_d_ref - controller->i_d;
       float i_q_error = controller->i_q_ref - controller->i_q;//  + cogging_current;
	  	 //����ǰ����ѹ��ǰ����ѹ=�����*�����//
			 // float v_d_ff = SQRT3*(1.0f*controller->i_d_ref*R_PHASE  - controller->dtheta_elec*L_Q*controller->i_q);   //feed-forward voltages
			 // float v_q_ff =  SQRT3*(1.0f*controller->i_q_ref*R_PHASE +  controller->dtheta_elec*(L_D*controller->i_d + 1.0f*WB));
       
       //��2������PI����������P���������I//
       controller->d_int += controller->k_d*controller->ki_d*i_d_error;  //ki_d��Ϊ��d��������棬k_d���ǵ���������
       controller->q_int += controller->k_q*controller->ki_q*i_q_error;	 //ki_q��Ϊʹq��������棬k_q���ǵ���������
       controller->d_int = fmaxf(fminf(controller->d_int, OVERMODULATION*controller->v_bus), - OVERMODULATION*controller->v_bus);
       controller->q_int = fmaxf(fminf(controller->q_int, OVERMODULATION*controller->v_bus), - OVERMODULATION*controller->v_bus); 
       controller->v_d = controller->k_d*i_d_error + controller->d_int ;//+ v_d_ff;  
       controller->v_q = controller->k_q*i_q_error + controller->q_int ;//+ v_q_ff; 
			 
       // ��3������ο���ѹ����v_d��v_q�������Ի�
       controller->v_ref = sqrt(controller->v_d*controller->v_d + controller->v_q*controller->v_q);
       limit_norm(&controller->v_d, &controller->v_q, OVERMODULATION*controller->v_bus);       //��ѹ�޷������Ŀ����ʹ�����ֵ�ڵ�Դ��ѹv_busΪ�뾶��ԲȦ��
       float dtc_d = controller->v_d/controller->v_bus;//dtc_d����ʹһ����ֵ���������Ի�
       float dtc_q = controller->v_q/controller->v_bus;
       linearize_dtc(&dtc_d);													 //ʹ�������������Ի�
       linearize_dtc(&dtc_q);
       controller->v_d = dtc_d*controller->v_bus;			 //d���ѹ
       controller->v_q = dtc_q*controller->v_bus;			 //q���ѹ
			 
 /*�����岽������Ҫ����DQ0�任 (��park�任+��clark�任)*/			 			 
       abc(controller->theta_elec + 0.0f*DT*controller->dtheta_elec, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //��ѹ��dq�任
 /*���������� �ռ�ʸ������ �����ʹ���Ҳ�ת����PWM����*/			 
       svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //�ռ�ʸ������
       if(PHASE_ORDER)													 							// ���UV��˳��
				 {                                                        
            TIM1->CCR3 = (PWM_ARR)*(1.0f-controller->dtc_u);  //дPWM��ռ�ձ�
            TIM1->CCR2 = (PWM_ARR)*(1.0f-controller->dtc_v);
            TIM1->CCR1 = (PWM_ARR)*(1.0f-controller->dtc_w);
        }
        else{
            TIM1->CCR3 = (PWM_ARR)*(1.0f-controller->dtc_u);
            TIM1->CCR1 = (PWM_ARR)*(1.0f-controller->dtc_v);
            TIM1->CCR2 =  (PWM_ARR)*(1.0f-controller->dtc_w);
        }
       controller->theta_elec = theta;   //��¼��Ƕ�                                       
    }
    
 //�������ؿ��� 
void torque_control(ControllerStruct *controller)
{
	//���ɲο������أ��ο�����ֵ=Kp(�û�������λ��-�������Ļ�еλ��)+�û����õ�����ֵ+Kd(�û��������ٶ�-���������ٶ�)
    float torque_ref = controller->kp*(controller->p_des - controller->theta_mech) + controller->t_ff + controller->kd*(controller->v_des - controller->dtheta_mech);
  //����q�����������òο������ظ��� 
		controller->i_q_ref = torque_ref/KT_OUT;    //i_q_ref��ת���йأ�����q��������=�ο�����/dq���Ч�ĵ���
  //����d��������    
		controller->i_d_ref = 0.0f;									//i_d_ref���ͨ�й�
}


