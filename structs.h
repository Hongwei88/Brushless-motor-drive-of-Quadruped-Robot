#ifndef STRUCTS_H
#define STRUCTS_H

#include "mbed.h"
#include "FastPWM.h"


typedef struct
{
    DigitalOut *enable;                                 // DRV832x Enable Pin
    DigitalOut *led;                                    // LED                    
    FastPWM *pwm_u, *pwm_v, *pwm_w;                     // Motor Control pwm output
}GPIOStruct;

    
typedef struct
{
    int adc1_raw, adc2_raw, adc3_raw;                       // ԭʼADC��ֵ
    float i_a, i_b, i_c;                                    // a,b,c�����
    float v_bus;                                            // ֱ�����ӵ�ѹ
    float theta_mech, theta_elec;                           // ��еת�ӣ���Ƕ�
    float dtheta_mech, dtheta_elec, dtheta_elec_filt;       // ��еת�ӣ�����ٶ�
    float i_d, i_q, i_q_filt, i_d_filt;                     // D/Q �����
    float v_d, v_q;                                         // D/Q ���ѹ
    float dtc_u, dtc_v, dtc_w;                              // �����ռ�ձ�
    float v_u, v_v, v_w;                                    // ����˵�ѹ
    float k_d, k_q, ki_d, ki_q, alpha;                      // ���������桢�������ο��˲���ϵ��
    float d_int, q_int;                                     // ����������
    int adc1_offset, adc2_offset;                           // ADC ƫ��
    float i_d_ref, i_q_ref, i_d_ref_filt, i_q_ref_filt;     // �ο�����
    int loop_count;                                         // ѭ��������
    int timeout;                                            // ���Ź�������
    int mode;																								// ���ģʽ
    int ovp_flag;                                           // ��ѹ��־λ
    float p_des, v_des, kp, kd, t_ff;                       // ����λ�á������ٶȡ�����ϵ����΢��ϵ��������
    float v_ref, fw_int;                                    // �����ѹ��ֵ�����Ż���
    float cogging[128];																			//�ݲ�
} ControllerStruct;

typedef struct
{
    double temperature;                                     //���Ƶ��¶�
    double temperature2;																		//�¶�
    float resistance;																				//�迹
} ObserverStruct;

#endif
