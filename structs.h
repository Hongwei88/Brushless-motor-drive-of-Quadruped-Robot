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
    int adc1_raw, adc2_raw, adc3_raw;                       // 原始ADC的值
    float i_a, i_b, i_c;                                    // a,b,c相电流
    float v_bus;                                            // 直流连接电压
    float theta_mech, theta_elec;                           // 机械转子，电角度
    float dtheta_mech, dtheta_elec, dtheta_elec_filt;       // 机械转子，电角速度
    float i_d, i_q, i_q_filt, i_d_filt;                     // D/Q 轴电流
    float v_d, v_q;                                         // D/Q 轴电压
    float dtc_u, dtc_v, dtc_w;                              // 输出端占空比
    float v_u, v_v, v_w;                                    // 输出端电压
    float k_d, k_q, ki_d, ki_q, alpha;                      // 电流环增益、电流环参考滤波器系数
    float d_int, q_int;                                     // 电流误差积分
    int adc1_offset, adc2_offset;                           // ADC 偏移
    float i_d_ref, i_q_ref, i_d_ref_filt, i_q_ref_filt;     // 参考电流
    int loop_count;                                         // 循环计数器
    int timeout;                                            // 看门狗计数器
    int mode;																								// 电机模式
    int ovp_flag;                                           // 过压标志位
    float p_des, v_des, kp, kd, t_ff;                       // 期望位置、期望速度、比例系数、微分系数、力矩
    float v_ref, fw_int;                                    // 输出电压幅值、弱磁积分
    float cogging[128];																			//齿槽
} ControllerStruct;

typedef struct
{
    double temperature;                                     //估计的温度
    double temperature2;																		//温度
    float resistance;																				//阻抗
} ObserverStruct;

#endif
