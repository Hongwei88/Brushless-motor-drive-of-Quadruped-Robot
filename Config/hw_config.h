#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#define PIN_U PA_10								//给驱动IC的PWMA信号引脚
#define PIN_V PA_9								//给驱动IC的PWMB信号引脚
#define PIN_W PA_8								//给驱动IC的PWMC信号引脚

#define ENABLE_PIN  PA_11        // 使能驱动IC的引脚
#define LED         PC_5         // LED 引脚
#define I_SCALE 0.02014160156f   //AD计数值加1对应的安培数，电流检测用的
#define V_SCALE 0.012890625f     //AD计数值加1对应的电压数，电压检测用的 
#define DTC_MAX 0.94f            //最大相位占空比
#define DTC_MIN 0.0f             //最小相位占空比
#define PWM_ARR 0x8CA            //定时器自动读数2250

static float inverter_tab[16] = {2.5f, 2.4f, 2.3f, 2.2f, 2.1f, 2.0f, 1.9f, 1.8f, 1.7f, 1.6f, 1.59f, 1.58f, 1.57f, 1.56f, 1.55f, 1.5f};


#endif
