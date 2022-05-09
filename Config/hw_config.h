#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#define PIN_U PA_10								//������IC��PWMA�ź�����
#define PIN_V PA_9								//������IC��PWMB�ź�����
#define PIN_W PA_8								//������IC��PWMC�ź�����

#define ENABLE_PIN  PA_11        // ʹ������IC������
#define LED         PC_5         // LED ����
#define I_SCALE 0.02014160156f   //AD����ֵ��1��Ӧ�İ���������������õ�
#define V_SCALE 0.012890625f     //AD����ֵ��1��Ӧ�ĵ�ѹ������ѹ����õ� 
#define DTC_MAX 0.94f            //�����λռ�ձ�
#define DTC_MIN 0.0f             //��С��λռ�ձ�
#define PWM_ARR 0x8CA            //��ʱ���Զ�����2250

static float inverter_tab[16] = {2.5f, 2.4f, 2.3f, 2.2f, 2.1f, 2.0f, 1.9f, 1.8f, 1.7f, 1.6f, 1.59f, 1.58f, 1.57f, 1.56f, 1.55f, 1.5f};


#endif
