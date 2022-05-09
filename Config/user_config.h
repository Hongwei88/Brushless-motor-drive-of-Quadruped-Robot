
/*
*������flash�е�ֵ���������û������޸�
*/
#ifndef USER_CONFIG_H
#define USER_CONFIG_H


#define E_OFFSET                __float_reg[0]      // ��������Ƕ�ƫ�� 
#define M_OFFSET                __float_reg[1]      // ��������е�Ƕ�ƫ��
#define I_BW                    __float_reg[2]      // ����������
#define I_MAX                   __float_reg[3]      // ����������� (current limit = torque_limit/(kt*gear ratio))
#define THETA_MIN               __float_reg[4       // ��Сλ���趨ֵ
#define THETA_MAX               __float_reg[5]      // ���λ���趨ֵ
#define I_FW_MAX                __float_reg[6]      // ������ŵ���

#define PHASE_ORDER             __int_reg[0]        // У׼�ڼ����λ�任
#define CAN_ID                  __int_reg[1]        // CAN �����ϵ� ID
#define CAN_MASTER              __int_reg[2]        // CAN �������豸�� ID
#define CAN_TIMEOUT             __int_reg[3]        // CAN ���߳�ʱʱ��
#define ENCODER_LUT             __int_reg[5]        // ������ƫ��LUT-128��Ԫ�س���


extern float __float_reg[];
extern int __int_reg[];

#endif
