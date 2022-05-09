
/*
*储存在flash中的值，可以由用户操作修改
*/
#ifndef USER_CONFIG_H
#define USER_CONFIG_H


#define E_OFFSET                __float_reg[0]      // 编码器电角度偏移 
#define M_OFFSET                __float_reg[1]      // 编码器机械角度偏移
#define I_BW                    __float_reg[2]      // 电流环带宽
#define I_MAX                   __float_reg[3]      // 输出力矩限制 (current limit = torque_limit/(kt*gear ratio))
#define THETA_MIN               __float_reg[4       // 最小位置设定值
#define THETA_MAX               __float_reg[5]      // 最大位置设定值
#define I_FW_MAX                __float_reg[6]      // 最大弱磁电流

#define PHASE_ORDER             __int_reg[0]        // 校准期间的相位变换
#define CAN_ID                  __int_reg[1]        // CAN 总线上的 ID
#define CAN_MASTER              __int_reg[2]        // CAN 总线主设备的 ID
#define CAN_TIMEOUT             __int_reg[3]        // CAN 总线超时时间
#define ENCODER_LUT             __int_reg[5]        // 编码器偏移LUT-128个元素长度


extern float __float_reg[];
extern int __int_reg[];

#endif
