#ifndef CURRENT_CONTROLLER_CONFIG_H
#define CURRENT_CONTROLLER_CONFIG_H

// 电流控制器///
#define K_D .05f                    // 环路增益，伏特/安培
#define K_Q .05f                    // 环路增益，伏特/安培
#define K_SCALE 0.0001f             // K_loop/Loop BW (Hz) 0.0042
#define KI_D 0.0255f                // PI zero, 单位为每个样本的弧度
#define KI_Q 0.0255f                // PI zero, i单位为每个样本的弧度
#define V_BUS 24.0f                 // 电压
#define OVERMODULATION 1.15f        // 1.0 = no overmodulation  无过调制

#define D_INT_LIM V_BUS/(K_D*KI_D)  // 安培*采样值
#define Q_INT_LIM V_BUS/(K_Q*KI_Q)  // 安培*采样值

//Observer//
#define DT 0.000025f
#define K_O 0.02f




#endif
