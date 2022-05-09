#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#define R_PHASE     0.14f       //欧姆，线圈电阻
#define L_D         0.000017f   //D轴电感，单位亨利
#define L_Q         0.000017f   //Q轴电感，单位亨利
#define KT          0.08f       //峰值相电流N-m=WB*NPP*3/2
#define NPP         21          //线圈极对数
#define GR          1.0f        //齿轮传动比
#define KT_OUT      0.45f       //KT*GR,峰值相电流*齿轮传动比=dq轴等效的电流
#define WB          0.0025f     //磁链，单位韦伯
#define R_TH        1.25f       //开尔文/瓦
#define INV_M_TH    0.03125f    //开尔文/焦耳


#endif
