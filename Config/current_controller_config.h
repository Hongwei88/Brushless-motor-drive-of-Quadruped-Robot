#ifndef CURRENT_CONTROLLER_CONFIG_H
#define CURRENT_CONTROLLER_CONFIG_H

// ����������///
#define K_D .05f                    // ��·���棬����/����
#define K_Q .05f                    // ��·���棬����/����
#define K_SCALE 0.0001f             // K_loop/Loop BW (Hz) 0.0042
#define KI_D 0.0255f                // PI zero, ��λΪÿ�������Ļ���
#define KI_Q 0.0255f                // PI zero, i��λΪÿ�������Ļ���
#define V_BUS 24.0f                 // ��ѹ
#define OVERMODULATION 1.15f        // 1.0 = no overmodulation  �޹�����

#define D_INT_LIM V_BUS/(K_D*KI_D)  // ����*����ֵ
#define Q_INT_LIM V_BUS/(K_Q*KI_Q)  // ����*����ֵ

//Observer//
#define DT 0.000025f
#define K_O 0.02f




#endif
