/// �����˸ߴ������������ƹ���
/// TI DRV8323����оƬ�汾
/*
*ʹ�ô��ڽ����������У׼��ͨ��mָ���������FOC����ģʽ��ͨ��CANָ��Ϳ��Ƶ���Ĳ�����
��1��λ��ָ�position command
��2���ٶ�ָ�velocity command
��3��ǰ������ feed forward torque
��4���������� kp
��5��΢�ֲ��� kd
CAN�ӿڵ�ָ����غ����μ�CAN_com.cpp�ļ��е�unpack_cmd(CANMessage msg, ControllerStruct * controller)

CANͨѶЭ��鿴main.cpp�ļ��е�void onMsgReceived()����
*/

#define REST_MODE 0
#define CALIBRATION_MODE 1
#define MOTOR_MODE 2
#define SETUP_MODE 4
#define ENCODER_MODE 5

#define VERSION_NUM "1.9"


float __float_reg[64];      // ��flash�ڶ���ĸ������ڴ�
int __int_reg[256];         // ��flash�ڶ���������ڴ棬����λ�ô�����У׼�Ĳ��ұ�

#include "mbed.h"
#include "PositionSensor.h"
#include "structs.h"
#include "foc.h"
#include "calibration.h"
#include "hw_setup.h"
#include "math_ops.h" 
#include "current_controller_config.h"
#include "hw_config.h"
#include "motor_config.h"
#include "stm32f4xx_flash.h"
#include "FlashWriter.h"
#include "user_config.h"
#include "PreferenceWriter.h"
#include "CAN_com.h"
#include "DRV.h"
 
//���ֽṹ�弰��Ķ���
PreferenceWriter    prefs(6);												//��"FlashWriter.h"�ļ��ж��壬�ڴ��ȡ\д��\����\�رյı�������һ����
GPIOStruct          gpio;                           //�ڡ�struct.h"���壬PWM��ʹ��ADC��LED GPIO����һ���ṹ��
ControllerStruct    controller;                     //�ڡ�struct.h"���壬FOC���Ƶģ���һ���ṹ��
ObserverStruct      observer;                       //�ڡ�struct.h"���壬����¶ȼ��ģ���һ���ṹ��
Serial              pc(PA_2, PA_3);                 //mbed�ײ�Serial.h�����崮�ڵ�TX,RX����һ����
CAN                 can(PB_8, PB_9, 1000000);       //mbed�ײ�CAN.h�� ����CAN Rx, CAN Tx����һ����
CANMessage          rxMsg;                          //mbed�ײ�CAN.h�� CAN rx ��Ϣ����һ����
CANMessage          txMsg;                          //mbed�ײ�CAN.h�� CAN tx ��Ϣ����һ����
SPI                 drv_spi(PA_7, PA_6, PA_5);      //mbed�ײ�SPI.h�� ����оƬ����ͨѶ����һ����
DigitalOut          drv_spi_cs(PA_4);               //mbed�ײ�DigitalOut.h�� �����������оƬDrv spi cs pin.����һ����
DRV832x             drv(&drv_spi, &drv_spi_cs);     //�ڡ�DVR.h�� �ж��壬������ʼ������оƬ�����ŵ�
PositionSensorAM5147 Encoder_spi(16384, 0.0, NPP);  //�ڡ�PositionSensor���ж��壬 �ű�������SPIͨѶ

volatile int count = 0;						
volatile int state = REST_MODE;				//�����ģʽ��Ĭ�ϵ��û��ģʽ
volatile int state_change;						//���ģʽ���ı�־λ��state_change=1�����Ը��ĵ��ģʽ��֮���ܡ�


/*����˵�ָʾ����*/
//����ֻ�д��ڴ�ӡ����
//��TIM�ж��н��е�����
void enter_menu_state(void)
{
    drv.disable_gd();
    printf("\n\r\n\r\n\r");
    printf(" Commands:\n\r");
    wait_us(10);
    printf(" m - Motor Mode\n\r");
    wait_us(10);
    printf(" c - Calibrate Encoder\n\r");
    wait_us(10);
    printf(" s - Setup\n\r");
    wait_us(10);
    printf(" e - Display Encoder\n\r");
    wait_us(10);
    printf(" z - Set Zero Position\n\r");
    wait_us(10);
    printf(" esc - Exit to Menu\n\r");
    wait_us(10);
    state_change = 0;
    gpio.led->write(0);
}

/*����ģʽ����ָ���*/
//����ֻ�д��ڴ�ӡ����
//��TIM�ж��н��е�����
void enter_setup_state(void)
{
    printf("\n\r\n\r Configuration Options \n\r\n\n");
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %-2s\n\r\n\r", "prefix", "parameter", "min", "max", "current value");
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "b", "Current Bandwidth (Hz)", "100", "2000", I_BW);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %-5i\n\r", "i", "CAN ID", "0", "127", CAN_ID);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %-5i\n\r", "m", "CAN Master ID", "0", "127", CAN_MASTER);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "l", "Current Limit (A)", "0.0", "40.0", I_MAX);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "f", "FW Current Limit (A)", "0.0", "33.0", I_FW_MAX);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %d\n\r", "t", "CAN Timeout (cycles)(0 = none)", "0", "100000", CAN_TIMEOUT);
    wait_us(10);
    printf("\n\r To change a value, type 'prefix''value''ENTER'\n\r i.e. 'b1000''ENTER'\n\r\n\r");
    wait_us(10);
    state_change = 0;
}

/*�������ؿ���ģʽ*/   
//���ж��н��е�����
void enter_torque_mode(void)
{
    drv.enable_gd();
    //gpio.enable->write(1);
    controller.ovp_flag = 0;												//
    reset_foc(&controller);                         // ��λFOC�����Ի��������������ƻ�·���� [�ص㿴������]
    wait(.001);
    controller.i_d_ref = 0;
    controller.i_q_ref = 0;                         //�����趨ֵ
    gpio.led->write(1);                             //��״̬LED
    state_change = 0;																//������λһ�Σ�����Ϊ0���´�ֱ�ӻؽ�ȥ����ģʽ
    printf("\n\r Entering Motor Mode \n\r");
}

/*������У������*/ 
//��TIM�ж��н��е�����
//����������ʼ���н��е�����
void calibrate(void)
{
    drv.enable_gd();																					 //����ʹ�ܣ�ʹ���е�mos�ܴ���Hi-Z״̬	
    gpio.led->write(1);                            						 //��״̬LED
    order_phases(&Encoder_spi, &gpio, &controller, &prefs);    //��鲢������λ˳����λУ׼��	
    calibrate(&Encoder_spi, &gpio, &controller, &prefs);       //ִ��У׼����ƫ����У׼=�����Ի�У׼��
    gpio.led->write(0);                                        //�ر�״̬LED
    wait(.2);
    printf("\n\r Calibration complete.  Press 'esc' to return to menu\n\r");		//У׼��ɺ���ʾ�˳����˵���
    drv.disable_gd();																					 //����ʧ�ܣ�ʹ���е�mos�ܴ��ڷ�Hi-Z״̬	
    state_change = 0;
}

/*�����������ֵ����*/
//��TIM�ж��н��е�����
void print_encoder(void)
{
    printf(" Mechanical Angle:  %f    Electrical Angle:  %f    Raw:  %d\n\r", Encoder_spi.GetMechPosition(), Encoder_spi.GetElecPosition(), Encoder_spi.GetRawPosition());
    wait(0.001);
}
////////////////////////////////////////////////////////////////////
//////////////////////////�жϺ���//////////////////////////////////
////////////////////////////////////////////////////////////////////



/*
*���������жϲ��ܵ������������ʲôģʽ������40KHZ����
*����ʹ��ʱ���ж�
*/
extern "C" 
void TIM1_UP_TIM10_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_UIF )
    {
        ADC1->CR2  |= 0x40000000;                   //��ʼ���õ�����ת���������¶�ֵ
        Encoder_spi.Sample(DT);                     //����λ�ô�����
			
				///����оƬ����������� ,������ȡֵ��///
        controller.adc2_raw = ADC2->DR;             //��ȡADC���ݼĴ�������������оƬ�Ͽ��Զ������������ĵ�ѹֵ
        controller.adc1_raw = ADC1->DR;
        controller.adc3_raw = ADC3->DR;							//�����Դ��ѹ
			
				///�ű���������Ƕȡ���е�Ƕȣ�λ�á��ٶȼ�� ,������ȡֵ��///
        controller.theta_elec = Encoder_spi.GetElecPosition();   
        controller.theta_mech = (1.0f/GR)*Encoder_spi.GetMechPosition();
        controller.dtheta_mech = (1.0f/GR)*Encoder_spi.GetMechVelocity();  
        controller.dtheta_elec = Encoder_spi.GetElecVelocity();
			
			  ///��������ѹ������ֱ����·��ѹ����ֵ///
        controller.v_bus = 0.95f*controller.v_bus + 0.05f*((float)controller.adc3_raw)*V_SCALE; 
        
        /// ���״̬��״̬���������ʵ��ĺ��� ///
        switch(state)
        {
            case REST_MODE:                //��������˵����ڴ�ӡ��������ѡ����ģ��ڴ����жϽ����и���
                if(state_change)   //�ڸ�λģʽ�£����ĵ��ģʽ��Ҫ��ȥ�˵�����������
                {
                    enter_menu_state();
                }
                break;
            
            case CALIBRATION_MODE:        //���б�����У׼����
                if(state_change)		//�ڱ�����У��ģʽ�£�������ĵ��״̬����Ҫ����У׼
                {
                    calibrate();
                }
                break;
             
            case MOTOR_MODE:              //�������ؿ��Ƴ���
                if(state_change)	//�����ؿ�����������ģʽ�£�����������״̬����λFOC�ĸ�������
                {
                    enter_torque_mode();
                    count = 0;
                }
								//
                else
                {
                    /*
                    if(controller.v_bus>28.0f)   //������ߵĵ�ѹ���ߣ���ת��դ���������Է�ֹ���������������߱��жϷ���FET��ЧӦ�ܱ�ը������һ�ֱ���
									     {        
                        gpio.
                        ->write(0);
                        controller.ovp_flag = 1;
                        state = REST_MODE;
                        state_change = 1;
                        printf("OVP Triggered!\n\r");
                        }
                     */  
                    
                    if((controller.timeout > CAN_TIMEOUT) && (CAN_TIMEOUT > 0))//��������·��ʱ����CANͨѶ��ʱ�������൱�ڵ�������������һ�ֱ���
                    {
                        controller.i_d_ref = 0;
                        controller.i_q_ref = 0;
                        controller.kp = 0;
                        controller.kd = 0;
                        controller.t_ff = 0;
                    } 

                    torque_control(&controller);																								//�������ؿ��ƣ��ص㿴,foc���ļ��У�
                    commutate(&controller, &observer, &gpio, controller.theta_elec);           // ���е����ջ�

                    controller.timeout++;
                    count++; 
                }     
                break;
                
            case SETUP_MODE:										/*����ģʽ����ָ���������ֻ�д��ڴ�ӡ����*/
                if(state_change)
                {
                    enter_setup_state();
                }
                break;
                
            case ENCODER_MODE:								//�������ģʽ�������������������
                print_encoder();
                break;
            
            default:
                break;
        }                 
    }
    TIM1->SR = 0x00;                       // ��λ״̬�Ĵ���
}
//


char cmd_val[8] = {0};
char cmd_id = 0;
char char_count = 0;

/*
*ʹ�����Դ��������ն˻�������GUI������״̬��
*�����н���������ɵ�ʱ�����
*�����Ǵ����ж�
*/
void serial_interrupt(void)
{
    while(pc.readable())				//�ȴ����ڽ��յ�����
    {
        char c = pc.getc();			//ת�洮�ڽ��յ����ַ����䵱��־λ
        if(c == 27)	                  //���ģʽ��λ������ͨѶ��8λ��������						
        {
            state = REST_MODE;			 //����Ϊ�����λ״̬������ָ��ǿ�Ƶ����λ
            state_change = 1;
            char_count = 0;
            cmd_id = 0;
            gpio.led->write(0);; 
            for(int i = 0; i<8; i++) //��λ��λ����
            {
                cmd_val[i] = 0;
            }
        }
        
        if(state == REST_MODE)      	//�����������ĸ�λ״̬�£���������е����ģʽ���ã�����ϵ�������һ�ε�	
        {
            switch (c)
            {
            case 'c':											//����Ϊ�ڱ�����У��״̬
                state = CALIBRATION_MODE;
                state_change = 1;
                break;
            
            case 'm':											//����Ϊ�������ģʽ
                state = MOTOR_MODE;
                state_change = 1;
                break;
            
            case 'e':											//����Ϊ���������ģʽ
                state = ENCODER_MODE;
                state_change = 1;
                break;
            
            case 's':											//����Ϊ�����������ģʽ
                state = SETUP_MODE;
                state_change = 1;
                break;
            
            case 'z':									  	//����Ϊ��е��λΪ��ǰ������λ��
                Encoder_spi.SetMechOffset(0);
                Encoder_spi.Sample(DT);
                wait_us(20);
                M_OFFSET = Encoder_spi.GetMechPosition();
                if (!prefs.ready())//�ڴ��ʼ��
                    prefs.open();
                prefs.flush();          // Write new prefs to flash
                prefs.close();    
                prefs.load(); 
                Encoder_spi.SetMechOffset(M_OFFSET);
                printf("\n\r  Saved new zero position:  %.4f\n\r\n\r", M_OFFSET);
                break;
                
            default:
                break;
            }             
        }
        else if(state == SETUP_MODE)	//�ڵ����������ģʽ
        {
            if(c == 13)
            {
                switch (cmd_id)
                {
                    case 'b':		// ����������
                        I_BW = fmaxf(fminf(atof(cmd_val), 2000.0f), 100.0f);
                        break;
                    
                    case 'i':		//CAN �����ϵ���� ID
                        CAN_ID = atoi(cmd_val);
                        break;
                    
                    case 'm':		//CAN �������豸�� ID
                        CAN_MASTER = atoi(cmd_val);
                        break;
                    
                    case 'l':		// ����������� (current limit = torque_limit/(kt*gear ratio))
                        I_MAX = fmaxf(fminf(atof(cmd_val), 40.0f), 0.0f);
                        break;
                    
                    case 'f':		// ������ŵ���
                        I_FW_MAX = fmaxf(fminf(atof(cmd_val), 33.0f), 0.0f);
                        break;
                    
                    case 't':		// CAN ���߳�ʱʱ��
                        CAN_TIMEOUT = atoi(cmd_val);
                        break;
                    
                    default:
                        printf("\n\r '%c' Not a valid command prefix\n\r\n\r", cmd_id);
                        break;
                }  
                if (!prefs.ready())
                    prefs.open();
                prefs.flush();      // Write new prefs to flash
                prefs.close();    
                prefs.load();                                              
                state_change = 1;
                char_count = 0;
                cmd_id = 0;
                for(int i = 0; i<8; i++)	//��λ��λ����
                {
                    cmd_val[i] = 0;
                }
            }
            else
            {
                if(char_count == 0)
                {
                    cmd_id = c;
                }
                else
                {
                    cmd_val[char_count-1] = c; 
                }
                pc.putc(c);
                char_count++;
            }
        }
        else if (state == ENCODER_MODE)	//�ڱ��������ģʽ��
        {
            switch (c)
            {
                case 27:
                    state = REST_MODE;
                    state_change = 1;
                    break;
                
                default:
                    break;
            }
        }
        else if (state == MOTOR_MODE)		//�ڵ�����ģʽ��
        {
            switch (c)
            {
                case 'd':
                    controller.i_q_ref = 0;
                    controller.i_d_ref = 0;
                    break;
                
                default:
                    break;
            }
        }
            
    }
}
//



//��CAN������ɡ��жϷ�����
void onMsgReceived()
{
    printf("%df\n", rxMsg.id);//���������豸��Ӧ��ָ������Ǹ����
    can.read(rxMsg);          //�����ݲ��洢��������
    
    if(rxMsg.id == CAN_ID)    //�յ���ID�Ǳ������Ӧ��ID,�������ID�Ѿ����ô���flash������
    {
        controller.timeout = 0;	//��λCANͨѶ��ʱʱ�䣬ι��
			
			//��������������ָ��
			//������ģʽ[OxFF, OxFF, OxFF, OxFF, OxFF, OxFF, OxFF, OxFC]
        if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) & (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFC)))
        {
            state = MOTOR_MODE;//�ֶ��õ��ֹͣ��������dq��ĵ�ѹΪ0�������ֶ������
            state_change = 1;
        }
				
			//�˳����ģʽ[OxFF, OxFF, OxFF, OxFF, OxFF, OxFF, OxFF, OxFD]
        else if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFD)))
        {
            state = REST_MODE;//ģʽûѡ��״̬
            state_change = 1;
            gpio.led->write(0);//������оƬʧ�ܣ������
        }
				
			//���û�еλ�ô�����λ0��[OxFF, OxFF, OxFF, OxFF, OxFF, OxFF, OxFF, OxFE]
        else if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFE)))
        {
            Encoder_spi.ZeroPosition();//�ֶ�У����������λ��ֵ
        }
			
				//ִ�е������������ģʽ��������������������ָ�
        else if(state == MOTOR_MODE)
        {
          unpack_cmd(rxMsg, &controller);//CAN�ӿڵ�ָ�����,��CAN�Ĵ�����������ݰᵽ������Ʋ������ڴ���
        }
        pack_reply(&txMsg, controller.theta_mech, controller.dtheta_mech, controller.i_q_filt*KT_OUT);//���յ������ݷ��ظ����豸�������Լ��յ��ˣ�����������
        can.write(txMsg);	//CAN����
    }
}
//



//////////////////////////////////////////////////////////////////////
//////////////////////////////////������//////////////////////////////
//////////////////////////////////////////////////////////////////////
uint16_t ram_encoder_data = 0;		//���������ݵ��ڴ����
int main()
{
/*��ʼ��Ƭ�������*/
    controller.v_bus = V_BUS;			//���õ�ѹ24V
    controller.mode = 0;					//��û���õ��ģʽ
    Init_All_HW(&gpio);           //��ʼ��PWM, ADC, LEDӲ��Ƭ������
    wait(0.1);  
	
/*��ʼ������оƬ��*/
    gpio.enable->write(1);        //ʹ��Drv8323x����оƬ��DRV832xʹ���������ߣ�mbed�ײ��д������������������ʼ������
    wait_us(100);
    drv.calibrate();               //����ICƫ��У׼�ĵ������Ŵ���ABC�Ķ�����
    wait_us(100);
    drv.write_DCR(0x0, 0x0, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);                 //�������������ƵļĴ���,���忴����IC�������ֲ�
    wait_us(100);
    drv.write_CSACR(0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x0, 0x0, 0x0, SEN_LVL_1_0);       //���õ������Ŵ����Ĵ���
    wait_us(100);
    drv.write_OCPCR(TRETRY_4MS, DEADTIME_200NS, OCP_RETRY, OCP_DEG_8US, VDS_LVL_1_88);  //���ù����������Ĵ���
    zero_current(&controller.adc1_offset, &controller.adc2_offset);                     //�����������������Ư��
    drv.disable_gd();																																		// IC״̬ʧ��
    wait(0.1);
/*��ʼ��FOC�㷨*/
    reset_foc(&controller);             // ��λ����������
    reset_observer(&observer);          // ��λ�۲���
/*��ʼ��TIM��ʱ���ж�*/
    TIM1->CR1 ^= TIM_CR1_UDIS;          // �ر��ж�
    //TIM1->CR1 |= TIM_CR1_UDIS;        // ʹ���ж� 
    wait(0.1);  
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2);    // �����ն���������
/*��ʼ��CAN�����ж�*/
    NVIC_SetPriority(CAN1_RX0_IRQn, 3);
    can.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0);                                 
    txMsg.id = CAN_MASTER;
    txMsg.len = 6;
    rxMsg.len = 8;
    can.attach(&onMsgReceived);         //�������ӡ�CAN���մ����жϴ�����  
/*��ʼ��������Ʋ���*/		
    //����û�û��������ѡ�������Ĭ��״̬
		//������flash�е�ֵ���������û������޸�
    prefs.load();                       //�ȶ� flash��������ݣ����ڳ�ʼ����ʱ�����õ������
		//��������Ըĵ�����������ڣ�
    if(isnan(E_OFFSET))                         {E_OFFSET = 0.0f;}// ���ñ�������Ƕ�ƫ�� 
    if(isnan(M_OFFSET))                         {M_OFFSET = 0.0f;}// ���ñ�������е�Ƕ�ƫ��
    if(isnan(I_BW) || I_BW==-1)                 {I_BW = 1000;}		// ���õ���������
    if(isnan(I_MAX) || I_MAX ==-1)              {I_MAX = 15;}			// ��������������� (current limit = torque_limit/(kt*gear ratio))������������������
    if(isnan(I_FW_MAX) || I_FW_MAX ==-1)        {I_FW_MAX=0;}			// ����������ŵ���
    if(isnan(CAN_ID) || CAN_ID==-1)             {CAN_ID = 1;}			// ����CAN �����ϵ���� ID
    if(isnan(CAN_MASTER) || CAN_MASTER==-1)     {CAN_MASTER = 0;}	// ����CAN �������豸�� ID
    if(isnan(CAN_TIMEOUT) || CAN_TIMEOUT==-1)   {CAN_TIMEOUT = 0;}// ����CAN ���߳�ʱʱ��
		init_controller_params(&controller);													//��ʼ�����FOC������Ʋ���
/*��ʼ��������λ�ã��ôű������ṩ��API���У�*/  
    Encoder_spi.SetElecOffset(E_OFFSET);        // ���ñ�������Ƕ�ƫ��
    Encoder_spi.SetMechOffset(M_OFFSET);        // ���ñ�������е�Ƕ�ƫ��
    int lut[128] = {0};													// ������ƫ��LUT-128��Ԫ�س��ȱ�
    memcpy(&lut, &ENCODER_LUT, sizeof(lut));  	// ���ư�����
    Encoder_spi.WriteLUT(lut);                  // ����λ�ô����������Բ��ұ�
/*��ʼ�������жϣ�����ӡ��������Ϣ*/
    pc.baud(921600);                            //λ�ô��ڲ�����
    wait(0.01);
    pc.printf("\n\r\n\r HobbyKing Cheetah\n\r\n\r");
    wait(0.01);
    printf("\n\r Debug Info:\n\r");
    printf(" Firmware Version: %s\n\r", VERSION_NUM);
    printf(" ADC1 Offset: %d    ADC2 Offset: %d\n\r", controller.adc1_offset, controller.adc2_offset);	//��ʾADC������ѹֵ
    printf(" Position Sensor Electrical Offset:   %.4f\n\r", E_OFFSET);																	//��ʾ�������⵽�ĵ�Ƕ�
    printf(" Output Zero Position:  %.4f\n\r", M_OFFSET);																								//��ʾ�������⵽�Ļ�е�Ƕ�
    printf(" CAN ID:  %d\n\r", CAN_ID);																																	//��ʾCANͨѶID
    pc.attach(&serial_interrupt);               																												//���Ӵ����ж�
//��ʼѡ����ģʽ   
		state_change = 1;
    while(1)
    {

        wait(0.1);                                  		 // �ȴ� 100ms
        ram_encoder_data = Encoder_spi.GetRawPosition(); // ��ȡ������ʵʱλ��ת��
//      drv.print_faults();        											 // �����õģ� ʵʱ��ʾ����������״̬��MIT����IC��faultsӦ���ǽ��˵�Ƭ����
//      printf("%.4f\n\r", controller.v_bus);						 // �����õģ�ʵʱ��ʾ�����ѹ���� ����ѹ�����ADC��ͨ����ʱ������     
       /*
        if(state == MOTOR_MODE)//�����õ�
        {
            //printf("%.3f  %.3f  %.3f\n\r", (float)observer.temperature, (float)observer.temperature2, observer.resistance);//ʵʱ��ʾ����ĵ�
            //printf("%.3f  %.3f  %.3f %.3f %.3f\n\r", controller.v_d, controller.v_q, controller.i_d_filt, controller.i_q_filt, controller.dtheta_elec);
            //printf("%.3f\n\r", controller.dtheta_mech);
            wait(0.002);
        }
        */
    }
}
