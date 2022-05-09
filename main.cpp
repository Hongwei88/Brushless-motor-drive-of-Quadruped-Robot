/// 机器人高带宽三相电机控制工程
/// TI DRV8323驱动芯片版本
/*
*使用串口进行完编码器校准后，通过m指令进入电机的FOC控制模式，通过CAN指令发送控制电机的参数：
（1）位置指令：position command
（2）速度指令：velocity command
（3）前馈力矩 feed forward torque
（4）比例参数 kp
（5）微分参数 kd
CAN接口的指令相关函数参见CAN_com.cpp文件中的unpack_cmd(CANMessage msg, ControllerStruct * controller)

CAN通讯协议查看main.cpp文件中的void onMsgReceived()函数
*/

#define REST_MODE 0
#define CALIBRATION_MODE 1
#define MOTOR_MODE 2
#define SETUP_MODE 4
#define ENCODER_MODE 5

#define VERSION_NUM "1.9"


float __float_reg[64];      // 在flash内定义的浮动型内存
int __int_reg[256];         // 在flash内定义的整形内存，包括位置传感器校准的查找表

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
 
//各种结构体及类的定义
PreferenceWriter    prefs(6);												//在"FlashWriter.h"文件中定义，内存读取\写入\加载\关闭的变量，是一个类
GPIOStruct          gpio;                           //在“struct.h"定义，PWM灯使能ADC，LED GPIO，是一个结构体
ControllerStruct    controller;                     //在“struct.h"定义，FOC控制的，是一个结构体
ObserverStruct      observer;                       //在“struct.h"定义，电机温度检测的，是一个结构体
Serial              pc(PA_2, PA_3);                 //mbed底层Serial.h，定义串口的TX,RX，是一个类
CAN                 can(PB_8, PB_9, 1000000);       //mbed底层CAN.h， 定义CAN Rx, CAN Tx，是一个类
CANMessage          rxMsg;                          //mbed底层CAN.h， CAN rx 信息，是一个类
CANMessage          txMsg;                          //mbed底层CAN.h， CAN tx 信息，是一个类
SPI                 drv_spi(PA_7, PA_6, PA_5);      //mbed底层SPI.h， 驱动芯片引脚通讯，是一个类
DigitalOut          drv_spi_cs(PA_4);               //mbed底层DigitalOut.h， 数字输出驱动芯片Drv spi cs pin.，是一个类
DRV832x             drv(&drv_spi, &drv_spi_cs);     //在”DVR.h“ 中定义，用来初始化驱动芯片的引脚的
PositionSensorAM5147 Encoder_spi(16384, 0.0, NPP);  //在”PositionSensor“中定义， 磁编码器的SPI通讯

volatile int count = 0;						
volatile int state = REST_MODE;				//电机的模式，默认电机没有模式
volatile int state_change;						//电机模式更改标志位，state_change=1，可以更改电机模式反之不能。


/*进入菜单指示函数*/
//仅仅只有串口打印功能
//在TIM中断中进行调用了
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

/*进入模式设置指令函数*/
//仅仅只有串口打印功能
//在TIM中断中进行调用了
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

/*进入力矩控制模式*/   
//在中断中进行调用了
void enter_torque_mode(void)
{
    drv.enable_gd();
    //gpio.enable->write(1);
    controller.ovp_flag = 0;												//
    reset_foc(&controller);                         // 复位FOC，测试积分器和其他控制回路参数 [重点看看这里]
    wait(.001);
    controller.i_d_ref = 0;
    controller.i_q_ref = 0;                         //电流设定值
    gpio.led->write(1);                             //打开状态LED
    state_change = 0;																//仅仅复位一次，设置为0后下次直接回进去力矩模式
    printf("\n\r Entering Motor Mode \n\r");
}

/*编码器校正函数*/ 
//在TIM中断中进行调用了
//在主函数初始化中进行调用了
void calibrate(void)
{
    drv.enable_gd();																					 //驱动使能，使所有的mos管处于Hi-Z状态	
    gpio.led->write(1);                            						 //打开状态LED
    order_phases(&Encoder_spi, &gpio, &controller, &prefs);    //检查并设置相位顺序（相位校准）	
    calibrate(&Encoder_spi, &gpio, &controller, &prefs);       //执行校准程序（偏移量校准=非线性化校准）
    gpio.led->write(0);                                        //关闭状态LED
    wait(.2);
    printf("\n\r Calibration complete.  Press 'esc' to return to menu\n\r");		//校准完成后提示退出到菜单栏
    drv.disable_gd();																					 //驱动失能，使所有的mos管处于非Hi-Z状态	
    state_change = 0;
}

/*输出编码器的值函数*/
//在TIM中断中进行调用了
void print_encoder(void)
{
    printf(" Mechanical Angle:  %f    Electrical Angle:  %f    Raw:  %d\n\r", Encoder_spi.GetMechPosition(), Encoder_spi.GetElecPosition(), Encoder_spi.GetRawPosition());
    wait(0.001);
}
////////////////////////////////////////////////////////////////////
//////////////////////////中断函数//////////////////////////////////
////////////////////////////////////////////////////////////////////



/*
*电流采样中断不管电机控制器处于什么模式，都以40KHZ运行
*类型使定时器中断
*/
extern "C" 
void TIM1_UP_TIM10_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_UIF )
    {
        ADC1->CR2  |= 0x40000000;                   //开始采用电流并转换，表征温度值
        Encoder_spi.Sample(DT);                     //采样位置传感器
			
				///驱动芯片电流采样检测 ,给后面取值用///
        controller.adc2_raw = ADC2->DR;             //读取ADC数据寄存器，，在驱动芯片上可以读到表征电流的电压值
        controller.adc1_raw = ADC1->DR;
        controller.adc3_raw = ADC3->DR;							//电机电源电压
			
				///磁编码器（电角度、机械角度）位置、速度检测 ,给后面取值用///
        controller.theta_elec = Encoder_spi.GetElecPosition();   
        controller.theta_mech = (1.0f/GR)*Encoder_spi.GetMechPosition();
        controller.dtheta_mech = (1.0f/GR)*Encoder_spi.GetMechVelocity();  
        controller.dtheta_elec = Encoder_spi.GetElecVelocity();
			
			  ///计算电机电压，过滤直流链路电压测量值///
        controller.v_bus = 0.95f*controller.v_bus + 0.05f*((float)controller.adc3_raw)*V_SCALE; 
        
        /// 检查状态机状态，并运行适当的函数 ///
        switch(state)
        {
            case REST_MODE:                //仅仅进入菜单串口打印参数设置选项及更改，在串口中断接收中更改
                if(state_change)   //在复位模式下，更改电机模式需要进去菜单上重新设置
                {
                    enter_menu_state();
                }
                break;
            
            case CALIBRATION_MODE:        //运行编码器校准程序
                if(state_change)		//在编码器校正模式下，如果更改电机状态，需要重新校准
                {
                    calibrate();
                }
                break;
             
            case MOTOR_MODE:              //运行力矩控制程序
                if(state_change)	//在力矩控制正常运行模式下，如果更爱电机状态，复位FOC的各各参数
                {
                    enter_torque_mode();
                    count = 0;
                }
								//
                else
                {
                    /*
                    if(controller.v_bus>28.0f)   //如果总线的电压过高，则转动栅极驱动，以防止在再生过程中总线被切断发生FET场效应管爆炸，算是一种保护
									     {        
                        gpio.
                        ->write(0);
                        controller.ovp_flag = 1;
                        state = REST_MODE;
                        state_change = 1;
                        printf("OVP Triggered!\n\r");
                        }
                     */  
                    
                    if((controller.timeout > CAN_TIMEOUT) && (CAN_TIMEOUT > 0))//控制器回路超时，并CAN通讯超时保护，相当于电机不输出，算是一种保护
                    {
                        controller.i_d_ref = 0;
                        controller.i_q_ref = 0;
                        controller.kp = 0;
                        controller.kd = 0;
                        controller.t_ff = 0;
                    } 

                    torque_control(&controller);																								//进行力矩控制（重点看,foc的文件夹）
                    commutate(&controller, &observer, &gpio, controller.theta_elec);           // 运行电流闭环

                    controller.timeout++;
                    count++; 
                }     
                break;
                
            case SETUP_MODE:										/*进入模式设置指令函数，仅仅只有串口打印功能*/
                if(state_change)
                {
                    enter_setup_state();
                }
                break;
                
            case ENCODER_MODE:								//编码调试模式，仅仅输出编码器数据
                print_encoder();
                break;
            
            default:
                break;
        }                 
    }
    TIM1->SR = 0x00;                       // 复位状态寄存器
}
//


char cmd_val[8] = {0};
char cmd_id = 0;
char char_count = 0;

/*
*使用来自串口助手终端或者配置GUI来管理状态机
*当串行接收数据完成的时候调用
*类型是串口中断
*/
void serial_interrupt(void)
{
    while(pc.readable())				//等待串口接收到数据
    {
        char c = pc.getc();			//转存串口接收到的字符，充当标志位
        if(c == 27)	                  //电机模式复位，并把通讯的8位数据清零						
        {
            state = REST_MODE;			 //设置为电机复位状态，特殊指令强制电机复位
            state_change = 1;
            char_count = 0;
            cmd_id = 0;
            gpio.led->write(0);; 
            for(int i = 0; i<8; i++) //复位八位数据
            {
                cmd_val[i] = 0;
            }
        }
        
        if(state == REST_MODE)      	//若电机在最初的复位状态下，在这里进行电机的模式设置，电机上电必须进来一次的	
        {
            switch (c)
            {
            case 'c':											//设置为在编码器校正状态
                state = CALIBRATION_MODE;
                state_change = 1;
                break;
            
            case 'm':											//设置为电机运行模式
                state = MOTOR_MODE;
                state_change = 1;
                break;
            
            case 'e':											//设置为编码器输出模式
                state = ENCODER_MODE;
                state_change = 1;
                break;
            
            case 's':											//设置为电机参数设置模式
                state = SETUP_MODE;
                state_change = 1;
                break;
            
            case 'z':									  	//设置为机械零位为当前编码器位置
                Encoder_spi.SetMechOffset(0);
                Encoder_spi.Sample(DT);
                wait_us(20);
                M_OFFSET = Encoder_spi.GetMechPosition();
                if (!prefs.ready())//内存初始化
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
        else if(state == SETUP_MODE)	//在电机参数设置模式
        {
            if(c == 13)
            {
                switch (cmd_id)
                {
                    case 'b':		// 电流环带宽
                        I_BW = fmaxf(fminf(atof(cmd_val), 2000.0f), 100.0f);
                        break;
                    
                    case 'i':		//CAN 总线上电机的 ID
                        CAN_ID = atoi(cmd_val);
                        break;
                    
                    case 'm':		//CAN 总线主设备的 ID
                        CAN_MASTER = atoi(cmd_val);
                        break;
                    
                    case 'l':		// 输出力矩限制 (current limit = torque_limit/(kt*gear ratio))
                        I_MAX = fmaxf(fminf(atof(cmd_val), 40.0f), 0.0f);
                        break;
                    
                    case 'f':		// 最大弱磁电流
                        I_FW_MAX = fmaxf(fminf(atof(cmd_val), 33.0f), 0.0f);
                        break;
                    
                    case 't':		// CAN 总线超时时间
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
                for(int i = 0; i<8; i++)	//复位八位数据
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
        else if (state == ENCODER_MODE)	//在编码器输出模式上
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
        else if (state == MOTOR_MODE)		//在电机输出模式下
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



//”CAN接收完成“中断服务函数
void onMsgReceived()
{
    printf("%df\n", rxMsg.id);//反馈给主设备响应该指令的是那个电机
    can.read(rxMsg);          //读数据并存储到缓存器
    
    if(rxMsg.id == CAN_ID)    //收到的ID是本电机对应的ID,本电机的ID已经设置存入flash里面了
    {
        controller.timeout = 0;	//复位CAN通讯超时时间，喂狗
			
			//下面是驱动特殊指令
			//进入电机模式[OxFF, OxFF, OxFF, OxFF, OxFF, OxFF, OxFF, OxFC]
        if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) & (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFC)))
        {
            state = MOTOR_MODE;//手动让电机停止，并设置dq轴的电压为0，可以手动掰电机
            state_change = 1;
        }
				
			//退出电机模式[OxFF, OxFF, OxFF, OxFF, OxFF, OxFF, OxFF, OxFD]
        else if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFD)))
        {
            state = REST_MODE;//模式没选择状态
            state_change = 1;
            gpio.led->write(0);//把驱动芯片失能，不输出
        }
				
			//设置机械位置传感器位0，[OxFF, OxFF, OxFF, OxFF, OxFF, OxFF, OxFF, OxFE]
        else if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFE)))
        {
            Encoder_spi.ZeroPosition();//手动校正编码器的位置值
        }
			
				//执行电机正常的驱动模式（都不是上面三种特殊指令）
        else if(state == MOTOR_MODE)
        {
          unpack_cmd(rxMsg, &controller);//CAN接口的指令解析,把CAN寄存器里面的数据搬到电机控制参数的内存上
        }
        pack_reply(&txMsg, controller.theta_mech, controller.dtheta_mech, controller.i_q_filt*KT_OUT);//把收到的数据返回给主设备，表明自己收到了，数据填充而已
        can.write(txMsg);	//CAN发送
    }
}
//



//////////////////////////////////////////////////////////////////////
//////////////////////////////////主函数//////////////////////////////
//////////////////////////////////////////////////////////////////////
uint16_t ram_encoder_data = 0;		//编码器数据的内存变量
int main()
{
/*初始化片内外设的*/
    controller.v_bus = V_BUS;			//设置电压24V
    controller.mode = 0;					//还没设置电机模式
    Init_All_HW(&gpio);           //初始化PWM, ADC, LED硬件片内外设
    wait(0.1);  
	
/*初始化驱动芯片的*/
    gpio.enable->write(1);        //使能Drv8323x驱动芯片，DRV832x使能引脚拉高，mbed底层的写法，这个引脚在上面初始化过的
    wait_us(100);
    drv.calibrate();               //驱动IC偏移校准的电流检测放大器ABC的短输入
    wait_us(100);
    drv.write_DCR(0x0, 0x0, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);                 //配置驱动器控制的寄存器,具体看驱动IC的数据手册
    wait_us(100);
    drv.write_CSACR(0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x0, 0x0, 0x0, SEN_LVL_1_0);       //配置电流检测放大器寄存器
    wait_us(100);
    drv.write_OCPCR(TRETRY_4MS, DEADTIME_200NS, OCP_RETRY, OCP_DEG_8US, VDS_LVL_1_88);  //配置过电流保护寄存器
    zero_current(&controller.adc1_offset, &controller.adc2_offset);                     //测量电流传感器零点漂移
    drv.disable_gd();																																		// IC状态失能
    wait(0.1);
/*初始化FOC算法*/
    reset_foc(&controller);             // 复位电流控制器
    reset_observer(&observer);          // 复位观测器
/*初始化TIM定时器中断*/
    TIM1->CR1 ^= TIM_CR1_UDIS;          // 关闭中断
    //TIM1->CR1 |= TIM_CR1_UDIS;        // 使能中断 
    wait(0.1);  
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2);    // 建立终端向量连接
/*初始化CAN接收中断*/
    NVIC_SetPriority(CAN1_RX0_IRQn, 3);
    can.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0);                                 
    txMsg.id = CAN_MASTER;
    txMsg.len = 6;
    rxMsg.len = 8;
    can.attach(&onMsgReceived);         //建立连接“CAN接收处理”中断处理函数  
/*初始化电机控制参数*/		
    //如果用户没有设置首选项，请设置默认状态
		//储存在flash中的值，可以由用户操作修改
    prefs.load();                       //先读 flash里面的数据，用于初始化的时候设置电机参数
		//在下面可以改电机参数（对内）
    if(isnan(E_OFFSET))                         {E_OFFSET = 0.0f;}// 设置编码器电角度偏移 
    if(isnan(M_OFFSET))                         {M_OFFSET = 0.0f;}// 设置编码器机械角度偏移
    if(isnan(I_BW) || I_BW==-1)                 {I_BW = 1000;}		// 设置电流环带宽
    if(isnan(I_MAX) || I_MAX ==-1)              {I_MAX = 15;}			// 设置输出力矩限制 (current limit = torque_limit/(kt*gear ratio))，改输出力矩在这里改
    if(isnan(I_FW_MAX) || I_FW_MAX ==-1)        {I_FW_MAX=0;}			// 设置最大弱磁电流
    if(isnan(CAN_ID) || CAN_ID==-1)             {CAN_ID = 1;}			// 设置CAN 总线上电机的 ID
    if(isnan(CAN_MASTER) || CAN_MASTER==-1)     {CAN_MASTER = 0;}	// 设置CAN 总线主设备的 ID
    if(isnan(CAN_TIMEOUT) || CAN_TIMEOUT==-1)   {CAN_TIMEOUT = 0;}// 设置CAN 总线超时时间
		init_controller_params(&controller);													//初始化电机FOC五个控制参数
/*初始化编码器位置（用磁编码器提供的API就行）*/  
    Encoder_spi.SetElecOffset(E_OFFSET);        // 设置编码器电角度偏移
    Encoder_spi.SetMechOffset(M_OFFSET);        // 设置编码器机械角度偏移
    int lut[128] = {0};													// 编码器偏移LUT-128个元素长度表
    memcpy(&lut, &ENCODER_LUT, sizeof(lut));  	// 复制搬数据
    Encoder_spi.WriteLUT(lut);                  // 设置位置传感器非线性查找表
/*初始化串口中断，并打印电机相关信息*/
    pc.baud(921600);                            //位置串口波特率
    wait(0.01);
    pc.printf("\n\r\n\r HobbyKing Cheetah\n\r\n\r");
    wait(0.01);
    printf("\n\r Debug Info:\n\r");
    printf(" Firmware Version: %s\n\r", VERSION_NUM);
    printf(" ADC1 Offset: %d    ADC2 Offset: %d\n\r", controller.adc1_offset, controller.adc2_offset);	//显示ADC表征电压值
    printf(" Position Sensor Electrical Offset:   %.4f\n\r", E_OFFSET);																	//显示编码器测到的电角度
    printf(" Output Zero Position:  %.4f\n\r", M_OFFSET);																								//显示编码器测到的机械角度
    printf(" CAN ID:  %d\n\r", CAN_ID);																																	//显示CAN通讯ID
    pc.attach(&serial_interrupt);               																												//连接串口中断
//开始选择电机模式   
		state_change = 1;
    while(1)
    {

        wait(0.1);                                  		 // 等待 100ms
        ram_encoder_data = Encoder_spi.GetRawPosition(); // 获取编码器实时位置转角
//      drv.print_faults();        											 // 调试用的， 实时显示驱动器错误状态，MIT驱动IC的faults应该是接了单片机的
//      printf("%.4f\n\r", controller.v_bus);						 // 调试用的，实时显示电机电压跳变 ，电压跳变的ADC是通过定时器做的     
       /*
        if(state == MOTOR_MODE)//调试用的
        {
            //printf("%.3f  %.3f  %.3f\n\r", (float)observer.temperature, (float)observer.temperature2, observer.resistance);//实时显示电机文帝
            //printf("%.3f  %.3f  %.3f %.3f %.3f\n\r", controller.v_d, controller.v_q, controller.i_d_filt, controller.i_q_filt, controller.dtheta_elec);
            //printf("%.3f\n\r", controller.dtheta_mech);
            wait(0.002);
        }
        */
    }
}
