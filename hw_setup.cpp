/*
*��Ƭ��Ƭ�������ʼ������ʼ��PWM, ADC, LED
*/
#include "mbed.h"
#include "hw_setup.h"
#include "hw_config.h"
#include "structs.h"
#include "FastPWM.h"

/*
*������IC��PWM�ź�����Ӳ����ʱ��TIM1,TIM2�����ģ�PWM��Ƶ�ʺ�MCUʱ��Ƶ��һ��
*
*/
void Init_PWM(GPIOStruct *gpio)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;            // enable the clock to GPIOC
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;             // enable TIM2 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;             // enable TIM1 clock

    GPIOC->MODER |= GPIO_MODER_MODE5_0;             //������5����ΪLED��ͨ�����
    gpio->enable = new DigitalOut(ENABLE_PIN);			//��ʼ��ʹ������IC������
    gpio->pwm_u = new FastPWM(PIN_U);								//��ʼ����������IC��PWM����������
    gpio->pwm_v = new FastPWM(PIN_V);
    gpio->pwm_w = new FastPWM(PIN_W);
    
    //ISR Setup     
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);             //Enable TIM1 IRQ

    TIM1->DIER |= TIM_DIER_UIE;                     // enable update interrupt
    TIM1->CR1 = TIM_CR1_CMS_1;                      // CMS = 10, �����ڼ���ʱ���ж�
    TIM1->CR1 |= TIM_CR1_UDIS;
    TIM1->CR1 |= TIM_CR1_ARPE;                      //�Զ�����翪��  
    TIM1->RCR |= 0x001;                             //ÿ��tim1�ĵ���/�ݼ���������һ���¼�
    TIM1->EGR |= TIM_EGR_UG;
 
    //PWM Setup
    TIM1->PSC = 0x0;                                // no prescaler, timer counts up in sync with the peripheral clock
    TIM1->ARR = PWM_ARR;                            // �����Զ�װ��, 40 khz
    TIM1->CCER |= ~(TIM_CCER_CC1NP);                // �Ͳ��ʱ���ж�
    TIM1->CR1 |= TIM_CR1_CEN;                       // ʹ�� TIM1
}

/*
*��ʼ���ɼ�����ic������ADCͨ��PC0\PC1
*����PA0��Ӧ�Ǽ��ʲô�ͺŻ���֪����Ҫô���¶�ADC,Ҫô�ǵ�Դ��ѹADC
*/
void Init_ADC(void)
{
    // ADC Setup
     RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;            // clock for ADC3
     RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;            // clock for ADC2
     RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;            // clock for ADC1
     
     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;           // Enable clock for GPIOC
     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;           // Enable clock for GPIOA
    
     ADC->CCR = 0x00000016;                         // Regular simultaneous mode only
     ADC1->CR2 |= ADC_CR2_ADON;                     // ADC1 ON
     ADC1->SQR3 = 0x000000A;                        // use PC_0 as input- ADC1_IN0
     ADC2->CR2 |= ADC_CR2_ADON;                     // ADC2 ON
     ADC2->SQR3 = 0x0000000B;                       // use PC_1 as input - ADC2_IN11
     ADC3->CR2 |= ADC_CR2_ADON;                     // ADC3 ON
     ADC3->SQR3 = 0x00000000;                       // use PA_0, - ADC3_IN0
     GPIOC->MODER |= 0x0000000f;                    // Alternate function, PC_0, PC_1 are analog inputs 
     GPIOA->MODER |= 0x3;                           // PA_0 as analog input
     
     ADC1->SMPR1 |= 0x1;                                        // 15 cycles on CH_10, 0b 001
     ADC2->SMPR1 |= 0x8;                                        // 15 cycles on CH_11, 0b 0001 000
     ADC3->SMPR2 |= 0x1;                                        // 15 cycles on CH_0, 0b 001;
}


void Init_All_HW(GPIOStruct *gpio)
{
    Init_PWM(gpio);
    Init_ADC();												//��ʼ������IC�ɼ�������ADC,����һ����Դ��ѹ�����¶ȵ�ADC
    gpio->led = new DigitalOut(LED);	//��ʼ��LED����
}
