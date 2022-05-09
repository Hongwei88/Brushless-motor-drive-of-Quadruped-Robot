/*
* È·¶¨Î»ÖÃ´«¸ĞÆ÷Æ«ÒÆ¡¢ÏàÎ»ÅÅĞòºÍÎ»ÖÃ´«¸ĞÆ÷ÏßĞÔ»¯µÄĞ£×¼³ÌĞò
*/

#include "calibration.h"
#include "foc.h"
#include "PreferenceWriter.h"
#include "user_config.h"
#include "motor_config.h"
#include "current_controller_config.h"

//¼ì²é²¢ÉèÖÃÏàÎ»Ë³Ğò
///¼ì²éÏàĞòÃüÁî£¬ÒÔÈ·±£ÕıQµçÁ÷ÔÚÎ»ÖÃ´«¸ĞÆ÷Õı·½Ïò²úÉúµçÁ÷///
void order_phases(PositionSensor *ps, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs)
	{   
   //³õÊ¼»¯µç»úFOC¿ØÖÆµÄ±äÁ¿  
    printf("\n\r Checking phase ordering\n\r");
    float theta_ref = 0;					//¶¨ÒåÒ»¸ö²Î¿¼µÄtheta½Ç¶È
    float theta_actual = 0;				//¶¨ÒåÒ»¸öÊµ¼ÊµÄtheta½Ç¶È
    float v_d = V_CAL;  					//°ÑËùÓĞµÄµçÑ¹·ÅÔÚDÖáÉÏ
    float v_q = 0.0f;							//°ÑqÖáµÄµçÑ¹ÖÃ0
    float v_u, v_v, v_w = 0;			//°ÑÈıÏàÕñ·ùÖÃ0£¨±êÁ¿£©
    float dtc_u, dtc_v, dtc_w = .5f;//°ÑÈıÏàµÄÊ¸Á¿µçÑ¹ÖÃ0
    int sample_counter = 0;				//°Ñ²ÉÓÃµÄ¼ÆÊıÖµÖÃ0
   ///////////////////////////////////////////////////////////////////////////////////////////////////	
		
		
		

   
    ///½«µçÑ¹½ÇÉèÖÃÎªÁã£¬¼´ÉèÖÃ×ª¶¯Ğ£ÕıÇ°µç»úµÄÎ»ÖÃ£¬Ò»´Î¾ÍºÃ
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);          //µçÑ¹·´dq0±ä»»£¬Í¨¹ıÉÏÃæ¸ø¶¨µÄdq×ø±êÏµÏÂµÄµçÑ¹ºÍ½Ç¶È£¬¼ÆËã³öÈıÏàµÄÕñ·ù
    svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);     //¿Õ¼äÊ¸Á¿µ÷ÖÆ£¬Í¨¹ıµçÔ´µçÑ¹ºÍÈıÏàÕñ·ù£¬¼ÆËã³öÈıÏà¶ÔÓ¦µÄµçÑ¹Ê¸Á¿ 
		for(int i = 0; i<20000; i++)												 //ÉèÖÃ³õÊ¼Õ¼¿Õ±È£¬¿ªÊ¼¿ØÖÆµç»úÁË£¬ÕâÒ»Ë²¼äµç»úÊÇËøËÀµÄ
			{
			TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);                                
			TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
			TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
			wait_us(100);
			}
		//////
    ps->Sample(DT); 																		//Í¨¹ı´Å±àÂëÆ÷¶ÔÏÖÔÚµÄ	×ª×ÓÎ»ÖÃ½øĞĞ²ÉÑù
    wait_us(1000);
		float theta_start;	 //´Å±àÂëÆ÷»ñÈ¡¾ø¶ÔµÄ»úĞµ½Ç¶È£¬×î¿ªÊ¼µÄÊ±ºò£¬¶¨ÒåµÄÊ±ºò¾ÉÄ¬ÈÏÎª0ÁË
    //float theta_start = ps->GetMechPositionFixed();                                  //»ñÈ¡³õÊ¼»¯×ª×ÓÎ»ÖÃ
		//////	
    controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);    //¸ù¾İADCµÄ¶ÁÊı¼ÆËãÏàµçÁ÷
    controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);
    controller->i_a = -controller->i_b - controller->i_c;					//¸ù¾İ»ù¶û»ô·òµçÁ÷¶¨ÂÉ£¬ÖªµÀÁ½ÏàµçÁ÷¿ÉÒÔÇóµÚÈıÏàµçÁ÷
	 
			//µçÁ÷dq0±ä»»£¬µÃµ½dqÖáÉÏµÄÁ½¸öµçÁ÷·ÖÁ¿
    dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);   
    float current = sqrt(pow(controller->i_d, 2) + pow(controller->i_q, 2));	//¼ÆËã³ödqÖáÉÏµÄÁ½¸öµçÁ÷·ÖÁ¿ºÏ³ÉµÄ×ÜµçÁ÷
    printf("\n\rCurrent\n\r");
    printf("%f    %f   %f\n\r\n\r", controller->i_d, controller->i_q, current);
		///////////////////////////////////////////////////////////////////////////////////////////////////





			
    /// ¿ØÖÆĞı×ªµçÑ¹½Ç¶È£¬³ÖĞøÈÃÈıÏàµçÑ¹Ê¸Á¿ÑØ×ÅÕı·½Ïò×ª¶¯£¬±ß×ª¶¯£¬±ß¼ÆËãºÍÊä³öÊµ¼ÊµÄ»úĞµ½Ç¶È
    while(theta_ref < 4*PI)//Ğı×ªËÄÈ¦£¬Êµ¼ÊÉÏµç½Ç¶ÈĞı×ª6È¦£¬»úĞµ½Ç²ÅĞı×ªÒ»È¦µÄ
			{                                                    //Ğı×ªÁ½¸öµçÖÜÆÚ
        abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                             //µçÑ¹·´dq0±ä»»
        svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);                        //¿Õ¼äÊ¸Á¿µ÷ÖÆ
        wait_us(100);
        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);                                        //ÉèÖÃÕ¼¿Õ±È
        TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
        TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
       ps->Sample(DT);                                                            //²É¼¯Î»ÖÃ´«¸ĞÆ÷
       theta_actual = ps->GetMechPositionFixed();
       if(theta_ref==0){theta_start = theta_actual;}
       if(sample_counter > 200)
				{
           sample_counter = 0 ;
        printf("%.4f   %.4f\n\r", theta_ref/(NPP), theta_actual);
        }
        sample_counter++;
       theta_ref += 0.001f;
       }
		///////////////////////////////////////////////////////////////////////////////////////////////////	
			 
			 
			 
			 
    float theta_end = ps->GetMechPositionFixed(); //´Å±àÂëÆ÷»ñÈ¡¾ø¶ÔµÄ»úĞµ½Ç¶È£¬×îºóµÄÊ±ºò	 
    int direction = (theta_end - theta_start)>0;  //ÅĞ¶Ïµç»ú×ª¶¯µÄ·½Ïò£¬ÕâÀïÍ¨¹ıÅĞ¶ÏÔËËã·û£¬Êä³öµÄÊÇ²¼¶ûÖµ
    printf("Theta Start:   %f    Theta End:  %f\n\r", theta_start, theta_end);
    printf("Direction:  %d\n\r", direction);
    if(direction){printf("Phasing correct\n\r");}
    else if(!direction){printf("Phasing incorrect.  Swapping phases V and W\n\r");}
    PHASE_ORDER = direction;      //¸ü¸ÄÏàÎ»·½ÏòË³Ğò
    }
    
//Ğ£Õı³ÌĞò
///²âÁ¿Î»ÖÃ´«¸ĞÆ÷µÄµç½Ç¶ÈÆ«ÒÆ£¬²¢£¨½«À´£© Ğ£ÕıÓÉÓÚÎ»ÖÃ´«¸ĞÆ÷Æ«ĞÄ¶øÒıÆğµÄ·ÇÏßĞÔ/// 		
void calibrate(PositionSensor *ps, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs)
	{
		///¶¨ÒåĞ£Õı³ÌĞòµÄ±äÁ¿//////////////////////////////////////////////////////////////////////////////////////
    printf("Starting calibration procedure\n\r");
    float * error_f;															  //ÕıĞı×ª¼ÇÂ¼µÄĞı×ª½ÇÆ«ÒÆÁ¿
    float * error_b;																//·´Ğı×ª¼ÇÂ¼µÄĞı×ª½ÇÆ«ÒÆÁ¿
    int * lut;																			//²é±íÓÃµÄ±äÁ¿
    int * raw_f;																		//Ç°½ø·½Ïò×ª×ÓµÄ½Ç¶È
    int * raw_b;																		//ºóÍË·½Ïò×ª×ÓµÄ½Ç¶È
    float * error;																	//Ğı×ªÁ¿×ÜÆ«²î
    float * error_filt; 														//Ğ£ÕıºóĞı×ªÁ¿×ÜÆ«²î	
    const int n = 128*NPP;                          // Ã¿´Î»úĞµĞı×ª½Ç¶È²ÉÑùµÄÎ»ÖÃÊıÁ¿
    const int n2 = 40;                              // ±£´æ²ÉÑùÖ®¼äµÄÔöÁ¿£¨ÓÃÓÚÆ½»¬ÔË¶¯£©
    float delta = 2*PI*NPP/(n*n2);                  // ²ÉÑùÊı¾İµÄ½Ç¶È±ä»¯
    error_f = new float[n]();                       // ¸³ÖµÎó²îÊ¸Á¿ÏòÇ°Ğı×ª
    error_b = new float[n]();                       // ¸³ÖµÎó²îÊ¸Á¿ÏòºóĞı×ª
    const int  n_lut = 128;													//¸³Öµ²é±í±éÀú±äÁ¿	
    lut = new int[n_lut]();                         // ÔÚ¿ªÊ¼Ç°£¬Çå³ıËùÓĞ¾ÍµÃ²éÔÄ±í¸ñ
    error = new float[n]();
    const int window = 128;
    error_filt = new float[n]();
    ps->WriteLUT(lut); 
    raw_f = new int[n]();
    raw_b = new int[n]();
    float theta_ref = 0;														//²Î¿¼µÄtheta½Ç
    float theta_actual = 0;													//ÕæÊµµÄtheta½Ç
    float v_d = V_CAL;                              //°ÑËùÓĞµÄµçÑ¹·ÅÔÚDÖáÉÏ
    float v_q = 0.0f;																//ÇåÁãQÖáµçÑ¹
    float v_u, v_v, v_w = 0;												//ÈıÏàÏßµçÑ¹
    float dtc_u, dtc_v, dtc_w = .5f;								//ÈıÏàÏßµçÁ÷£¨²»Çå³ş£©
    
    ///½«µçÑ¹½ÇÉèÖÃÎªÁã£¬µÈ´ı×ª×ÓÎ»ÖÃÎÈ¶¨£¬½ö½ö×öÒ»´Î/////////////////////////////////////////////////////////////////////////////////////////////////////
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);      //µçÑ¹·´dq0±ä»»
    svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); //¿Õ¼äÊ¸Á¿µ÷ÖÆ
    for(int i = 0; i<40000; i++)										 //ÉèÖÃÕ¼¿Õ±È 
		{
			TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);        //¸ø¶¨VÏàµÄPWM                        
			if(PHASE_ORDER)																 //Ö®Ç°ÒÑ¾­¼ì²éÁËUVË³ĞòµÄ£¬°´×ÅÈ·¶¨µÄË³Ğò¸øµçÑ¹ĞÅºÅ
				{                                   
					TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);		 //¸ø¶¨VÏàµÄPWM
					TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);		 //¸ø¶¨WÏàµÄPWM
				}
			else
				{
					TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_v);		 //¸ø¶¨VÏàµÄPWM
					TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_w);    //¸ø¶¨WÏàµÄPWM
				}
			wait_us(100);
		}
    ps->Sample(DT); 																//´Å±àÂëÆ÷  ¿ªÊ¼²ÉÑù
    controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);     //¸ù¾İADCµÄ¶ÁÊı¼ÆËãÏàµçÁ÷B,Í¨¹ı£¨ÊµÊ±ADC²É¼¯Çı¶¯ICµÄµçÁ÷¼ÆËã-Ğ£ÕıºóµÄµçÁ÷¼ÆÊı£©*µ¥Î»¼ÆËã¶ÔÓ¦µÄµçÁ÷£=Êµ¼ÊµÄµçÁ÷
    controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);		 //¸ù¾İADCµÄ¶ÁÊı¼ÆËãÏàµçÁ÷C
    controller->i_a = -controller->i_b - controller->i_c;																	 //¸ù¾İADCµÄ¶ÁÊı¼ÆËãÏàµçÁ÷A
    dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);   //µçÁ÷dq0±ä»»
    float current = sqrt(pow(controller->i_d, 2) + pow(controller->i_q, 2));							 //¼ÆËã×ÜµçÁ÷
    printf(" Current Angle : Rotor Angle : Raw Encoder \n\r\n\r");
		
		
		
		///ÏòÇ°Ğı×ª//////////////////////////////////////////////////////////////////////////////////////////////////////
    for(int i = 0; i<n; i++)		 												// Ã¿´Î»úĞµĞı×ª½Ç¶È²ÉÑùµÄÎ»ÖÃÊıÁ¿
		{                                                   
       for(int j = 0; j<n2; j++) 												// ÏòÇ°Ğı×ª£¬·¢ÄÇÃ´¶à´ÎPWMÊÇÎªÁË¿ØÖÆÏìÓ¦Æ½»¬Ò»µã
			{   
       theta_ref += delta;			 												// ²Î¿¼µÄµç½Ç¶È°´ÔöÁ¿µİÔö
       abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);			//µçÑ¹·´dq0±ä»»
       svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); //¿Õ¼äÊ¸Á¿µ÷ÖÆ
        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);
        if(PHASE_ORDER){                                //¼ì²éÏàĞòË³Ğò 
            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);     // ÉèÖÃÕ¼¿Õ±È
            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
            }
        else{
            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_v);
            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_w);
            }
            wait_us(100);
            ps->Sample(DT);
        }
       ps->Sample(DT);
       theta_actual = ps->GetMechPositionFixed();
       error_f[i] = theta_ref/NPP - theta_actual;		//¼ÆËã¸ø¶¨½Ç¶ÈºÍĞı×ª½Ç¶ÈµÄÆ«²î£¬È·¶¨Î»ÖÃ´«¸ĞÆ÷µÄÆ«ÒÆ
       raw_f[i] = ps->GetRawPosition();							//±àÂëÆ÷¼ÇÂ¼µÄÎ»ÖÃ
        printf("%.4f   %.4f    %d\n\r", theta_ref/(NPP), theta_actual, raw_f[i]);//´òÓ¡¸ø¶¨µÄ»úĞµ½Ç¶È
    }

	///ÏòºóĞı×ª»ñÈ¡///////////////////////////////////////////////////////////////////////////////////////////////////////////	
    for(int i = 0; i<n; i++)														// Ã¿´Î»úĞµĞı×ª½Ç¶È²ÉÑùµÄÎ»ÖÃÊıÁ¿							
		{                                                   
			 for(int j = 0; j<n2; j++)								  			// ÏòºóĞı×ª
			{
			 theta_ref -= delta;														  // ²Î¿¼µÄµç½Ç¶È°´ÔöÁ¿µİÔö
			 abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);      //µçÑ¹·´dq0±ä»»
			 svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); //¿Õ¼äÊ¸Á¿µ÷ÖÆ
				TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);
				if(PHASE_ORDER)
						{
						TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
						TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
						}
				else{
						TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_v);
						TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_w);
						}
						wait_us(100);
						ps->Sample(DT);
				}
			 ps->Sample(DT);                                  //²ÉÓÃÎ»ÖÃ´«¸ĞÆ÷
			 theta_actual = ps->GetMechPositionFixed();       // »ñÈ¡»úĞµ½Ç¶È
			 error_b[i] = theta_ref/NPP - theta_actual;				//¼ÆËã¸ø¶¨½Ç¶ÈºÍĞı×ª½Ç¶ÈµÄÆ«²î£¬È·¶¨Î»ÖÃ´«¸ĞÆ÷µÄÆ«ÒÆ
			 raw_b[i] = ps->GetRawPosition();
			 printf("%.4f   %.4f    %d\n\r", theta_ref/(NPP), theta_actual, raw_b[i]);
			 //theta_ref -= delta;
		}    
 	////¼ÆËãÆ½¾ùÎ»ÖÃ´«¸ĞÆ÷Æ«ÒÆ²¢ÉèÖÃ´Å±àÂëÆ÷µÄÆ«ÒÆ¼Ä´æÆ÷//////////////////////////////////////////////////////////////////////////////////////////////////////////	
        float offset = 0;                                  
        for(int i = 0; i<n; i++)
					{
					offset += (error_f[i] + error_b[n-1-i])/(2.0f*n);                   // ¼ÆËãÆ½¾ùÎ»ÖÃ´«¸ĞÆ÷Æ«ÒÆ
					}
        offset = fmod(offset*NPP, 2*PI);                                    // ½«»úĞµ½Ç¶È×ª»»ÎªµçÆø½Ç¶È   
        ps->SetElecOffset(offset);                                          // ÉèÖÃÎ»ÖÃ´«¸ĞÆ÷Æ«ÒÆ
        __float_reg[0] = offset;
        E_OFFSET = offset;
        
	/// ÒÔÏßĞÔ»¯Î»ÖÃ´«¸ĞÆ÷Æ«ĞÄÂÊÖ´ĞĞÂË²¨
	/// nÎ»²ÉÑùÆ½¾ùÖµ£¬ÆäÖĞn=Ò»¸öµçÑ­»·ÖĞµÃÑùÆ·ÊıÁ¿
	///¸ÃÂË²¨Æ÷ÔÚµãÆµÂÊÏÂÔöÒæÎª0£¬ÇÒ¾ùÎªÕû±¶Êı£¬ËùÒÔ³İ²ÛĞ§Ó¦Ó¦¸Ã»Ø±ÜÍêÈ«ÂËµô
        float mean = 0;
        for (int i = 0; i<n; i++)   //Æ½¾ùÇ°ºóÁ½¸ö·½Ïò
					{                                              
            error[i] = 0.5f*(error_f[i] + error_b[n-i-1]);
          }
					
				//ÂË²¨	
        for (int i = 0; i<n; i++)
					{
            for(int j = 0; j<window; j++)
						{
                int ind = -window/2 + j + i;                                    // Indexes from -window/2 to + window/2
                if(ind<0)
									{ind += n;}                                                  //ÒÆ¶¯Æ½¾ùÈÆÏß 
                else if(ind > n-1) 
									{ind -= n;}
                error_filt[i] += error[ind]/(float)window;
            }
            if(i<window){
//                cogging_current[i] = current*sinf((error[i] - error_filt[i])*NPP);
                }
            //printf("%.4f   %4f    %.4f   %.4f\n\r", error[i], error_filt[i], error_f[i], error_b[i]);
            mean += error_filt[i]/n;
           }
        int raw_offset = (raw_f[0] + raw_b[n-1])/2;                             //¶ÔÕâ¸ö·½Ïò´íÎó²»Ãô¸Ğ£¬ËùÒÔÁ½·Ö¾Í×ã¹»ÁË
        
       // //Éú³ÉÏßĞÔ»¯²éÕÒ±í
        printf("\n\r Encoder non-linearity compensation table\n\r");
        printf(" Sample Number : Lookup Index : Lookup Value\n\r\n\r");
        for (int i = 0; i<n_lut; i++)
					 {                                          //Éú³É²éÕÒ±í
            int ind = (raw_offset>>7) + i;
            if(ind > (n_lut-1)){ 
                ind -= n_lut;
                }
            lut[ind] = (int) ((error_filt[i*NPP] - mean)*(float)(ps->GetCPR())/(2.0f*PI));
            printf("%d   %d   %d \n\r", i, ind, lut[ind]);
            wait(.001);
            }
            
        ps->WriteLUT(lut);                                                      // ½«²éÕÒ±íĞ´ÈëÎ»ÖÃ´«¸ĞÆ÷¶ÔÏó
        memcpy(&ENCODER_LUT, lut, sizeof(lut));                                 // ½«²éÕÒ±í¸´ÖÆµ½ÉÁ´æÕóÁĞ
        printf("\n\rEncoder Electrical Offset (rad) %f\n\r",  offset);
        //±£´æµ½ÉÁ´æ
        if (!prefs->ready()) prefs->open();
        prefs->flush();                                                         // ½«Æ«ÒÆÁ¿ºÍ²éÕÒ±íĞ´ÈëÉÁ´æ
        prefs->close();
        
        delete[] error_f;       //±ØĞëÊÍ·ÅÉÁ´æ
        delete[] error_b;
        delete[] lut;
        delete[] raw_f;
        delete[] raw_b;

    }
