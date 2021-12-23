//////////////////////////////////////////////////////////////////////
// PR2: 반도체 공정 제어기
// 제출자: 2017130036_이찬해
// 주요내용

///////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"
//#include "FRAM.h"

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

void _GPIO_Init(void);
void _EXTI_Init(void);
void DisplayOn(void);
void _ADC_Init(void);
uint16_t KEY_Scan(void);

void TIMER3_Init(void);
void TIMER2_PWM_Init(void);	// Timer4 PWM mode 

void Process_Time_Setting(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

uint16_t ADC_Value, Voltage;
uint8_t str[20];

//flag 선언부
// 상황 flag
//1: 
int flag=0;

//0: 세부입력 진입, 1: 온도값 입력, 2: 압력값 입력, 3: O2값 입력, 4: plasma 세기
int processing_op =0;
//프로세스가 끝나면 알려주는 플레그
//0: 대기상태, 1: 온도 프로세스, 2: 압력 프로세스, 3: o2 프로세스, 4:플라즈마 프로세스
int process_done=0;


//int mode_flag=0;
//int input_flag=0;
 
 //타이머에서 숫자표시를 위한 변수
 int data_in_timer;
 
 //fram 저장 조건에 의해 히팅 시간의 10을 곱한 값을 저장한다.
//온도값           //압력값             // 산소값          //plasma
int te;                int pr;                  int o2;             int dr;

int process_flag=0;
//타이머에서 시간표현할때 쓰는 변수
int cnt=0;
void ADC_IRQHandler(void)
{
	ADC1->SR &= ~(1<<1);		// EOC flag clear

	ADC_Value = ADC1->DR;		// Reading ADC result

        //ADC에 따른 온도값, 압력값, O2값, PLASMA 값
        te=5*((int)(ADC_Value/585.1)+2);
        pr=5*((int)(ADC_Value/585.1)+1);
        o2=5*((int)(ADC_Value/585.1));
        dr=(int)(ADC_Value/410);

        //세부입력모드에서
        if(processing_op==1) {           //온도값
                LCD_SetTextColor(RGB_RED);	//글자색
                LCD_DisplayChar(3,4,te/10 + 0x30);
                LCD_DisplayText(3,5,".");
                LCD_DisplayChar(3,6,te%10 + 0x30);
        }
        else if(processing_op==2){       //압력값
                LCD_SetTextColor(RGB_RED);	//글자색
                LCD_DisplayChar(4,4,pr/10 + 0x30);
                LCD_DisplayText(4,5,".");
                LCD_DisplayChar(4,6,pr%10 + 0x30);
        }
        else if(processing_op==3)       //o2값
        {
                LCD_SetTextColor(RGB_RED);	//글자색
                LCD_DisplayChar(5,4,o2/10 + 0x30);
                LCD_DisplayText(5,5,".");
                LCD_DisplayChar(5,6,o2%10 + 0x30);
        }
        else if(processing_op==4)       //plasma값
        {
                LCD_SetTextColor(RGB_RED);	//글자색
                LCD_DisplayChar(6,4,dr+ 0x30);
        }

        //0~4095 확인용 
       	//sprintf(str,"%4d",ADC_Value);	// ADC result : 12bits (0~4095)
       //LCD_DisplayText(7,6,str);
	//Voltage = ADC_Value * (3.3 * 100) / 4095;   // 3.3 : 4095 =  Volatge : ADC_Value 
                                                    // 100:  소수점아래 두자리까지 표시하기 위한 값  
   
  	ADC1->CR2 |=  (1<<30); 	// swstart adc
}

int main(void)
{
  LCD_Init();   // LCD 구동 함수
  DelayMS(100);   // LCD구동 딜레이
  _GPIO_Init(); //GPIO 초기화
	  _ADC_Init();
  	Fram_Init();                    // FRAM 초기화 H/W 초기화
	Fram_Status_Config();   // FRAM 초기화 S/W 초기화
        DisplayOn();   //LCD 초기화면구동 함수
              TIMER3_Init();		// 범용타이머(TIM3) 초기화 : up counting mode
      TIMER2_PWM_Init();		// TIM4 Init (PWM mode)

  _EXTI_Init();	// EXTI 초기화
  GPIOG->ODR &= 0x00;   // LED0~7 Off 
  GPIOG->ODR |= 0x01;   // LED1 on

  while(1)
  {
      if(process_flag==1)                       //실행 영역
      {
        flag=3;                 //실행 후 대기모드
                  GPIOG->ODR |= 0x04;   // LED2 on
                  LCD_DisplayText(2,13,"W");
                  GPIOG->ODR |= 0x08;   // LED3 on
                  DelayMS(500);
                               
                  //공정 시작
                  process_done=1;
                  GPIOG->ODR |= 0x0;
                  
                  //공정 LCD 시간 세팅
                  Process_Time_Setting();
                 
         //온도 공정
                  GPIOG->ODR |= 0x10;                    // LED4 on
                  LCD_DisplayText(3,0,"*");
                  while(process_done!=2){
                        LCD_DisplayChar(3,9,cnt/10+0x30);
                        LCD_DisplayText(3,10,".");
                        LCD_DisplayChar(3,11,cnt%10+0x30);
                  }
                  
                  LCD_DisplayText(3,0," ");
                  GPIOG->ODR &= ~0x10;                 //LED4 OFF
                   
         //압력 프로세스
                  LCD_DisplayText(4,0,"*"); 
                  GPIOG->ODR |= 0x20;                    // LED5 on
                  while(process_done!=3){
                        LCD_DisplayChar(4,9,cnt/10+0x30);
                        LCD_DisplayText(4,10,"."); 
                        LCD_DisplayChar(4,11,cnt%10+0x30);
                  }
                  LCD_DisplayText(4,0," ");
                  GPIOG->ODR &= ~0x20;                    // LED5 off
                  
          //O2 프로세스 & 플라즈마 프로세스
                  LCD_DisplayText(5,0,"*");
                  LCD_DisplayText(6,0,"*");
                  GPIOG->ODR |= 0x40;                   // LED6 oN 
                  GPIOG->ODR |= 0x80;                   // LED7 oN 
                  
                  //타이머2 CCR3 을 Fram_Read(1206)로 지정
                  /*
                  switch
                  */
                  
                  TIM2->CCR3 = 0;
                  //TIM2->CR1|= (1<<0);	// CEN: Counter TIM2 enable
                  while(process_done!=0){
                        LCD_DisplayChar(5,9,cnt/10+0x30);
                        LCD_DisplayText(5,10,".");
                        LCD_DisplayChar(5,11,cnt%10+0x30);
                        
                        //TIM2->CR1&= ~(1<<0);	// CEN: Counter TIM2 disnable
                  }
                  TIM2->CCR3  = 5000;		// 플라즈마 공정 중지
                  LCD_DisplayText(5,0," ");
                  LCD_DisplayText(6,0," ");
                  GPIOG->ODR &= ~0x40;                   // LED6 oFF
                  GPIOG->ODR &= ~0x80;                   // LED7 oFF`
                  
                  DelayMS(500);
                  LCD_DisplayText(2,13,"E");
                  GPIOG->ODR &= ~0x08;   // LED3 off
                  process_flag=0;
      }
  }
}
// Fram_Write(1203,te)
//Fram_Read(50)
void EXTI15_10_IRQHandler(void)		
{
  //EXTI->IMR  |= (15<<12);   마스크 활용해서 누르지 말아야 할 버튼 막기
        //sw4 세부입력모드 저장
	if(EXTI->PR & (1<<12))                   // EXTI12Interrupt Pending(발생) 여부?
	{
		EXTI->PR |= (1<<12); 	
                if(flag==2){                       //세부 입력 모드
                      if(processing_op==1) {           //온도값
                        	Fram_Write(1203,te); 
                    }
                    else if(processing_op==2){       //압력값
                               Fram_Write(1204,pr); 
                    }
                    else if(processing_op==3) {      //o2값
                                Fram_Write(1205,o2); 
                    }
                    else if(processing_op==4)  {     //plasma값
                                 Fram_Write(1206, dr); 
                    }
                }

	}
        //sw5 입력<-->실행 모드 전환
        else if(EXTI->PR & (1<<13))                   // EXTI13 Interrupt Pending(발생) 여부?
	{
		EXTI->PR |= (1<<13); 
                if(flag==0){                            //입력 대기--> 실행 대기
                  flag=1; 
                  GPIOG->ODR &= ~0x01;   // LED0 off
                  GPIOG->ODR |= 0x02;   // LED1 on
                  LCD_SetTextColor(RGB_RED);
                  LCD_DisplayText(2,5,"R");
                }
                else if(flag==1){                       //실행 대기 --> 입력 대기
                  flag=0;
                  GPIOG->ODR |= 0x01;   // LED0 on
                  GPIOG->ODR &= ~0x02;   // LED1 off
                  LCD_SetTextColor(RGB_RED);
                  LCD_DisplayText(2,5,"I");
                }
 
	}
        //sw6 모드 선택
        else if(EXTI->PR & (1<<14))                   // EXTI14 Interrupt Pending(발생) 여부?
	{
		EXTI->PR |= (1<<14); 	
                if(flag==0){                            //입력 대기 --> 세부입력모드
                  flag=2;
                  GPIOG->ODR |= 0x04;   // LED2 on
                }
                else if(flag==1){                       //실행 대기 --> 실행
                  process_flag=1;
                }
                else if(flag==2){                       //세부 입력 모드 --> 입력대기
                  flag=0;
                  GPIOG->ODR &= ~0x04;   // LED2 off
                  //세부입력모드에서 나오면 별하고 LED 끄는 것 같은데 우선 대기
                  GPIOG->ODR &= ~0xF0;   // LED4~7 off
                  LCD_DisplayText(3,0," ");
                  LCD_DisplayText(4,0," ");
                  LCD_DisplayText(5,0," ");
                  LCD_DisplayText(6,0," ");
                  processing_op =0;
                }
                else if(flag==3){                       //실행 (실행종료 후) --> 실행대기
                  flag=1;
                  GPIOG->ODR &= ~0x04;   // LED2 off
                }
	}
        //sw7 세부입력모드에서 항목 선택
        else if(EXTI->PR & (1<<15))                   // EXTI15 Interrupt Pending(발생) 여부?
	{
		EXTI->PR |= (1<<15); 	
                 if(flag==2){                       //세부 입력 모드
                    processing_op++;
                    if(processing_op>4){
                      processing_op=1;
                    }
                    switch(processing_op)
                      {
                      case 1:
                        GPIOG->ODR &= ~0x80;
                        GPIOG->ODR |= 0x10;
                        LCD_DisplayText(6,0," ");
                        LCD_DisplayText(3,0,"*");
                        break;
                      case 2:
                        GPIOG->ODR &= ~0x10;
                        GPIOG->ODR |= 0x20;
                        LCD_DisplayText(3,0," ");
                        LCD_DisplayText(4,0,"*");
                        break;
                      case 3:
                        GPIOG->ODR &= ~0x20;
                        GPIOG->ODR |= 0x40;
                        LCD_DisplayText(4,0," ");
                        LCD_DisplayText(5,0,"*");                    
                        break;
                      case 4:
                        GPIOG->ODR &= ~0x40;
                        GPIOG->ODR |= 0x80;
                        LCD_DisplayText(5,0," ");
                        LCD_DisplayText(6,0,"*");                    
                        break;
                      }
                }
	}
}

void TIM3_IRQHandler(void)  	// 0.1s Interrupt
{

        TIM3->SR &= ~(1<<0);	// Interrupt flag Clear
        if(process_done==1){    //온도 프로세스 진행
                cnt++;
                if(cnt>Fram_Read(1203)){
                process_done=2;
                  cnt=0;
                }
         }
        else if(process_done==2){    //압력 프로세스 진행
                 cnt++;
                if(cnt>Fram_Read(1204)){
                  process_done=3;
                  cnt=0;
                }
        }
        else if(process_done==3){    //o2 프로세스 진행
                cnt++;
                if(cnt>Fram_Read(1205)){
                process_done=0;
                  cnt=0;
                }
        }
        else if(process_done==4){    //플라즈마 프로세스 진행
          
        }
}

void TIMER3_Init(void)
{
	RCC->APB1ENR |= 0x02;	// RCC_APB1ENR TIMER3 Enable

	// Setting CR1 : 0x0000 
	TIM3->CR1 &= ~(1<<4);  // DIR=0(Up counter)(reset state)
	TIM3->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
                            //  Counter Overflow/Underflow, 
                            //  Setting the UG bit Set,
                            //  Update Generation through the slave mode controller 
                            // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
	TIM3->CR1 &= ~(1<<2);	// URS=0(Update Request Source  Selection):  By one of following events
                            //	Counter Overflow/Underflow, 
                            // Setting the UG bit Set,
                            //	Update Generation through the slave mode controller 
                            // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
	TIM3->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM3->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM3->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM3->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
                            // Center-aligned mode: The counter counts UP and DOWN alternatively


    // Deciding the Period
	TIM3->PSC = 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz (0.1ms)  (1~65536)
	TIM3->ARR = 1000-1;	// Auto reload  0.1ms * 10 = 0.1s

   	// Clear the Counter
	TIM3->EGR |= (1<<0);	// UG(Update generation)=1 
                        // Re-initialize the counter(CNT=0) & generates an update of registers   

	// Setting an UI(UEV) Interrupt 
	NVIC->ISER[0] |= (1<<29); // Enable Timer3 global Interrupt
 	TIM3->DIER |= (1<<0);	// Enable the Tim3 Update interrupt

	TIM3->CR1 |= (1<<0);	// Enable the Tim3 Counter (clock enable)   
}


void TIMER2_PWM_Init(void)
{   
// TIM2 CH3 : PB10 
  // Clock Enable : GPIOB & TIMER4
	RCC->AHB1ENR	|= (1<<1);	// GPIOB CLOCK Enable
	RCC->APB1ENR 	|= (1<<0);	// TIMER2 CLOCK Enable 
    						
// PB10을 출력설정하고 Alternate function(TIM4_CH3)으로 사용 선언 : PWM 출력
	GPIOB->MODER 	|= (2<<20);	// 0x00020000 PB10Output Alternate function mode					
	GPIOB->OSPEEDR 	|= (3<<20);	// 0x00030000 PB10 Output speed (100MHz High speed)
	GPIOB->OTYPER	&= ~(1<<10);	// PB8 Output type push-pull (reset state)
	GPIOB->PUPDR	|= (1<<20);	// 0x00010000 PB8 Pull-up
 	GPIOB->AFR[1]	|= (1<<8);	// 0x00000002 (AFR[1].(3~0)=0b0010): Connect TIM4 pins(PB8) to AF2(TIM3..5)
    
// TIM2 Channel 3 : PWM 1 mode
	// Assign 'PWM Pulse Period'
	TIM2->PSC	= 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
	TIM2->ARR	= 20000-1;	// Auto reload  (0.1ms * 20000= 2s : PWM Period)

	// Setting CR1 : 0x0000 (Up counting)
	TIM2->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM2->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled)
	TIM2->CR1 &= ~(1<<2);	// URS=0(Update event source Selection)g events
	TIM2->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM2->CR1 |= (1<<7);	// ARPE=1(ARR is buffered): ARR Preload Enable 
	TIM2->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM2->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 : Edge-aligned mode(reset state)
				
	// Define the corresponding pin by 'Output'  
	// CCER(Capture/Compare Enable Register) : Enable "Channel 3" 
	TIM2->CCER	|= (1<<8);	// CC3E=1: OC3(TIM2_CH3) Active(Capture/Compare 3 output enable)

	TIM2->CCER	&= ~(1<<9);	// CC3P=0: CC3 Output Polarity (OCPolarity_High : OC3으로 반전없이 출력)

	// Duty Ratio 
	TIM2->CCR3	= 10000;		// CCR3 value

	// 'Mode' Selection : Output mode, PWM 1
	// CCMR2(Capture/Compare Mode Register 2) : Setting the MODE of Ch3 or Ch4
	TIM2->CCMR2 &= ~(3<<0); // CC3S(CC3 channel)= '0b00' : Output 
	TIM2->CCMR2 |= (1<<3); 	// OC3PE=1: Output Compare 3 preload Enabl
	TIM2->CCMR2	|= (6<<4);	// OC3M: Output compare 3 mode: PWM 1mode
	TIM2->CCMR2	|= (1<<7);	// OC3CE=1: Output compare 3 Clear enable
	
	//Counter TIM2 enable
	TIM2->CR1|= (1<<0);	// CEN: Counter TIM2 enable
}

void _ADC_Init(void)
{	// ADC1: PA1(pin 41)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// (1<<0) ENABLE GPIOA CLK (stm32f4xx.h 참조)
	GPIOA->MODER |= (3<<2*1);		// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
						
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	// (1<<8) ENABLE ADC1 CLK (stm32f4xx.h 참조)

	ADC->CCR &= ~(0X1F<<0);		// MULTI[4:0]: ADC_Mode_Independent
	ADC->CCR |=  (1<<16); 		// 0x00010000 ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
        
	ADC1->CR1 &= ~(3<<24);		// RES[1:0]=0b00 : 12bit Resolution
	ADC1->CR1 &= ~(1<<8);		// SCAN=0 : ADC_ScanCovMode Disable
	ADC1->CR1 |=  (1<<5);		// EOCIE=1: Interrupt enable for EOC

	ADC1->CR2 &= ~(1<<1);		// CONT=0: ADC_Continuous ConvMode Disable
	ADC1->CR2 &= ~(3<<28);		// EXTEN[1:0]=0b00: ADC_ExternalTrigConvEdge_None
	ADC1->CR2 &= ~(1<<11);		// ALIGN=0: ADC_DataAlign_Right
	ADC1->CR2 &= ~(1<<10);		// EOCS=0: The EOC bit is set at the end of each sequence of regular conversions

	ADC1->SQR1 &= ~(0xF<<20);	// L[3:0]=0b0000: ADC Regular channel sequece length 
					// 0b0000:1 conversion)
    
        ADC1->SMPR2 |= (0x7<<(3*1));	// ADC1_CH1 Sample TIme_480Cycles (3*Channel_1)
 	//Channel selection, The Conversion Sequence of PIN1(ADC1_CH1) is first, Config sequence Range is possible from 0 to 17
	ADC1->SQR3 |= (1<<0);		// SQ1[4:0]=0b0001 : CH1

	NVIC->ISER[0] |= (1<<18);	// Enable ADC global Interrupt

	ADC1->CR2 |= (1<<0);		// ADON: ADC ON
        ADC1->CR2 |=  (1<<30); 	// swstart adc

}
void _GPIO_Init(void)
{
	// LED (GPIO G) 설정
    	RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) 설정 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) 설정 
    	RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x0200;	// GPIOF 9 : Push-pull  	
 	GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
	
	//Joy Stick SW(PORT I) 설정
	RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER 	&= ~0x000FFC00;	// GPIOI 5~9 : Input mode (reset state)
	GPIOI->PUPDR    &= ~0x000FFC00;	// GPIOI 5~9 : Floating input (No Pull-up, pull-down) (reset state)
}	

/* EXTI (EXTI8(GPIOH.8, SW0), EXTI9(GPIOH.9, SW1)) 초기 설정  */
void _EXTI_Init(void)
{
    	RCC->AHB1ENR 	|= 0x00000080;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x00004000;	// Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH PIN8~PIN15 Input mode (reset state)		(sw0~sw7)		 

        SYSCFG->EXTICR[3] |= 0x7777; 	// EXTI8,9에 대한 소스 입력은 GPIOH로 설정
        
        
	EXTI->FTSR |= (15<<12);		// EXTI12~15: Falling Trigger Enable
        
    	EXTI->IMR  |= (15<<12);  	// EXTI12~15인터럽트 mask (Interrupt Enable) 설정
        NVIC->ISER[1] |= ( 1 << 40-32); 
}

void Process_Time_Setting(void)
{
        LCD_DisplayChar(3,4,Fram_Read(1203)/10+0x30);
        LCD_DisplayText(3,5,".");
        LCD_DisplayChar(3,6,Fram_Read(1203)%10+0x30);
        LCD_DisplayChar(4,4,Fram_Read(1204)/10+0x30);
        LCD_DisplayText(4,5,".");
        LCD_DisplayChar(4,6,Fram_Read(1204)%10+0x30);
        LCD_DisplayChar(5,4,Fram_Read(1205)/10+0x30);
        LCD_DisplayText(5,5,".");
        LCD_DisplayChar(5,6,Fram_Read(1205)%10+0x30);
        LCD_DisplayChar(6,4,Fram_Read(1206)+0x30);
}
void DisplayOn(void)
{   
  LCD_Clear(RGB_YELLOW);                 //배경 노란색으로 초기화
  LCD_SetFont(&Gulim8);
  LCD_SetBackColor(RGB_YELLOW);   //배경색 노란색
  
  //이름 및 학번
  LCD_SetTextColor(RGB_BLUE);   
  LCD_DisplayText(0,0,"LCH 2017130036");
  LCD_DisplayText(1,0,"SPC monitor");
  
  //제목
  LCD_SetTextColor(RGB_BLACK);
  LCD_DisplayText(2,0,"Mode:  Wafer:");
  LCD_DisplayText(3,0," TE:   s    s");
  LCD_DisplayText(4,0," PR:   s    s");
  LCD_DisplayText(5,0," O2:   s    s");
  LCD_DisplayText(6,0," DR:");
  
  //초기상태
  LCD_SetTextColor(RGB_RED);
  LCD_DisplayText(3,0,"*");
  LCD_DisplayText(2,5,"I");
  LCD_DisplayText(2,13,"E");
  
  LCD_DisplayChar(3,4,Fram_Read(1203)/10+0x30);
  LCD_DisplayText(3,5,".");
  LCD_DisplayChar(3,6,Fram_Read(1203)%10+0x30);
  LCD_DisplayChar(4,4,Fram_Read(1204)/10+0x30);
  LCD_DisplayText(4,5,".");
  LCD_DisplayChar(4,6,Fram_Read(1204)%10+0x30);
  LCD_DisplayChar(5,4,Fram_Read(1205)/10+0x30);
  LCD_DisplayText(5,5,".");
  LCD_DisplayChar(5,6,Fram_Read(1205)%10+0x30);
  LCD_DisplayChar(6,4,Fram_Read(1206)+0x30);

  
  //초기에fram에 저장된 동작 예상시간 표시하기
}


void DelayMS(unsigned short wMS)
{   
  register unsigned short i;
  for (i=0; i<wMS; i++)
    DelayUS(1000);  // 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{   
  volatile int Dly = (int)wUS*17;
  for(; Dly; Dly--);
}

uint8_t key_flag = 0;
uint16_t KEY_Scan(void)   // input key SW0 - SW7 
{ 
  uint16_t key;
  key = GPIOH->IDR & 0xFF00;   // any key pressed ?
  if(key == 0xFF00)      // if no key, check key off
  {     
    if(key_flag == 0)
      return key;
    else
    {   
      DelayMS(10);
      key_flag = 0;
      return key;
    }
  }
  else            // if key input, check continuous key
  {   
    if(key_flag != 0)   // if continuous key, treat as no key input
      return 0xFF00;
    else         // if new key,delay for debounce
    {   
      key_flag = 1;
      DelayMS(10);
      return key;
    }
  }
}
void BEEP(void)         // Beep for 20 ms 
{    
  GPIOF->ODR |= (1<<9);   // PF9 'H' Buzzer on
  DelayMS(20);      // Delay 20 ms
  GPIOF->ODR &= ~(1<<9);   // PF9 'L' Buzzer off
}