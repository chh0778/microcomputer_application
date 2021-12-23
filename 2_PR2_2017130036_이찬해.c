//////////////////////////////////////////////////////////////////////
// PR2: 반도체 공정 제어기
// 제출자: 2017130036_이찬해
// 주요내용
//웨이퍼가 들어와서 여러 화학공정을 거치는 반도체 공정장비의
//process chamber 내부를 제어하는 프로세스 제어 프로그램
//주요 스펙
//1. 세부입력모드---> ADC1_IN1 사용
//가변저항에 의해 변화하는 전압을 0~4095의 디지털 값으로 변화시키는 ADC1을 사용
//간단한 수식을 이용하여 0~4095의 수를 7등분하여 온도과정, 압력과정, O2과정이 걸리는 시간으로 변환하고
//10등분하여 PLASMA과정에서 PWM의 Duty Ratio를 조절한다.

//2. 스위치는 sw4,5,6,7 (GPIOH), LED는 0번부터 7번 모두(GPIOG) 사용한다.
//SW4: 세부 입력모드에서 입력값 결정 및 저장
//SW5: 모드변환(입력<->실행)
//SW6: 각 모드에서 세부 프로세스 시작
//SW7: 세부입력모드에서 입력할 파라미터 변경

//3. FRAM을 이용하여 세부입력모드의 과정 시간을 저장
//SW4를 누르면 세부입력모드에서 해당 과정의 시간을 저장한다.
//저장한 값들은 프로그램을 재시작했을 때 LCD화면에 출력하기위해 사용한다.
//저장한 값들은 공정 실행을 할 때 목표 공정 시간을 LCD 화면에 출력하기 위해 사용한다.

//4. 타이머 사용
//TIM3을 사용하여 0.1s 마다 변하는 과정시간을 LCD화면에 디스플레이하기
//TIM2를 사용하여 PLASMA공정의 Duty Ratio에 따른 pwm 신호 발생을 위해 사용
///////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"
#include "FRAM.h"

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
//0: 입력모드 대기, 1: 실행모드 대기, 2: 세부입력모드, 3: 실행모드
int flag=0;

//0: 세부입력 진입, 1: 온도값 입력, 2: 압력값 입력, 3: O2값 입력, 4: plasma 세기
int processing_op =0;
//프로세스가 끝나면 알려주는 flag
//0: 대기상태, 1: 온도 프로세스, 2: 압력 프로세스, 3: o2 프로세스, 4:플라즈마 프로세스
int process_done=0;

//fram 저장 조건에 의해 히팅 시간의 10을 곱한 값을 저장한다.
//온도값           //압력값             // 산소값          //plasma duty ratio
   int te;              int pr;                 int o2;                  int dr;

//웨이퍼 공정을 실행시키는 flag
//0: 실행중 아님, 1: 실행중
int process_flag=0;

//0.1s 인터럽트마다 1씩 증가한다.
//웨이퍼 공정 실행중에 시간증가를 LCD화면에 표현하기 위해 사용
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
  if(processing_op==1) {           //온도값 입력
    LCD_SetTextColor(RGB_RED);	//글자색
    LCD_DisplayChar(3,4,te/10 + 0x30);
    LCD_DisplayText(3,5,".");
    LCD_DisplayChar(3,6,te%10 + 0x30);
  }
  else if(processing_op==2){       //압력값 입력
    LCD_SetTextColor(RGB_RED);	//글자색
    LCD_DisplayChar(4,4,pr/10 + 0x30);
    LCD_DisplayText(4,5,".");
    LCD_DisplayChar(4,6,pr%10 + 0x30);
  }
  else if(processing_op==3)       //o2값 입력
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
    if(process_flag==1)                       //실행모드에서 SW7을 누르면 if문 실행
    {
      flag=3;                 //실행 후 대기모드
      GPIOG->ODR |= 0x04;   // LED2 on
      LCD_DisplayText(2,13,"W");
      GPIOG->ODR |= 0x08;   // LED3 on
      DelayMS(500);
      
//공정 시작
      process_done=1;                           //온도과정 실행 상태를 저장하는flag                  
      
      Process_Time_Setting();                   //공정 LCD 화면 세팅
      
//온도 공정
      GPIOG->ODR |= 0x10;                    // LED4 on
      LCD_DisplayText(3,0,"*");
      while(process_done!=2){}                  //온도공정 시간이 되기 전까지 while문에 갖힘
      
      LCD_DisplayText(3,0," ");
      GPIOG->ODR &= ~0x10;                 //LED4 OFF
      
//압력 프로세스
      LCD_DisplayText(4,0,"*"); 
      GPIOG->ODR |= 0x20;                    // LED5 on
      while(process_done!=3){}                  //압력공정 시간이 되기 전까지 while문에 갖힘
      
      LCD_DisplayText(4,0," ");
      GPIOG->ODR &= ~0x20;                    // LED5 off
      
//O2 프로세스 & 플라즈마 프로세스
      LCD_DisplayText(5,0,"*");
      LCD_DisplayText(6,0,"*");
      GPIOG->ODR |= 0x40;                   // LED6 oN 
      GPIOG->ODR |= 0x80;                   // LED7 oN 
      
      int i=Fram_Read(1206);
      TIM2->CCR3 = i*2000;                       //플라즈마 공정에 해당하는 Duty Ratio 설정
      
      while(process_done!=0){}                  //o2 % 플라즈마 공정 시간이 되기 전까지 while문에 갖힘
      
      TIM2->CCR3  = 0;		        // 플라즈마 공정 중지
      LCD_DisplayText(5,0," ");
      LCD_DisplayText(6,0," ");
      GPIOG->ODR &= ~0x40;                   // LED6 oFF
      GPIOG->ODR &= ~0x80;                   // LED7 oFF
      
      DelayMS(500);
      LCD_DisplayText(2,13,"E");
      GPIOG->ODR &= ~0x08;                      // LED3 off
      process_flag=0;
    }
  }
}

void EXTI15_10_IRQHandler(void)		
{
  //sw4 세부입력모드 저장
  if(EXTI->PR & (1<<12))                   // EXTI12Interrupt Pending(발생) 여부?
  {
    EXTI->PR |= (1<<12); 	
    if(flag==2){                       //세부 입력 모드
      if(processing_op==1) {           //온도공정 시간
        Fram_Write(1203,te); 
      }
      else if(processing_op==2){       //압력공정 시간
        Fram_Write(1204,pr); 
      }
      else if(processing_op==3) {      //o2공정 시간
        Fram_Write(1205,o2); 
      }
      else if(processing_op==4)  {     //plasma Duty Ratio 값
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
      LCD_DisplayText(2,5,"R");                 //실행모드'R'임을 LCD로 출력
    }
    else if(flag==1){                       //실행 대기 --> 입력 대기
      flag=0;
      GPIOG->ODR |= 0x01;   // LED0 on
      GPIOG->ODR &= ~0x02;   // LED1 off
      LCD_SetTextColor(RGB_RED);
      LCD_DisplayText(2,5,"I");                 //입력모드'I'임을 LCD로 출력
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
      process_flag=1;                           //main 문 안에 있는 실행 명령어를 실행시키기 위한 flag on
    }
    else if(flag==2){                       //세부 입력 모드 --> 입력대기
      flag=0;
      GPIOG->ODR &= ~0x04;   // LED2 off
      GPIOG->ODR &= ~0xF0;   // LED4~7 off
      LCD_DisplayText(3,0,"*");
      LCD_DisplayText(4,0," ");
      LCD_DisplayText(5,0," ");
      LCD_DisplayText(6,0," ");
      processing_op =0;
    }
    else if(flag==3){                       //실행 (실행종료 후) --> 실행대기
      flag=1;
      GPIOG->ODR &= ~0x04;          // LED2 off
      LCD_DisplayText(3,0,"*");           //LCD 초기화면처럼 * 표시
    }
  }
  //sw7 세부입력모드에서 항목 선택
  else if(EXTI->PR & (1<<15))                   // EXTI15 Interrupt Pending(발생) 여부?
  {
    EXTI->PR |= (1<<15); 	
    if(flag==2){                       //세부 입력 모드 선택
      processing_op++;
      if(processing_op>4){
        processing_op=1;
      }
      switch(processing_op)
      {
      case 1:
        GPIOG->ODR &= ~0x80;                    //LED 7 OFF
        GPIOG->ODR |= 0x10;                    //LED 4 ON
        LCD_DisplayText(6,0," ");
        LCD_DisplayText(3,0,"*");
        break;
      case 2:
        GPIOG->ODR &= ~0x10;                    //LED 4 OFF
        GPIOG->ODR |= 0x20;                    //LED 5 ON
        LCD_DisplayText(3,0," ");
        LCD_DisplayText(4,0,"*");
        break;
      case 3:
        GPIOG->ODR &= ~0x20;                    //LED 5 OFF
        GPIOG->ODR |= 0x40;                    //LED 6 ON
        LCD_DisplayText(4,0," ");
        LCD_DisplayText(5,0,"*");                    
        break;
      case 4:
        GPIOG->ODR &= ~0x40;                    //LED 6 OFF
        GPIOG->ODR |= 0x80;                    //LED 7 ON
        LCD_DisplayText(5,0," ");
        LCD_DisplayText(6,0,"*");                    
        break;
      }
    }
  }
}

// 0.1s Interrupt, 공정 실행 중 LCD 화면에 시간 출력을 위한 TIM
void TIM3_IRQHandler(void)  	
{
  
  TIM3->SR &= ~(1<<0);	// Interrupt flag Clear
  if(process_done==1){    //온도 프로세스 진행
    
    LCD_DisplayChar(3,9,cnt/10+0x30);
    LCD_DisplayText(3,10,".");
    LCD_DisplayChar(3,11,cnt%10+0x30);
    cnt++;
    if(cnt>Fram_Read(1203)){
      process_done=2;
      cnt=0;
    }
  }
  else if(process_done==2){    //압력 프로세스 진행
    
    LCD_DisplayChar(4,9,cnt/10+0x30);
    LCD_DisplayText(4,10,".");
    LCD_DisplayChar(4,11,cnt%10+0x30);
    cnt++;
    
    if(cnt>Fram_Read(1204)){
      process_done=3;
      cnt=0;
    }
  }
  else if(process_done==3){    //o2 프로세스 진행
    LCD_DisplayChar(5,9,cnt/10+0x30);
    LCD_DisplayText(5,10,".");
    LCD_DisplayChar(5,11,cnt%10+0x30);
    cnt++;
    
    if(cnt>Fram_Read(1205)){
      process_done=0;
      cnt=0;
    }
  }
}

//PLASMA 공정에서 PWM 신호 외부출력을 위한 타이머
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
  // Clock Enable : GPIOB & TIMER2
  RCC->AHB1ENR	|= (1<<1);	// GPIOB CLOCK Enable
  RCC->APB1ENR 	|= (1<<0);	// TIMER2 CLOCK Enable 
  
  // PB10을 출력설정하고 Alternate function(TIM2_CH3)으로 사용 선언 : PWM 출력
  GPIOB->MODER 	|= (2<<20);	// PB10Output Alternate function mode					
  GPIOB->OSPEEDR 	|= (3<<20);	// PB10 Output speed (100MHz High speed)
  GPIOB->OTYPER	&= ~(1<<10);	// PB10 Output type push-pull (reset state)
  GPIOB->PUPDR	|= (1<<20);	//  PB10 Pull-up
  GPIOB->AFR[1]	|= (1<<8);	// (AFR[1].(3~0)): Connect TIM2 pins(PB10) to AF1(TIM1,2)
  
  // TIM2 Channel 3 : PWM 1 mode
  // Assign 'PWM Pulse Period'
  TIM2->PSC	= 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
  TIM2->ARR	= 20000-1;	// Auto reload  (0.1ms * 20000= 2s : PWM Period)
  
  // Setting CR1 : 0x0000 (Up counting)
  TIM2->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
  TIM2->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled)
  TIM2->CR1 &= ~(1<<2);	// URS=0(Update event source Selection)g events
  TIM2->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
  TIM2->CR1 |= (1<<7);	        // ARPE=1(ARR is buffered): ARR Preload Enable 
  TIM2->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
  TIM2->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 : Edge-aligned mode(reset state)
  
  // Define the corresponding pin by 'Output'  
  // CCER(Capture/Compare Enable Register) : Enable "Channel 3" 
  TIM2->CCER	|= (1<<8);	// CC3E=1: OC3(TIM2_CH3) Active(Capture/Compare 3 output enable)
  
  TIM2->CCER	&= ~(1<<9);	// CC3P=0: CC3 Output Polarity (OCPolarity_High : OC3으로 반전없이 출력)
  
  // Duty Ratio 
  TIM2->CCR3	= 0;		// CCR3 value
  
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
}	

/* EXTI (EXTI12~15(SW4~SW7))초기 설정  */
void _EXTI_Init(void)
{
  RCC->AHB1ENR 	|= 0x00000080;	// RCC_AHB1ENR GPIOH Enable
  RCC->APB2ENR 	|= 0x00004000;	// Enable System Configuration Controller Clock
  
  GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH PIN8~PIN15 Input mode (reset state)		(sw0~sw7)		 
  
  SYSCFG->EXTICR[3] |= 0x7777; 	// EXTI12~15에 대한 소스 입력은 GPIOH로 설정
  
  
  EXTI->FTSR |= (15<<12);		// EXTI12~15: Falling Trigger Enable
  
  EXTI->IMR  |= (15<<12);  	// EXTI12~15인터럽트 mask (Interrupt Enable) 설정
  NVIC->ISER[1] |= ( 1 << 40-32); 
}

void Process_Time_Setting(void)
{
  LCD_DisplayText(3,9,"0.0");
  LCD_DisplayText(4,9,"0.0");
  LCD_DisplayText(5,9,"0.0");
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
  
  LCD_DisplayText(3,9,"0.0");
  LCD_DisplayText(4,9,"0.0");
  LCD_DisplayText(5,9,"0.0");
  
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