//////////////////////////////////////////////////////////////////////
// PR1: 자동차 추종 시스템
// 제출자: 2017130036_이찬해
// 주요내용

//1. 선도차와의 거리 측정
//ADC2_IN1(PA1)이용, 가변저항에 의한 전압변화를 선도차와의 거리로 변환(0~3.3v-->5~38m)
//TIM1_CH3을 ADC 트리거로 사용
//TIM1_PSC:  1680, TIM1_ARR = 150000 

//2. 추종차 속도 제어
//TIM2_CH4 PWM 모드 사용
//TIM2_PSC: 8400, TIM2_ARR: 50000
//선도차와의 거리에 따라 TIM2_CCR4조절-->PWM DUTY_RATIO 변경
//모터구동 대신 LED ON-OFF 주기로 PWM  신호 확인

//3. 자동차 출발/ 정지 명령
//스위치입력
//sw0(GPIO)입력시 MOVE, SW7(EXTI)입력시 STOP
//PC 프로그램 입력
//USART1 사용, M 전송시 MOVE, S 전송시 STOP
//MOVE상태: 막대표시, 거리표시, duty_ratio표시, TIM2_CH4 PWM 에 따른 LED 동작
//STOP상태: 막대표시제거, 거리표시:0, duty_ratio표시:00, LED ON 유지
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

//USART 관련 함수
void USART1_Init(void);
void USART_BRR_Configuration(uint32_t USART_BaudRate);
void SerialSendChar(uint8_t c);
void SerialSendString(char *s);

//ADC 관련 함수
void _ADC_Init(void);

uint16_t KEY_Scan(void);

//타이머 설정
void TIMER2CH4_PWM_Init(void);
void TIMER1_Init(void);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

//추가 변수들
uint16_t ADC_Value, distance, pre_distance, duty_ratio10;
double Voltage;
uint8_t send_distance[2];

//move, stop flag
//0: stop 상태
//1: move 상태
//2: 현 상태 유지
int state_flag=0;
char state_fram;

//
void ADC_IRQHandler(void)
{
  ADC2->SR &= ~(1<<1);		// EOC flag clear
  ADC_Value = ADC2->DR;		// Reading ADC result
  Voltage = ADC_Value * 3.3  / 4095;   // 3.3 : 4095 =  Volatge : ADC_Value 
  
  distance = Voltage*10+5;                //거리값 계산(0~38)
  
  //ADC 인터럽트 주기인 300ms마다 usart를 통해 pc로 거리 값 전송
  sprintf(send_distance,"%2d",distance);
  SerialSendString(send_distance);
  SerialSendChar('m');
  SerialSendChar('\n');
  
  
  //추종차 속도 제어 및 duty_ratio 설정
  if(distance>=5 && distance<=8)
  {
    TIM2->CCR4 = 5000;
    duty_ratio10=1;                         //duty_ratio 10의 자리 1
  }
  else if (distance<=12) 
  {
    TIM2->CCR4 = 10000;
    duty_ratio10=2;                         //duty_ratio 10의 자리 2
  }
  else if (distance<=16) 
  {
    TIM2->CCR4 = 15000;
    duty_ratio10=3;                         //duty_ratio 10의 자리 3
  }
  else if (distance<=20)
  {
    TIM2->CCR4 = 20000;
    duty_ratio10=4;                         //duty_ratio 10의 자리 4
  }
  else if (distance<=24) 
  {
    TIM2->CCR4 = 25000;
    duty_ratio10=5;                         //duty_ratio 10의 자리 5
  }
  else if (distance<=28) 
  {
    TIM2->CCR4 = 30000;
    duty_ratio10=6;                         //duty_ratio 10의 자리 6
  }
  else if (distance<=32) 
  {
    TIM2->CCR4 = 35000;
    duty_ratio10=7;                         //duty_ratio 10의 자리 7
  }
  else if (distance<=36) 
  {
    TIM2->CCR4 = 40000;
    duty_ratio10=8;                         //duty_ratio 10의 자리 8
  }
  else if (distance<=38) 
  {
    TIM2->CCR4 = 45000;
    duty_ratio10=9;                         //duty_ratio 10의 자리 9
  }
  
  //이전값과 다를 때에만 LCD 화면 갱신
  if(pre_distance!=distance)
  {
      LCD_SetTextColor(RGB_BLUE); 
    //LCD에 거리 값 표시
    LCD_DisplayChar(2,17,distance/10+0x30);
    LCD_DisplayChar(2,18,distance%10+0x30);
    
    //duty ratio 표시
    LCD_DisplayChar(3,6,duty_ratio10+0x30);
    
    //LCD에 거리를 막대로 표시
    LCD_SetBrushColor(RGB_YELLOW);
    LCD_DrawFillRect(15,26,115, 11) ;
    LCD_SetBrushColor(RGB_RED);
    LCD_DrawFillRect(15,26,(distance-4)/34.0*115, 11) ;
  }
  pre_distance=distance;
}

int main(void)
{
  LCD_Init();                                              // LCD 구동 함수
  DelayMS(1000);                                      // LCD구동 딜레이
  DisplayOn();                                            //LCD 초기화면구동 함수
  Fram_Init();                                             // FRAM 초기화 H/W 초기화
  Fram_Status_Config();                              // FRAM 초기화 S/W 초기화
  state_fram=Fram_Read(1126);                   //fram에 저장된 이전 상태 저장
  if(state_fram=='S') state_flag=0;                //stop 상태일 경우 state_flag=0
  else if (state_fram=='M') state_flag=1;       //move 상태일 경우 state_flag=1
  
  _GPIO_Init();                                         //GPIO 초기화
  _EXTI_Init();	                                       //EXTI 초기화
  _ADC_Init();                                           //ADC 초기설정
  
  USART1_Init();                                         //USART 초기설정
  GPIOG->ODR &= 0x00;                                          //LED0~7 Off 
  
  TIMER2CH4_PWM_Init();                                 //타이머 초기화
  TIMER1_Init();                                                //타이머 초기화
  LCD_SetTextColor(RGB_BLUE);   
  
  while(1)
  {
    if(KEY_Scan()==SW0_PUSH)            //Move 상태로 변경
    {
      state_flag=1;
    }
    
    //stop 상태
    if(state_flag==0)
    {
      ADC2->CR2 &= ~(1<<0);                  //ADC 2 종료
      TIM2->CCR4	= 0;                            // PWM LOW유지하기 위한 TIM2_CCR4=0
      LCD_DisplayText(2,17," 0");                //LCD: 추종차와의 거리 0표시
      LCD_DisplayText(3,6,"00");                //LCD: DUTY RATIO 00 표시
      LCD_DisplayText(3,18,"S"); 	         // LCD: stop 상태 표시
      LCD_SetBrushColor(RGB_YELLOW);   //LCD: 막대 없애기
      LCD_DrawFillRect(15,26,115, 11) ;
      Fram_Write(1126,'S');                      //FRAM에 현 상태 저장(STOP)
      SerialSendChar('0');
      SerialSendChar('m');
      state_flag=2;                                  //현상태 유지 FLAG
    }
    
    //move 상태
    else if( state_flag ==1)
    {
      ADC2->CR2 |= (1<<0);                  //ADC 2 종료
      LCD_DisplayChar(2,17,distance/10+0x30);                      //LCD: 추종차와의 거리 표시
      LCD_DisplayChar(2,18,distance%10+0x30);
      LCD_DisplayChar(3,6,duty_ratio10+0x30);                      //dutyratio 표시
      LCD_DisplayText(3,18,"M"); 	                                   // LCD: move 상태 표시
      LCD_SetBrushColor(RGB_RED);
      LCD_DrawFillRect(15,26,(distance-4)/34.0*115, 11) ;       //LCD: 막대 표시
      Fram_Write(1126,'M');                                             //FRAM에 현 상태 저장(MOVE)
      state_flag=2;                                                             //현상태 유지 FLAG
    }
  }
}

int send_ch;
void USART1_IRQHandler(void)
{       
  // TX Buffer Empty Interrupt
  if ( (USART1->SR & USART_SR_TXE) )	// USART_SR_TXE=(1<<7)
  {
    USART1->CR1 &= ~(1<<7);	// TXE interrupt Disable 
  } 
  // RX Buffer Full interrupt
  if ( (USART1->SR & USART_SR_RXNE) ) 	// USART_SR_RXNE=(1<<5) 
  {
    char ch;
    ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// 수신된 문자 저장
    if(ch=='M')
    {
      state_flag=1;             //MOVE 상태로 변환
    }
    else if(ch=='S')
    {
      state_flag=0;             //STOP 상태로 변환
    }  
  } 
  // DR 을 읽으면 SR.RXNE bit(flag bit)는 clear 된다. 즉 clear 할 필요없음 
}

/* EXTI5~9 인터럽트 핸들러(ISR: Interrupt Service Routine) */
void EXTI15_10_IRQHandler(void)		
{
  if(EXTI->PR & (1<<15))                   // EXTI15 Interrupt Pending(발생) 여부?
  {
    EXTI->PR |= (1<<15); 		// Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
    state_flag=0;                               //SW7 입력시----> stop 상태로 변환
  }
}

//ADC2 사용, 선도차와의 거리를 가변저항 값에 따른 전압값으로 표현
void _ADC_Init(void)
{   	// ADC2: PA1(pin 41)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// (1<<0) ENABLE GPIOA CLK (stm32f4xx.h 참조)
  GPIOA->MODER |= (3<<2*1);		// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
  
  RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;	// (1<<8) ENABLE ADC2 CLK (stm32f4xx.h 참조)
  
  ADC->CCR &= ~(0X1F<<0);		// MULTI[4:0]: ADC_Mode_Independent
  ADC->CCR |=  (1<<16); 		// ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
  
  ADC2->CR1 &= ~(3<<24);		// RES[1:0]=      : 12bit Resolution
  ADC2->CR1 &= ~(1<<8);		// SCAN=0 : ADC_ScanCovMode Disable
  ADC2->CR1 |=  (1<<5);		// EOCIE=1: Interrupt enable for EOC
  
  ADC2->CR2 &= ~(1<<1);		// CONT=0: ADC_Continuous ConvMode Disable
  ADC2->CR2 |=  (2<<28);		// EXTEN[1:0]: ADC_ExternalTrigConvEdge_Enable(Falling Edge)
  ADC2->CR2 |= (2<<24);	        // EXTSEL[3:0]: External event select for regular gruop
  ADC2->CR2 &= ~(1<<11);		// ALIGN=0: ADC_DataAlign_Right
  ADC2->CR2 &= ~(1<<10);		// EOCS=0: The EOC bit is set at the end of each sequence of regular conversions
  
  ADC2->SQR1 &= ~(0xF<<20);	// L[3:0]=0b0000: ADC Regular channel sequece length 
  // 0b0000:1 conversion)
  //Channel selection, The Conversion Sequence of PIN1(ADC2_CH1) is first, Config sequence Range is possible from 0 to 17
  ADC2->SQR3 |= (1<<0);		// SQ1[4:0]=0b0001 : CH1
  ADC2->SMPR2 |= (0x7<<(3*1));	// ADC2_CH1 Sample TIme_480Cycles (3*Channel_1)
  //Channel selection, The Conversion Sequence of PIN1(ADC2_CH1) is first, Config sequence Range is possible from 0 to 17
  
  NVIC->ISER[0] |= (1<<18);	// Enable ADC global Interrupt
  
  ADC2->CR2 &= ~(1<<0);		// ADON: ADC OFF
}

//추종차와의 거리를 갱신하는 주기 300ms( oc mode )
void TIMER1_Init(void)
{
  // PE13 TIM1_CH3
  // PE13을 출력설정하고 Alternate function(TIM1_CH3)으로 사용 선언
  RCC->AHB1ENR   |= (1<<4);   // 0x08, RCC_AHB1ENR GPIOE Enable 
  
  GPIOE->MODER    |= (2<<26);   // (MODER.(25,24)=0b10), GPIOE PIN15 Output Alternate function mode                
  GPIOE->OSPEEDR    |= (3<<26);   // (OSPEEDER.(25,24)=0b11), GPIOE PIN15 Output speed (100MHz High speed)
  GPIOE->OTYPER   &= ~(1<<13);   //  GPIOE PIN15 Output type push-pull (reset state)
  GPIOE->PUPDR    |= (1<<13);    // GPIOE PIN15 Pull-up
  // PE13 ==> TIM1_CH3
  GPIOB->AFR[1]   |=(1<<20) ;  // (AFR[1].(7~4)=0b0001): Connect TIM1 pins(PE15) to AF1(TIM1)
  
  RCC->APB2ENR |= 0x01;   // RCC_APB1ENR TIMER1 Enable

  // Setting CR1 : 0x0000 
  TIM1->CR1 &= ~(1<<4);   // DIR=0(Up counter)(reset state)
  TIM1->CR1 &= ~(1<<1);   // UDIS=0(Update event Enabled): By one of following events
  //  Counter Overflow/Underflow, 
  //  Setting the UG bit Set,
  //  Update Generation through the slave mode controller 
  // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
  TIM1->CR1 &= ~(1<<2);   // URS=0(Update Request Source  Selection):  By one of following events
  //   Counter Overflow/Underflow, 
  // Setting the UG bit Set,
  //   Update Generation through the slave mode controller 
  // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
  TIM1->CR1 &= ~(1<<3);   // OPM=0(The counter is NOT stopped at update event) (reset state)
  TIM1->CR1 &= ~(1<<7);   // ARPE=0(ARR is NOT buffered) (reset state)
  TIM1->CR1 &= ~(3<<8);    // CKD(Clock division)=00(reset state)
  TIM1->CR1 &= ~(3<<5);    // CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
  // Center-aligned mode: The counter counts UP and DOWN alternatively
  TIM1->BDTR |= (1<<15);
    //////////////////////////////////////////////////주기변환중
  // Deciding the Period
  TIM1->PSC = 1680-1;   // Prescaler 168,000,000Hz/1680= 100,000 Hz (100ms)  (1~65536)
  TIM1->ARR = 15000-1;   // Auto reload  100ms * 15,000 = 150ms
  
  // Clear the Counter
  TIM1->EGR |= (1<<0);   // UG(Update generation)=1 
  // Re-initialize the counter(CNT=0) & generates an update of registers   
  
  TIM1->CCMR2 &= ~(3<<0); // CC3S(CC3 channe3) = '0b00' : Outsput 
  TIM1->CCMR2 &= ~(1<<2); // OC3FE=0: Output Compare 3 Fast disable 
  TIM1->CCMR2 &= ~(1<<3); // OC3PE=0: Output Compare 3 preload disable(CCR3에 언제든지 새로운 값을 loading 가능) 
  TIM1->CCMR2 |= (3<<4);   // OC3M(Output Compare 3 Mode : (toggles)
  // OC1REF toggles when CNT = CCR1
  
  // CCER(Capture/Compare Enable Register) : Enable "Channel 3" 
  TIM1->CCER |= (1<<8);   // CC3E=1: CC3 channel Output Enable
  TIM1->CCER &= ~(1<<9);   // CC3P=0: CC3 channel Output Polarity (OCPolarity_High : OC3으로 반전없이 출력)  
  TIM1->CCR3 = 6000;   // TIM1 CCR3 TIM1_Pulse
  
  TIM1->CR1 |= (1<<0);   // disable the Tim1 Counter (clock enable)
}


//pwm으로 추종 차에 따른 모터 속도 제어 구현
//실제로는 PWM 제어로 LED  on-off 주기 확인
void TIMER2CH4_PWM_Init(void)
{  
  // 모터펄스(PWM)핀:PB11(TIM2 CH4)
  RCC->AHB1ENR	|= (1<<1);	// GPIOB Enable
  RCC->APB1ENR 	|=  (1<<0);	// TIMER2 Enable 
  
  GPIOB->MODER 	|= (2<<22);	// PA0 Output Alternate function mode					
  GPIOB->OSPEEDR 	|= (3<<22);	// PA0 Output speed (100MHz High speed)
  GPIOB->OTYPER	&= ~(1<<11);	// PA0 Output type push-pull (reset state)
  GPIOB->AFR[1]	|= (1<<12); 
  // TIM2Channel 4 : PWM 1 mode
  // Assign 'PWM Pulse Period'
  TIM2->PSC	= 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
  TIM2->ARR	= 50000-1;	// Auto reload  (0.1ms *50000 =5S  : PWM Period)
  
  // Define the corresponding pin by 'Output'  
  // CCER(Capture/Compare Enable Register) : Enable "Channel 4" 
  TIM2->CCER	|= (1<<12);	// CC4E=1: OC1(TIM2_CH4) Active(Capture/Compare 4output enable)
  // 해당핀(40번)을 통해 신호출력
  TIM2->CCER	&= ~(1<<13);	// CC4P=0: CC4 output Polarity High (OC4으로 반전없이 출력)
  
  // 'Mode' Selection : Output mode, PWM 1
  // CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
  TIM2->CCMR2 	&= ~(3<<8); 	// CC4S(CC4 channel): Output 
  TIM2->CCMR2 	|= (1<<11); 	// OC4PE=1: Output Compare4 preload Enable
  
  TIM2->CCMR2	|= (6<<12);	// OC4M: Output compare 4 mode: PWM 2 mode
  TIM2->CCMR2	|= (1<<15);	// OC4CE: Output compare 4 Clear enable
  
  // CR1 : Up counting & Counter TIM5 enable
  TIM2->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
  TIM2->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
  TIM2->CR1 	&= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
  TIM2->CR1	|= (1<<7);	// ARPE: Auto-reload preload enable
  TIM2->EGR |= (1<<0);    // UG: Update generation   

  TIM2->CR1	|= (1<<0);	// CEN: Counter TIM2 enable
}

void _GPIO_Init(void)
{
  // LED (GPIO G) 설정
  RCC->AHB1ENR   |=  0x00000040;   // RCC_AHB1ENR : GPIOG(bit#6) Enable                     
  GPIOG->MODER    |=  0x00005555;   // GPIOG 0~7 : Output mode (0b01)                  
  GPIOG->OTYPER   &= ~0x00FF;   // GPIOG 0~7 : Push-pull  (GP8~15:reset state)   
  GPIOG->OSPEEDR    |=  0x00005555;   // GPIOG 0~7 : Output speed 25MHZ Medium speed 
  
  // SW (GPIO H) 설정 
  RCC->AHB1ENR    |=  0x00000080;   // RCC_AHB1ENR : GPIOH(bit#7) Enable                     
  GPIOH->MODER    &= ~0xFFFF0000;   // GPIOH 8~15 : Input mode (reset state)            
  GPIOH->PUPDR    &= ~0xFFFF0000;   // GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state
  
  // Buzzer (GPIO F) 설정 
  RCC->AHB1ENR   |=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable                     
  GPIOF->MODER    |=  0x00040000;   // GPIOF 9 : Output mode (0b01)                  
  GPIOF->OTYPER    &= ~0x0200;   // GPIOF 9 : Push-pull     
  GPIOF->OSPEEDR    |=  0x00040000;   // GPIOF 9 : Output speed 25MHZ Medium speed 
}


void USART1_Init(void)
{
  // USART1 : TX(PA9)
  RCC->AHB1ENR	|= (1<<0);	// RCC_AHB1ENR GPIOA Enable
  GPIOA->MODER	|= (2<<2*9);	// GPIOB PIN9 Output Alternate function mode					
  GPIOA->OSPEEDR	|= (3<<2*9);	// GPIOB PIN9 Output speed (100MHz Very High speed)
  GPIOA->AFR[1]	|= (7<<4);	// Connect GPIOA pin9 to AF7(USART1)
  
  // USART1 : RX(PA10)
  GPIOA->MODER 	|= (2<<2*10);	// GPIOA PIN10 Output Alternate function mode
  GPIOA->OSPEEDR	|= (3<<2*10);	// GPIOA PIN10 Output speed (100MHz Very High speed
  GPIOA->AFR[1]	|= (7<<8);	// Connect GPIOA pin10 to AF7(USART1)
  
  RCC->APB2ENR	|= (1<<4);	// RCC_APB2ENR USART1 Enable
  
  USART_BRR_Configuration(9600); // USART Baud rate Configuration
  
  USART1->CR1	&= ~(1<<12);	// USART_WordLength 8 Data bit
  USART1->CR1	&= ~(1<<10);	// NO USART_Parity
  USART1->CR1	|= (1<<2);	// 0x0004, USART_Mode_RX Enable
  USART1->CR1	|= (1<<3);	// 0x0008, USART_Mode_Tx Enable
  USART1->CR2	&= ~(3<<12);	// 0b00, USART_StopBits_1
  USART1->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
  
  USART1->CR1 	|= (1<<5);	// 0x0020, RXNE interrupt Enable
  USART1->CR1 	|= (1<<7); 	// 0x0080, TXE interrupt Enable
  
  NVIC->ISER[1]	|= (1<<(37-32));// Enable Interrupt USART1 (NVIC 37번)
  USART1->CR1 	|= (1<<13);	//  0x2000, USART1 Enable
  
}

void SerialSendChar(uint8_t Ch) // 1문자 보내기 함수
{
  while((USART1->SR & USART_SR_TXE) == RESET); // USART_SR_TXE=(1<<7), 송신 가능한 상태까지 대기
  
  USART1->DR = (Ch & 0x01FF);	// 전송 (최대 9bit 이므로 0x01FF과 masking)
}

void SerialSendString(char *str) // 여러문자 보내기 함수
{
  while (*str != '\0') // 종결문자가 나오기 전까지 구동, 종결문자가 나온후에도 구동시 메모리 오류 발생가능성 있음.
  {
    SerialSendChar(*str);	// 포인터가 가르키는 곳의 데이터를 송신
    str++; 			// 포인터 수치 증가
  }
}

// Baud rate  
void USART_BRR_Configuration(uint32_t USART_BaudRate)
{ 
  uint32_t tmpreg = 0x00;
  uint32_t APB2clock = 84000000;	//PCLK2_Frequency
  uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
  
  // Determine the integer part 
  if ((USART1->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
  {                                         // USART1->CR1.OVER8 = 1 (8 oversampling)
    // Computing 'Integer part' when the oversampling mode is 8 Samples 
    integerdivider = ((25 * APB2clock) / (2 * USART_BaudRate));    
  }
  else  // USART1->CR1.OVER8 = 0 (16 oversampling)
  {	// Computing 'Integer part' when the oversampling mode is 16 Samples 
    integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));    
  }
  tmpreg = (integerdivider / 100) << 4;
  
  // Determine the fractional part 
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
  
  // Implement the fractional part in the register 
  if ((USART1->CR1 & USART_CR1_OVER8) != 0)	// 8 oversampling
  {
    tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
  }
  else 						// 16 oversampling
  {
    tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
  }
  
  // Write to USART BRR register
  USART1->BRR = (uint16_t)tmpreg;
}

/* EXTI (EXTI8(GPIOH.8, SW0), EXTI9(GPIOH.9, SW1)) 초기 설정  */
void _EXTI_Init(void)
{
  RCC->AHB1ENR 	|= 0x00000080;	// RCC_AHB1ENR GPIOH Enable
  RCC->APB2ENR 	|= 0x00004000;	// Enable System Configuration Controller Clock
  
  GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH PIN8~PIN15 Input mode (reset state)	 
  
  SYSCFG->EXTICR[3] |= (7<<12); 	// EXTI8,9에 대한 소스 입력은 GPIOH로 설정
  // EXTI8 <- PH8, EXTI9 <- PH9 
  // EXTICR3(EXTICR[2])를 이용 
  // reset value: 0x0000	
  
  EXTI->FTSR |= (1<<15);		// EXTI8: Falling Trigger Enable
  EXTI->IMR  |= (1<<15);  	// EXTI8,9 인터럽트 mask (Interrupt Enable) 설정
  
  NVIC->ISER[1] |= ( 1 << 40-32 );   // 0x00800000
  // Enable 'Global Interrupt EXTI8,9'
  // Vector table Position 참조
}

void DisplayOn(void)
{   
  LCD_Clear(RGB_YELLOW);                 //배경 노란색으로 초기화
  LCD_SetFont(&Gulim8);
  LCD_SetBackColor(RGB_YELLOW);   //배경색 노란색
  
  //이름 및 학번
  LCD_SetTextColor(RGB_BLUE);   
  LCD_DisplayText(0,0,"LCH 2017130036");
  
  //제목
  LCD_SetTextColor(RGB_BLACK);   
  LCD_DisplayText(1,0,"Tracking Car");
  LCD_DisplayText(2,0,"D:");
  LCD_DisplayText(3,0,"S(DR):  %");
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