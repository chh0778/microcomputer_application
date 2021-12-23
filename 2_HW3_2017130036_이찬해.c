//////////////////////////////////////////////////////////////////////
// HW3: USART(BT) 통신을 이용한 센서모니터링 프로그램
// 제출자: 2017130036_이찬해
// 개요:
//1. 외부온도(MCU 외부 가변저항) 측정
//가변저항의 변화에 따른 전압을 감지하여 온도로 변환한 값을 LCD화면에 출력
//전압변화는 ADC2모듈사용
//ADC 트리거는 TIM1_CH1 oc_mode 사용(주기: 250ms)
//온도 1~15: 파란색, 16~27: 녹색, 28~39: 빨간색
//2. 내부 온도(MCU 내부 온도 센서) 측정
//MCU 내부 온도센서에서 감지한 온도를 LCD화면에 출력
//ADC1 모듈 사용
//온도 변화에 따른 전압변화
//3. 외부온도와 내부온도 표시 화면 변환(스마트폰과 블루투스통신)
//uart4를 이용한 블루투스 통신
//1. 휴대폰에서 0을 입력하면
/////////////////////////////////////////// ////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

//내부온도측정모드와 외부온도측정모드를 표현하는 FLAG
int mode_flag=0;
void DisplayExtTemp(void);
void DisplayIntTemp(void);

void _GPIO_Init(void);
uint16_t KEY_Scan(void);

//uart4 사용
void UART4_Init(void);
void USART_BRR_Configuration(uint32_t USART_BaudRate);

//블루투스에 연결된 장치에 문자열 혹은 문자를 보내는 함수
void SerialSendChar(uint8_t c);
void SerialSendString(char *s);

void _ADC_Init(void);

//TIM1_CH1사용
void TIMER1_Init(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

UINT8 no, CRdata;

//외부온도측정모드에서 사용하는 변수
uint16_t  ADC2_Value, Voltage, Ext_Temperature, Pre_Ext_Temperature=0;
uint8_t  str1[20], str2[20], Ext_temp[2];


//내부온도측정모드에서 사용하는 변수
uint16_t  ADC1_Value, Int_Temperature;
double VSENSE;
uint8_t   str2[20];


void ADC_IRQHandler(void)
{
  
  //내부온도측정 (ADC1)영역
  if (ADC1->SR && ADC_SR_EOC == ADC_SR_EOC) // ADC1 EOC int
  { 
    if(mode_flag==1)
    {
      ADC1->SR &= ~(1<<1);   // EOC flag clear
      ADC1_Value= ADC1->DR;      // Reading ADC result
      VSENSE = (ADC1_Value * 3.3  / 4095);   // 3.3 : 4095 =  VSENSE : ADC1_Value
      
      Int_Temperature=((VSENSE-0.76)*1000)/2.5+25;//전압-->온도 변환공식
      
      //온도값 LCD 출력
      LCD_DisplayChar(2,8,Int_Temperature/10 + 0x30);
      LCD_DisplayChar(2,9,Int_Temperature%10 + 0x30);
      ADC1->CR2|= (1<<30);            //소프트웨어로 스타트
    }
  }
  
  //외부온도측정 (ADC2)측정
  if (ADC2->SR && ADC_SR_EOC == ADC_SR_EOC) // ADC2 EOC int
  {
    if(mode_flag==0)
    {
      ADC2->SR &= ~(1<<1);      // EOC flag clear
      ADC2_Value = ADC2->DR;      // Reading ADC result
      
      
      Voltage = (int)(ADC2_Value * (3.3 * 100) / 4095);   // 3.3 : 4095 =  Volatge : ADC2_Value 
      // 100:  int형 변수에 넣기 위해 곱해주는 값
      Ext_Temperature = 3.5*(Voltage*Voltage)/10000+1;
      
      //정수형태의 온도값을 문자열 형태로 변환
      sprintf(Ext_temp,"%2d",Ext_Temperature);
      
      //전압값과 온도값 LCD 출력
      LCD_DisplayChar(2,13,Voltage/100 + 0x30);
      LCD_DisplayChar(2,15,Voltage%100/10 + 0x30);
      LCD_DisplayChar(2,8,Ext_Temperature/10 + 0x30);
      LCD_DisplayChar(2,9,Ext_Temperature%10 + 0x30);
      
      //온도 바 색 변경
      if(Pre_Ext_Temperature!=Ext_Temperature)                        //온도값이 바뀔때에만 LCD 온도바 변화 & 휴대폰으로 온도 정도
      {
        //MCU--->휴대폰
        //온도 전송
        SerialSendString(Ext_temp);
        if(Ext_Temperature<=15)                                                  //15도 이하일 때
        {
          LCD_SetBrushColor(RGB_WHITE);                            //이전 온도바 지우기
          LCD_DrawFillRect(10, 40, 139, 10);
          LCD_SetBrushColor(RGB_BLUE);                             //파란색 온도바 표현
          LCD_DrawFillRect(10, 40,Ext_Temperature*3.56, 10);
          
        }
        else if(Ext_Temperature<=27)                                         //15이상, 27도 이하일 때
        {
          LCD_SetBrushColor(RGB_WHITE);                           //이전 온도바 지우기
          LCD_DrawFillRect(10, 40, 139, 10);
          LCD_SetBrushColor(RGB_GREEN);                           //초록색 온도바 표현
          LCD_DrawFillRect(10, 40, Ext_Temperature*3.56, 10);
        }
        else                                                                               //28 도 이상일 때
        {
          LCD_SetBrushColor(RGB_WHITE);                           //이전 온도바 지우기            
          LCD_DrawFillRect(10, 40, 139, 10);
          LCD_SetBrushColor(RGB_RED);                             //빨간색 온도바 표현
          LCD_DrawFillRect(10, 40, Ext_Temperature*3.56, 10);
        }
      }
      Pre_Ext_Temperature=Ext_Temperature;
      
      // NO SWSTART !!!
    }
  }
}

int main(void)
{
  LCD_Init();   // LCD 구동 함수
  DelayMS(1000);   // LCD구동 딜레이
  
  _GPIO_Init();
  UART4_Init();         //UART4 설정 함수
  GPIOG->ODR &= 0x00;   // LED0~7 Off 
  
  DisplayExtTemp();   //LCD 초기화면구동 함수
  //BEEP();
  
  _ADC_Init();          //ADC 설정 함수
  TIMER1_Init();        //TIM 설정 함수
  
  while(1)
  {
    GPIOG->ODR ^= 0x01; 
    switch(KEY_Scan())
    {
    case SW0_PUSH :    //SW0   
      SerialSendString("AT");                                                   //AT: 블루투스 기본 설정
      SerialSendChar(0x0D); 
      LCD_DisplayText(8,0,"                                    "); 
      LCD_DisplayText(8,0,"AT"); 
      LCD_DisplayText(9,0,"                               ");
      no= 0;
      break;
      
    case SW1_PUSH :    //SW1
      SerialSendString("AT+BTSCAN");                                            //AT+BTSCAN: 블루투스가 검색되도록 설정
      SerialSendChar(0x0D); 
      LCD_DisplayText(8,0,"                                    "); 
      LCD_DisplayText(8,0,"AT+BTSCAN"); 
      LCD_DisplayText(9,0,"                               ");
      no= 0;
      break;
      
    case SW2_PUSH :    //SW2
      SerialSendString("AT+BTCANCEL");                                          //AT+BTCANCEL: 다른 기기가 키트를 발견할 수 있도록 설정
      SerialSendChar(0x0D); 
      LCD_DisplayText(8,0,"                                    "); 
      LCD_DisplayText(8,0,"AT+BTCANCEL"); 
      LCD_DisplayText(9,0,"                               ");
      no= 0;
      break;
      
    case SW3_PUSH :    //SW3
      SerialSendString("AT+BTNAME=WED08");                                     //AT+BTNAME=WED08": 이름 설정
      SerialSendChar(0x0D); 
      LCD_DisplayText(8,0,"                                    "); 
      LCD_DisplayText(8,0,"AT+BTNAME=WED08"); 
      LCD_DisplayText(9,0,"                               ");
      no= 0;
      break;
      
    case SW4_PUSH :    //SW4
      SerialSendString("ATZ");                                                             //ATZ:
      SerialSendChar(0x0D); 
      LCD_DisplayText(8,0,"                                    "); 
      LCD_DisplayText(8,0,"ATZ"); 
      LCD_DisplayText(9,0,"                               ");
      no= 0;
      break;
      
    case SW5_PUSH :    //SW5
      SerialSendString("AT+BTNAME?");                                                           //AT+BITANAME
      SerialSendChar(0x0D); 
      LCD_DisplayText(8,0,"                                    "); 
      LCD_DisplayText(8,0,"AT+BTNAME?");                                                //
      LCD_DisplayText(9,0,"                               ");
      no= 0;
      break;
    }
  }
}

void UART4_IRQHandler(void)   
{       
  if ( (UART4->SR & USART_SR_RXNE) ) // USART_SR_RXNE= 1? RX Buffer Full?
    // #define  USART_SR_RXNE ((uint16_t)0x0020)    //  Read Data Register Not Empty     
  {
    char ch=0+0x30;
    ch = UART4->DR;   // 수신된 문자 저장
    //(휴대폰-->MCU)
    //휴대폰에서 0입력-> 외부온도 측정모드
    if(ch==0+0x30)
    {
      DisplayExtTemp();
      ADC1->CR2 &= ~(1 << 0);
      ADC2->CR2 |= (1 << 0);
      mode_flag=0;
    }
    
    //휴대폰에서 1입력-> 내부온도 측정모드
    else if (ch==1+0x30)
    {
      DisplayIntTemp();
      ADC1->CR2 |= (1 << 0);
      ADC2->CR2 &= ~(1 << 0);
      mode_flag=1;
    }
  } 
  // DR 을 읽으면 SR.RXNE bit(flag bit)는 clear 된다. 즉 clear 할 필요없음 
}

void _ADC_Init(void)
{         
  RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;           // (1<<9) ENABLE ADC2 CLK (stm32f4xx.h 참조)
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;   // (1<<8) ENABLE ADC1 CLK (stm32f4xx.h 참조)
  
  ADC->CCR &= ~(0x1F<<0);      // MULTI[4:0]: ADC모드: 독립모드
  ADC->CCR |=  (1<<16);       // ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
  ADC->CCR |=  (1<<23);       //온도 센서 켜기
  
  //ADC1:  내부 온도 센서-> 내부 온도 센서(ADC1_IN16) 사용, 소프트웨어로 ADC 작동
  ADC1->CR1 &= ~(3<<24);      // RES[1:0]=0b00 : 12bit Resolution
  ADC1->CR1 &= ~(1<<8);      // SCAN=0 : ADC_ScanCovMode Disable
  ADC1->CR1 |=  (1<<5);      // EOCIE=1: Interrupt enable for EOC 
  
  //단일모드로 수정
  ADC1->CR2 &= ~(1<<1);      // CONT=0: ADC_Continuous ConvMode Disable
  
  //상승 트리거로 바꿈
  ADC1->CR2 |= (2<<28);      // EXTEN[1:0]=0b00: ADC_ExternalTrigConvEdge_r(Falling Edge)
  ADC1->CR2 &= ~(1<<11);      // ALIGN=0: ADC_DataAlign_Right
  
  // 각 시퀀스가 끝나면 eoc 인터럽트 발생
  ADC1->CR2 &= ~(1<<10);      // EOCS=0: The EOC bit is set at the end of each sequence of regular conversions
  
  ADC1->SQR1 &= ~(0xF<<20);   // L[3:0]: ADC Regular channel sequece length_1conversion
  //ch16으로 수정
  ADC1->SQR3 |= (0x10<<0);   // SQ1[4:0] : CH16
  
  ADC1->SMPR1 |= (7<<18);   // ADC1_CH16 Sample Time_480Cycles (3*Channel_16)
  //Channel selection, The Conversion Sequence of (ADC1_CH16) is first, Config sequence Range is possible from 0 to 17
  
  
  
  // ADC2: 가변저항 조절-> 온도변화표시, TIM1_CH1 250ms 외부 인터럽트시 ADC 작동
  RCC->AHB1ENR |= (1<<0);   // (1<<0) ENABLE GPIOA CLK (stm32f4xx.h 참조)
  GPIOA->MODER |= (3<<2*1);      // CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
  
  ADC2->CR1 &= ~(3<<24);      // RES[1:0]=      : 12bit Resolution
  ADC2->CR1 &= ~(1<<8);      // SCAN=0 : ADC_ScanCovMode Disable
  ADC2->CR1 |=  (1<<5);      // EOCIE=1: Interrupt enable for EOC
  
  ADC2->CR2 &= ~(1<<1);      // CONT=0: ADC_Continuous ConvMode Disable
  ADC2->CR2 |=  (2<<28);      // EXTEN[1:0]: ADC_ExternalTrigConvEdge_Enable(Falling Edge)
  ADC2->CR2 &= ~(15<<24);           // EXSEL[[3:0](27~24): ADC_External event select for regular group_Timer 1 CC1 event
  ADC2->CR2 &= ~(1<<11);      // ALIGN=0: ADC_DataAlign_Right
  ADC2->CR2 &= ~(1<<10);      // EOCS=0: The EOC bit is set at the end of each sequence of regular conversions
  
  ADC2->SQR1 &= ~(15<<20);   // L[3:0]: ADC Regular channel sequece length 
  //conversion)
  //Channel selection, The Conversion Sequence of PIN1(ADC2_CH1) is first, Config sequence Range is possible from 0 to 17
  ADC2->SQR3 |= (1<<0);      // SQ1[4:0]=0b0001 : CH1
  ADC2->SMPR2 |= (7<<(3*1));   // ADC2_CH1 Sample TIme_480Cycles (3*Channel_1)
  //Channel selection, The Conversion Sequence of PIN1(ADC1_CH1) is first, Config sequence Range is possible from 0 to 17
  NVIC->ISER[0] |= (1<<18);   // Enable ADC global Interrupt
  
  ADC2->CR2 |= (1<<0);      // ADON: ADC2 ON
  ADC1->CR2 &= ~(1<<0);      // ADON: ADC1 OFF(on으로 할 경우 스위치 동작이 안됩니다.)
  ADC1->CR2|= (1<<30);    //ADC1 소프트웨어로 시작
}

void TIMER1_Init(void)
{
  // TIM1_CH1 (PA8) : 125ms 인터럽트 발생
  // Clock Enable : GPIOA & TIMER1
  RCC->AHB1ENR   |= (1<<0);   // GPIOA Enable   
  RCC->APB2ENR    |= (1<<0);   // TIMER1 Enable   
  
  // PE9을 출력설정하고 Alternate function(TIM1_CH1)으로 사용 선언 
  GPIOA->MODER    |= (2<<2*8);   // PA8 Output Alternate function mode                
  GPIOA->OSPEEDR    |= (3<<2*8);   // PA8 Output speed (100MHz High speed)
  GPIOA->OTYPER   &= ~(1<<8);   // PA8 Output type push-pull (reset state)
  GPIOA->AFR[1]   |= (1 <<0);    // (AFR[1].(0~3)): Connect TIM1 pins(PA8) to AF2(TIM1..2)
  
  // Assign 'Interrupt Period' and 'Output Pulse Period'
  TIM1->PSC = 16800-1;   // Prescaler 168MHz/16800 = 10KHz (0.1ms)
  TIM1->ARR = 1250-1;   // Auto reload  : 0.1ms * 1250= 125ms(period)
  
  // CR1 : Up counting
  TIM1->CR1 &= ~(1<<4);   // DIR=0(Up counter)(reset state)
  TIM1->CR1 &= ~(1<<1);   // UDIS=0(Update event Enabled): By one of following events
  //   - Counter Overflow/Underflow, 
  //    - Setting the UG bit Set,
  //   - Update Generation through the slave mode controller 
  TIM1->CR1 &= ~(1<<2);   // URS=0(Update event source Selection): one of following events
  //   - Counter Overflow/Underflow, 
  //    - Setting the UG bit Set,
  //   - Update Generation through the slave mode controller 
  TIM1->CR1 &= ~(1<<3);   // OPM=0(The counter is NOT stopped at update event) (reset state)
  TIM1->CR1 &= ~(1<<7);   // ARPE=0(ARR is NOT buffered) (reset state)
  TIM1->CR1 &= ~(3<<8);    // CKD(Clock division)=00(reset state)
  TIM1->CR1 &= ~(3<<5);    // CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
  // Center-aligned mode: The counter counts Up and DOWN alternatively
  
  // Event & Interrup Enable : UI  
  TIM1->EGR |= (1<<0);    // UG: Update generation               
  
  // Define the corresponding pin by 'Output'  
  TIM1->CCER |= (1<<0);   // CC1E=1: CC1  channel Output Enable           
  // OC1(TIM1_CH1) Active: 해당핀을 통해 신호출력
  TIM1->CCER &= ~(1<<1);   // CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1으로 반전없이 출력)  
  
  // 'Mode' Selection : Output mode, toggle  
  TIM1->CCMR1 &= ~(3<<0); // CC1S(CC1 channel)  : Output  
  TIM1->CCMR1 &= ~(1<<3); // OC1P=0: Output Compare 1 preload disable
  TIM1->CCMR1 |= (3<<4);   // OC1M Output Compare 1 Mode : toggle
  // OC1REF toggles when CNT = CCR1
  
  TIM1->BDTR=(1<<15);
  TIM1->CCR1 = 800;   // TIM1 CCR1 TIM1_Pulse        
  
  TIM1->CR1 |= (1<<0);   // CEN: Enable the Tim1 Counter 
}

void UART4_Init(void)
{
  // UART4 : TX(PC10)
  RCC->AHB1ENR   |= (1<<2);   // RCC_AHB1ENR GPIOC Enable
  GPIOC->MODER   |= (2<<2*10);   // GPIOC PIN10 Output Alternate function mode               
  GPIOC->OSPEEDR   |= (3<<2*10);   // GPIOC PIN10 Output speed (100MHz Very High speed)
  GPIOC->AFR[1]   |= (8<<4*(10-8));// Connect GPIOC pin10 to AF8(USART1)
  
  // UART4 : RX(PC11)
  GPIOC->MODER    |= (2<<2*11);   // GPIOC PIN11 Output Alternate function mode
  GPIOC->OSPEEDR   |= (3<<2*11);   // GPIOC PIN11 Output speed (100MHz Very High speed
  GPIOC->AFR[1]   |= (8<<4*(11-8));// Connect GPIOC pin11 to AF8(USART1)
  
  // BT RESET (PC13) : GPIO
  GPIOC->MODER    |= (1<<2*13);   // GPIOC PIN13 Output mode
  GPIOC->OSPEEDR  |= (3<<2*13);
  GPIOC->ODR   |= (1<<13);   // BT Reset
  
  RCC->APB1ENR   |= (1<<19);   // RCC_APB1ENR UART4 Enable
  
  USART_BRR_Configuration(9600); // USART Baud rate Configuration
  
  UART4->CR1   &= ~(1<<12);   // USART_WordLength 8 Data bit
  UART4->CR1   &= ~(1<<10);   // NO USART_Parity
  
  UART4->CR1   |= (1<<2);   // 0x0004, USART_Mode_RX Enable
  UART4->CR1   |= (1<<3);   // 0x0008, USART_Mode_Tx Enable
  UART4->CR2   &= ~(3<<12);   // 0b00, USART_StopBits_1
  UART4->CR3   = 0x0000;   // No HardwareFlowControl, No DMA
  
  UART4->CR1    |= (1<<5);   // 0x0020, RXNE interrupt Enable
  NVIC->ISER[1]   |= (1<<(52-32));// Enable Interrupt USART1 (NVIC 52번)
  UART4->CR1    |= (1<<13);   //  0x2000, USART1 Enable
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

void SerialSendChar(uint8_t Ch) // 1문자 보내기 함수
{
  // USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty? 
  // TX buffer Empty되지 않으면 계속 대기(송신 가능한 상태까지 대기)
  while((UART4->SR & USART_SR_TXE) == RESET); 
  UART4->DR = (Ch & 0x01FF);   // 전송 (최대 9bit 이므로 0x01FF과 masking)
}

void SerialSendString(char *str) // 여러문자 보내기 함수
{
  while (*str != '\0') // 종결문자가 나오기 전까지 구동, 종결문자가 나온후에도 구동시 메모리 오류 발생가능성 있음.
  {
    SerialSendChar(*str);   // 포인터가 가르키는 곳의 데이터를 송신
    str++;          // 포인터 수치 증가
  }
}

// Baud rate 설정
void USART_BRR_Configuration(uint32_t USART_BaudRate)
{ 
  uint32_t tmpreg = 0x00;
  uint32_t APB1clock = 42000000;   //PCLK2_Frequency
  uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
  
  // Find the integer part 
  if ((UART4->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
    //  #define  USART_CR1_OVER8 ((uint16_t)0x8000) // USART Oversampling by 8 enable   
  {       // UART4->CR1.OVER8 = 1 (8 oversampling)
    // Computing 'Integer part' when the oversampling mode is 8 Samples 
    integerdivider = ((25 * APB1clock) / (2 * USART_BaudRate));  // 공식에 100을 곱한 곳임(소수점 두번째자리까지 유지하기 위함)  
  }
  else  // USART1->CR1.OVER8 = 0 (16 oversampling)
  {   // Computing 'Integer part' when the oversampling mode is 16 Samples 
    integerdivider = ((25 * APB1clock) / (4 * USART_BaudRate));  // 공식에 100을 곱한 곳임(소수점 두번째자리까지 유지하기 위함)    
  }
  tmpreg = (integerdivider / 100) << 4;
  
  // Find the fractional part 
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
  
  // Implement the fractional part in the register 
  if ((UART4->CR1 & USART_CR1_OVER8) != 0)   
  {   // 8 oversampling
    tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
  }
  else   // 16 oversampling
  {
    tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
  }
  
  // Write to USART BRR register
  UART4->BRR = (uint16_t)tmpreg;
}

//외부온도측정모드
void DisplayExtTemp(void)
{   
  LCD_Clear(RGB_WHITE);                 //배경 하얀색으로 초기화
  LCD_SetFont(&Gulim7);
  LCD_SetBackColor(RGB_WHITE);   //배경색 하얀색
  
  //글자색 검정색
  LCD_SetTextColor(RGB_BLACK);   
  //이름 및 학번
  LCD_DisplayText(0,0,"LCH 2017130036");
  //3.1Mission subtitle
  LCD_DisplayText(1,0,"[External Temperature]");
  
  //글자색 파란색
  LCD_SetTextColor(RGB_BLUE);
  LCD_DisplayText(2,0,"EXT TMP:  C (   V)");  
  
  //글자색 빨간색
  LCD_SetTextColor(RGB_RED);   
  LCD_DisplayText(2,14,",");
}

//내부온도측정모드
void DisplayIntTemp(void)
{   
  LCD_Clear(RGB_YELLOW);                //배경 노란색으로 초기화
  LCD_SetFont(&Gulim7);
  LCD_SetBackColor(RGB_YELLOW);   //배경색 노란색
  
  //글자색 검정색
  LCD_SetTextColor(RGB_BLACK);   
  //이름 및 학번
  LCD_DisplayText(0,0,"LCH 2017130036");
  //3.2Mission subtitle
  LCD_DisplayText(1,0,"[Internal Temperature]");
  
  //글자색 파란색
  LCD_SetTextColor(RGB_BLUE);
  LCD_DisplayText(2,0,"INT TMP:  C");
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