//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"
#include "default.h"
#include "Util.h"

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

void DisplayTitle(void);
void _ADC_Init(void);
void _GPIO_Init(void);

uint16_t KEY_Scan(void);
void TIMER1_Init(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

UINT8 no;

uint16_t VSENSE,ADC1_Value, ADC2_Value, Voltage, Ext_Temperature, Pre_Ext_Temperature=0, Int_Temperature;
uint8_t  str1[20], str2[20];;

void ADC_IRQHandler(void)
{
  ////////////////////////////////////////////////////////////////////////ADC1영역///////////////////////////////////////////
   if (ADC1->SR && ADC_SR_EOC == ADC_SR_EOC) // ADC1 EOC int
	{ 
          GPIOG->ODR ^= 0xFF;
          ADC1->SR &= ~(1<<1);	// EOC flag clear
          ADC1_Value= ADC1->DR;		// Reading ADC result
          VSENSE = (ADC1_Value * (3.3 * 100) / 4095);   // 3.3 : 4095 =  Volatge : ADC_Value 
         GPIOG->ODR ^= 0xFF; 
           sprintf(str1,"%4d",ADC1_Value);	// ADC result : 12bits (0~4095)
          
          LCD_DisplayText(7,0,str1);
          Int_Temperature=((VSENSE-0.76)/2.5)+25;
          
          LCD_DisplayChar(6,8,Int_Temperature/10 + 0x30);
          LCD_DisplayChar(6,9,Int_Temperature%10 + 0x30);
          ADC1->CR2|= (1<<30);            //소프트웨어로 스타트
        }
   
  ////////////////////////////////////////////////////////////////////////ADC2 영역///////////////////////////////////////////
    if (ADC2->SR && ADC_SR_EOC == ADC_SR_EOC) // ADC2 EOC int
	{
 
	ADC2->SR &= ~(1<<1);		// EOC flag clear
	ADC2_Value = ADC2->DR;		// Reading ADC result
      Voltage = (int)(ADC2_Value * (3.3 * 100) / 4095);   // 3.3 : 4095 =  Volatge : ADC_Value 
                                                    // 100:  소수점아래 두자리까지 표시하기 위한 값  
       Ext_Temperature = 3.5*(Voltage*Voltage)/10000+1;
       
       LCD_DisplayChar(2,13,Voltage/100 + 0x30);
	LCD_DisplayChar(2,15,Voltage%100/10 + 0x30);
        LCD_DisplayChar(2,8,Ext_Temperature/10 + 0x30);
        LCD_DisplayChar(2,9,Ext_Temperature%10 + 0x30);
        if(Pre_Ext_Temperature!=Ext_Temperature)
        {
          if(Ext_Temperature<=15)
            {
              LCD_SetBrushColor(RGB_WHITE);
               LCD_DrawFillRect(10, 40, 139, 10);
                LCD_SetBrushColor(RGB_BLUE);
                LCD_DrawFillRect(10, 40,Ext_Temperature*3.56, 10);
                  
            }
            else if(Ext_Temperature<=27)
            {
              LCD_SetBrushColor(RGB_WHITE);
               LCD_DrawFillRect(10, 40, 139, 10);
                LCD_SetBrushColor(RGB_GREEN);
                LCD_DrawFillRect(10, 40, Ext_Temperature*3.56, 10);
            }
            else
            {
              LCD_SetBrushColor(RGB_WHITE);
               LCD_DrawFillRect(10, 40, 139, 10);
                LCD_SetBrushColor(RGB_RED);
                LCD_DrawFillRect(10, 40, Ext_Temperature*3.56, 10);
            }
        }
        Pre_Ext_Temperature=Ext_Temperature;

	// NO SWSTART !!!
        }
}
int main(void)
{
	LCD_Init();	// LCD 구동 함수
	DelayMS(1000);	// LCD구동 딜레이
	_GPIO_Init();
	GPIOG->ODR &= 0x00;	// LED0~7 Off 
	DisplayTitle();	//LCD 초기화면구동 함수
	//BEEP();
 
    	_ADC_Init();
        TIMER1_Init();
	while(1)
	{
 	}
}
void _ADC_Init(void)
{         
        
        ADC->CCR |= (6<<0);		// MULTI[4:0]: ADC모드: 레귤러 동시모드
	ADC->CCR |=  (1<<16); 		// ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
        ADC->CCR |=  (1<<23); 		//온도 센서 켜기
        
        
       RCC->APB2ENR |= (1<<8);	// (1<<8) ENABLE ADC1 CLK (stm32f4xx.h 참조)
       RCC->APB2ENR |= (1<<9);	// (1<<8) ENABLE ADC2 CLK (stm32f4xx.h 참조)

        //ADC1:  내부 온도 센서-> 내부 온도 센서(ADC1_IN16) 사용, 소프트웨어로 ADC 작동
	ADC1->CR1 &= ~(3<<24);		// RES[1:0]=0b00 : 12bit Resolution
	ADC1->CR1 &= ~(1<<8);		// SCAN=0 : ADC_ScanCovMode Disable
	ADC1->CR1 |=  (1<<5);		// EOCIE=1: Interrupt enable for EOC 
        
        //단일모드로 수정
	ADC1->CR2 &= ~(1<<1);		// CONT=0: ADC_Continuous ConvMode Disable
        
        //트리거 해제으로 바꿈
	ADC1->CR2 &= ~(3<<28);		// EXTEN[1:0]=0b00: ADC_ExternalTrigConvEdge
	ADC1->CR2 &= ~(1<<11);		// ALIGN=0: ADC_DataAlign_Right
        
        // 각 시퀀스가 끝나면 eoc 인터럽트 발생
	ADC1->CR2 &= ~(1<<10);		// EOCS=0: The EOC bit is set at the end of each sequence of regular conversions

	ADC1->SQR1 &= ~(0xF<<20);	// L[3:0]: ADC Regular channel sequece length_1conversion
        //ch16으로 수정
        ADC1->SQR3 |= (1<<4);	// SQ1[4:0] : CH16
        
        ADC1->SMPR1 |= (7<<18);	// ADC1_CH16 Sample Time_480Cycles (3*Channel_16)
 	//Channel selection, The Conversion Sequence of PIN1(ADC1_CH16) is first, Config sequence Range is possible from 0 to 17
	
        
        
        // ADC2: 가변저항 조절-> 온도변화표시, TIM1_CH1 250ms 외부 인터럽트시 ADC 작동
	RCC->AHB1ENR |= (1<<0);	// (1<<0) ENABLE GPIOA CLK (stm32f4xx.h 참조)
	GPIOA->MODER |= (3<<2*1);		// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE

	ADC2->CR1 &= ~(3<<24);		// RES[1:0]=      : 10bit Resolution
	ADC2->CR1 &= ~(1<<8);		// SCAN=0 : ADC_ScanCovMode Disable
	ADC2->CR1 |=  (1<<5);		// EOCIE=1: Interrupt enable for EOC

	ADC2->CR2 &= ~(1<<1);		// CONT=0: ADC_Continuous ConvMode Disable
	ADC2->CR2 |=  (2<<28);		// EXTEN[1:0]: ADC_ExternalTrigConvEdge_Enable(Falling Edge)
	ADC2->CR2 &= ~(15<<24);	        // EXSEL[[3:0](27~24): ADC_External event select for regular group_Timer 1 CC1 event
	ADC2->CR2 &= ~(1<<11);		// ALIGN=0: ADC_DataAlign_Right
	ADC2->CR2 &= ~(1<<10);		// EOCS=0: The EOC bit is set at the end of each sequence of regular conversions

	ADC2->SQR1 &= ~(15<<20);	// L[3:0]: ADC Regular channel sequece length 
					//conversion)
 	//Channel selection, The Conversion Sequence of PIN1(ADC2_CH1) is first, Config sequence Range is possible from 0 to 17
	ADC2->SQR3 |= (1<<0);		// SQ1[4:0]=0b0001 : CH1
        ADC2->SMPR2 |= (7<<(3*1));	// ADC2_CH1 Sample TIme_480Cycles (3*Channel_1)
 	//Channel selection, The Conversion Sequence of PIN1(ADC1_CH1) is first, Config sequence Range is possible from 0 to 17
        NVIC->ISER[0] |= (1<<18);   // Enable ADC global Interrupt

        ADC2->CR2 |= (1<<0);      // ADON: ADC ON
        ADC1->CR2 |= (1<<0);      // ADON: ADC ON
        ADC1->CR2|= (1<<30);    //ADC1 소프트웨어로 시작
}

void TIMER1_Init(void)
{

// TIM1_CH1 (PA8) : 125ms 인터럽트 발생
// Clock Enable : GPIOA & TIMER1
	RCC->AHB1ENR	|= (1<<0);	// GPIOA Enable   
	RCC->APB2ENR 	|= (1<<0);	// TIMER1 Enable   
    						
// PE9을 출력설정하고 Alternate function(TIM1_CH1)으로 사용 선언 
	GPIOA->MODER 	|= (2<<2*8);	// PA8 Output Alternate function mode					 
	GPIOA->OSPEEDR 	|= (3<<2*8);	// PA8 Output speed (100MHz High speed)
	GPIOA->OTYPER	&= ~(1<<8);	// PA8 Output type push-pull (reset state)
	GPIOA->AFR[1]	|= (1 <<0); 	// (AFR[1].(0~3)): Connect TIM1 pins(PA8) to AF2(TIM1..2)


	// Assign 'Interrupt Period' and 'Output Pulse Period'
	TIM1->PSC = 16800-1;	// Prescaler 168MHz/16800 = 10KHz (0.1ms)
	TIM1->ARR = 1250-1;	// Auto reload  : 0.1ms * 1250= 125ms(period)

	// CR1 : Up counting
	TIM1->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM1->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
				//	- Counter Overflow/Underflow, 
				// 	- Setting the UG bit Set,
				//	- Update Generation through the slave mode controller 
	TIM1->CR1 &= ~(1<<2);	// URS=0(Update event source Selection): one of following events
				//	- Counter Overflow/Underflow, 
				// 	- Setting the UG bit Set,
				//	- Update Generation through the slave mode controller 
	TIM1->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM1->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM1->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM1->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
				// Center-aligned mode: The counter counts Up and DOWN alternatively

	// Event & Interrup Enable : UI  
	TIM1->EGR |= (1<<0);    // UG: Update generation               

	////////////////////////////////
	// Disable Tim1 Update interrupt
    
	// Define the corresponding pin by 'Output'  
	TIM1->CCER |= (1<<0);	// CC1E=1: CC1  channel Output Enable           
				// OC1(TIM1_CH1) Active: 해당핀을 통해 신호출력
	TIM1->CCER &= ~(1<<1);	// CC3P=0: CC3 channel Output Polarity (OCPolarity_High : OC1으로 반전없이 출력)  

	// 'Mode' Selection : Output mode, toggle  
	TIM1->CCMR1 &= ~(3<<0); // CC1S(CC1 channel)  : Output  
	TIM1->CCMR1 &= ~(1<<3); // OC1P=0: Output Compare 1 preload disable
	TIM1->CCMR1 |= (3<<4);	// OC1M Output Compare 1 Mode : toggle
				// OC1REF toggles when CNT = CCR1
        
        TIM1->BDTR=(1<<15);                                     
 	TIM1->CCR1 = 800;	// TIM1 CCR1 TIM1_Pulse        
      
	TIM1->CR1 |= (1<<0);	// CEN: Enable the Tim1 Counter  	        				
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
}

void DisplayTitle(void)
{	LCD_Clear(RGB_WHITE);
	LCD_SetFont(&Gulim7);
	LCD_SetBackColor(RGB_WHITE);	//배경색 하얀색
        
        //글자색 검정색
	LCD_SetTextColor(RGB_BLACK);	
        //이름 및 학번
	LCD_DisplayText(0,0,"LCH 2017130036");
        //3.1Mission subtitle
        LCD_DisplayText(1,0,"[External Temperature]");
       //3.2Mission subtitle
        LCD_DisplayText(5,0,"[Internal Temperature]");
        
        
        //글자색 파란색
        LCD_SetTextColor(RGB_BLUE);
        LCD_DisplayText(2,0,"EXT TMP:  C (   V)");  
        LCD_DisplayText(6,0,"INT TMP:  C");
        
        //글자색 빨간색
        LCD_SetTextColor(RGB_RED);	
        LCD_DisplayText(2,14,",");

}
void DelayMS(unsigned short wMS)
{	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);  // 1000us => 1ms
}
void DelayUS(unsigned short wUS)
{	volatile int Dly = (int)wUS*17;
	for(; Dly; Dly--);
}
uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{ 
	uint16_t key;
	key = GPIOH->IDR & 0xFF00;	// any key pressed ?
	if(key == 0xFF00)		// if no key, check key off
	{  	if(key_flag == 0)
        		return key;
      		else
		{	DelayMS(10);
        		key_flag = 0;
        		return key;
        	}
    	}
  	else				// if key input, check continuous key
	{	if(key_flag != 0)	// if continuous key, treat as no key input
        		return 0xFF00;
      		else			// if new key,delay for debounce
		{	key_flag = 1;
			DelayMS(10);
 			return key;
        	}
	}
}
void BEEP(void)			// Beep for 20 ms 
{ 	GPIOF->ODR |= (1<<9);	// PF9 'H' Buzzer on
	DelayMS(20);		// Delay 20 ms
	GPIOF->ODR &= ~(1<<9);	// PF9 'L' Buzzer off
}