//////////////////////////////////////////////////////////////////////
// HW3: USART(BT) 통신을 이용한 센서모니터링 프로그램
// 제출자: 2017130036_이찬해
// 개요:

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

void DisplayOn(void);

void _GPIO_Init(void);
uint16_t KEY_Scan(void);

//타이머 설정
void TIMER2CH4_PWM_Init(void);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

int main(void)
{
  LCD_Init();   // LCD 구동 함수
  DelayMS(1000);   // LCD구동 딜레이
  
  _GPIO_Init();
  GPIOG->ODR &= 0x00;   // LED0~7 Off 
  
  DisplayOn();   //LCD 초기화면구동 함수
  //BEEP();
  
  TIMER2CH4_PWM_Init();
  
  while(1)
  {
    GPIOG->ODR ^= 0x01; 
    
  }
}
void TIM2_IRQHandler(void)      //RESET: 0
{
	if ((TIM2->SR & 0x01) != RESET)	// Update interrupt flag (10ms)
	{
		TIM2->SR &= ~(1<<0);	// Update Interrupt Claer
		GPIOG->ODR ^= 0x01;	// LED0 On
                DelayMS(500);
	}
    
}

void TIMER2CH4_PWM_Init(void)
{  
// 모터펄스(PWM)핀:PB11(TIM2 CH4)
	RCC->AHB1ENR	|= (1<<1);	// GPIOB Enable
	RCC->APB1ENR 	|= (1<<0);	// TIMER2 Enable 
        
        GPIOB->MODER 	|= (2<<22);	// PA0 Output Alternate function mode					
	GPIOB->OSPEEDR 	|= (3<<22);	// PA0 Output speed (100MHz High speed)
	GPIOB->OTYPER	&= ~(1<<11);	// PA0 Output type push-pull (reset state)
	GPIOB->AFR[1]	|= (1<<12); 
// TIM2Channel 4 : PWM 1 mode
	// Assign 'PWM Pulse Period'
	TIM1->PSC	= 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
	TIM1->ARR	= 50000-1;	// Auto reload  (0.1ms *50000 =5S  : PWM Period)
   
	// Define the corresponding pin by 'Output'  
	// CCER(Capture/Compare Enable Register) : Enable "Channel 4" 
	TIM2->CCER	|= (1<<12);	// CC1E=1: OC1(TIM5_CH1) Active(Capture/Compare 1 output enable)
					// 해당핀(40번)을 통해 신호출력
	TIM2->CCER	&= ~(1<<13);	// CC1P=0: CC1 output Polarity High (OC1으로 반전없이 출력)

	// Duty Ratio 
	 TIM2->CCR4	= 5000;		// DR=10%
         
	// 'Mode' Selection : Output mode, PWM 1
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
	TIM2->CCMR2 	&= ~(3<<8); 	// CC4S(CC4 channel): Output 
	TIM2->CCMR2 	|= (1<<11); 	// OC4PE=1: Output Compare4 preload Enable

	TIM2->CCMR2	|= (6<<12);	// OC4M: Output compare 4 mode: PWM 1 mode
	TIM2->CCMR2	|= (1<<15);	// OC4CE: Output compare 4 Clear enable

	// CR1 : Up counting & Counter TIM5 enable
	TIM2->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM2->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
	TIM2->CR1 	&= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
	TIM2->CR1	|= (1<<7);	// ARPE: Auto-reload preload enable
        TIM2->EGR |= (1<<0);    // UG: Update generation   
        TIM2->DIER |= (1<<0);	// UIE: Enable Tim2 Update interrupt
	TIM2->DIER |= (1<<4);	// CC1IE: Enable the Tim2 CC4 interrupt

	NVIC->ISER[0] 	|= (1<<28);	// Enable Timer2 global Interrupt on NVIC

        
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

//외부온도측정모드
void DisplayOn(void)
{   
  LCD_Clear(RGB_YELLOW);                 //배경 노란색으로 초기화
  LCD_SetFont(&Gulim8);
  LCD_SetBackColor(RGB_YELLOW);   //배경색 노란색
  
  //이름 및 학번
  LCD_SetTextColor(RGB_BLUE);   
  LCD_DisplayText(0,0,"LCH 2017130036");

  //제목
  LCD_SetTextColor(RGB_BLUE);   
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