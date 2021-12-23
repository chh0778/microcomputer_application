/////////////////////////////////////////////////////////////
// PWM Mode
// Timer4 CH3 : PB8(167번 pin)
// OC3을 통한 PWM Pulse(DC motor 구동) 출력 (PWM 1 mode)
/////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"

// SW0~7 Define
#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

// Joystick Define
// NO Joy-Button : 0x03E0 : 0000 0011 1110 0000 
			//Bit No      FEDC BA98 7654 3210
#define NAVI_PUSH	0x03C0  //PI5 0000 0011 1100 0000 
#define NAVI_UP		0x03A0  //PI6 0000 0011 1010 0000 
#define NAVI_DOWN	0x0360  //PI7 0000 0011 0110 0000 
#define NAVI_RIGHT	0x02E0  //PI8 0000 0010 1110 0000 
#define NAVI_LEFT	0x01E0  //PI9 0000 0001 1110 0000 

void _GPIO_Init(void);
uint16_t KEY_Scan(void);	// Switch 입력 함수
uint16_t JOY_Scan(void);	// Joystick 입력 함수
void DisplayInitScreen(void);

void TIMER4_PWM_Init(void);	// Timer4 PWM mode 초기화 

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

int main(void)
{
	_GPIO_Init();
	LCD_Init();			// LCD 구동 함수
	DelayMS(10);			// LCD 구동 딜레이
	DisplayInitScreen();		// LCD 초기화면구동 함수
	BEEP();

	GPIOG->ODR &= ~0x00FF;		// 초기값: LED0~7 Off
	GPIOG->ODR |= (1<<0); 		// LED0 ON

	GPIOB->ODR |= (1<<9);		// 초기 모터 방향(PB9):CW
	TIMER4_PWM_Init();		// TIM4 Init (PWM mode)
    
	while(1)
	{
		switch(KEY_Scan())	// SW0~SW3 입력으로 Duty 변경
		{
			case SW0_PUSH : 	//SW0
				TIM4->CCR3 = 10;		// DR: 10%
				LCD_DisplayText(1,6,"10");
				GPIOG->ODR &= ~0x00FF;		// All LED Off
				GPIOG->ODR |= (1<<0);		// LED0 ON 
 			break;

			case SW1_PUSH : 	//SW1
				TIM4->CCR3 = 35;		// DR: 35%
				LCD_DisplayText(1,6,"35");
				GPIOG->ODR &= ~0x00FF;		// All LED Off
				GPIOG->ODR |= (1<<1);		// LED1 ON 
			break;

			case SW2_PUSH  : 	//SW2
				TIM4->CCR3 = 70;		// DR: 70%
				LCD_DisplayText(1,6,"70");
				GPIOG->ODR &= ~0x00FF;		// All LED Off
				GPIOG->ODR |= (1<<2);		// LED2 ON 
			break;

			case SW3_PUSH  : 	//SW3
				TIM4->CCR3 = 90;		// DR: 90%	  
				LCD_DisplayText(1,6,"90");
				GPIOG->ODR &= ~0x00FF;		// All LED Off
				GPIOG->ODR |= (1<<3);		// LED3 ON 
			break;

        }    // switch(KEY_Scan())  
		
        switch(JOY_Scan())	// Joystick 입력으로 모터 DIR 변경
        {
                case NAVI_LEFT : 	// Joystick LEFT
                    GPIOB->ODR |= (1<<9);	//DIR = CW(HIGH) (PB_9)
                    LCD_DisplayText(2,6," CW");	 	   
                break;
                case NAVI_RIGHT:	// Joystick RIGHT	
                    GPIOB->ODR &= ~(1<<9);	//DIR = CCW(LOW)(PB_9)
                    LCD_DisplayText(2,6,"CCW");
                break;
        }  // switch(JOY_Scan())
    }
}

void TIMER4_PWM_Init(void)
{   
// TIM CH3 : PB8 (167번 핀)
// Clock Enable : GPIOB & TIMER4
	RCC->AHB1ENR	|= (1<<1);	// GPIOB CLOCK Enable
	RCC->APB1ENR 	|= (1<<2);	// TIMER4 CLOCK Enable 
    						
// PB8을 출력설정하고 Alternate function(TIM4_CH3)으로 사용 선언 : PWM 출력
	GPIOB->MODER 	|= (2<<16);	// 0x00020000 PB8 Output Alternate function mode					
	GPIOB->OSPEEDR 	|= (3<<16);	// 0x00030000 PB8 Output speed (100MHz High speed)
	GPIOB->OTYPER	&= ~(1<<8);	// PB8 Output type push-pull (reset state)
	GPIOB->PUPDR	|= (1<<16);	// 0x00010000 PB8 Pull-up
 	GPIOB->AFR[1]	|= (2<<0);	// 0x00000002 (AFR[1].(3~0)=0b0010): Connect TIM4 pins(PB8) to AF2(TIM3..5)
    
// TIM4 Channel 3 : PWM 1 mode
	// Assign 'PWM Pulse Period'
	TIM4->PSC	= 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
	TIM4->ARR	= 100-1;	// Auto reload  (0.1ms * 50= 5ms : PWM Period)

	// Setting CR1 : 0x0000 (Up counting)
	TIM4->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM4->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled)
	TIM4->CR1 &= ~(1<<2);	// URS=0(Update event source Selection)g events
	TIM4->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM4->CR1 |= (1<<7);	// ARPE=1(ARR is buffered): ARR Preload Enable 
	TIM4->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM4->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 : Edge-aligned mode(reset state)
				
	// Define the corresponding pin by 'Output'  
	// CCER(Capture/Compare Enable Register) : Enable "Channel 3" 
	TIM4->CCER	|= (1<<8);	// CC3E=1: OC3(TIM4_CH3) Active(Capture/Compare 3 output enable)
					// 해당핀(167번)을 통해 신호출력
	TIM4->CCER	&= ~(1<<9);	// CC3P=0: CC3 Output Polarity (OCPolarity_High : OC3으로 반전없이 출력)

	// Duty Ratio 
	TIM4->CCR3	= 10;		// CCR3 value

	// 'Mode' Selection : Output mode, PWM 1
	// CCMR2(Capture/Compare Mode Register 2) : Setting the MODE of Ch3 or Ch4
	TIM4->CCMR2 &= ~(3<<0); // CC3S(CC3 channel)= '0b00' : Output 
	TIM4->CCMR2 |= (1<<3); 	// OC3PE=1: Output Compare 3 preload Enable
	TIM4->CCMR2	|= (7<<4);	// OC3M=0b111: Output compare 3 mode: PWM 12mode
	TIM4->CCMR2	|= (1<<7);	// OC3CE=1: Output compare 3 Clear enable
	
	//Counter TIM5 enable
	TIM4->CR1	|= (1<<0);	// CEN: Counter TIM4 enable
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
	
	// DC_MOTOR DIR (PB9)
	RCC->AHB1ENR	|= 0x00000002; 	// RCC_AHB1ENR GPIOB Enable							
	GPIOB->MODER 	|= 0x00040000;	// GPIOB PIN9 Output mode						
	GPIOB->OSPEEDR 	|= 0x00040000;	// GPIOB PIN9 Output speed (25MHZ Medium speed) 
}	

void BEEP(void)			// Beep for 20 ms 
{ 	GPIOF->ODR |= (1<<9);	// PF9 'H' Buzzer on
	DelayMS(20);		// Delay 20 ms
	GPIOF->ODR &= ~(1<<9);	// PF9 'L' Buzzer off
}

void DelayMS(unsigned short wMS)
{
	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);   // 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
	volatile int Dly = (int)wUS*17;
		for(; Dly; Dly--);
}

void DisplayInitScreen(void)
{
	LCD_Clear(RGB_WHITE);		// 화면 클리어
	LCD_SetFont(&Gulim8);		// 폰트 : 굴림 8
	LCD_SetBackColor(RGB_GREEN);	// 글자배경색 : Green
	LCD_SetTextColor(RGB_BLACK);	// 글자색 : Black

	LCD_DisplayText(0,0,"TIM4(CH3) PWM MODE");  // Title

	LCD_SetBackColor(RGB_YELLOW);	//글자배경색 : Yellow
	LCD_DisplayText(1,0,"DUTY: 10%");
	LCD_DisplayText(2,0,"DIR :  CW");
}

/* Joystick switch가 입력되었는지를 여부와 어떤 Joystick switch가 입력되었는지의 정보를 return하는 함수  */ 
uint8_t joy_flag = 0;
uint16_t JOY_Scan(void)	// input joy stick NAVI_* 
{ 
	uint16_t key;
	key = GPIOI->IDR & 0x03E0;	// any key pressed ?
	if(key == 0x03E0)		// if no key, check key off
	{  	if(joy_flag == 0)
        		return key;
      		else
		{	DelayMS(10);
        		joy_flag = 0;
        		return key;
        	}
    	}
  	else				// if key input, check continuous key
	{	if(joy_flag != 0)	// if continuous key, treat as no key input
        		return 0x03E0;
      		else			// if new key,delay for debounce
		{	joy_flag = 1;
			DelayMS(10);
 			return key;
        	}
	}
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
