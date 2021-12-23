/////////////////////////////////////////////////////////////
// Step motor Position/Speed Control (Output Compare Mode)
// Pulse output pin: PE13(TIM1 CH3), Direction pin: PE14(GPIO)
//	- OC1을 통한 펄스 출력(ARR을 이용하여 속도 제어) 
//	- Compare Match Interrupt(CC3I) 발생을 이용하여 위치제어(출력펄스 수 제어)  
//	- Step driver에 펄스 전송 
/////////////////////////////////////////////////////////////
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

//#define GOAL_DEG	720     //목표값
#define STEP_DEG	7.5     //스탭 각
//#define GOAL_RPM	5     //목표 RPM
//12.5ms 주기를 가지고있다.
//48개의 펄스가 나옴

void _GPIO_Init(void);
void _EXTI_Init(void);
uint16_t KEY_Scan(void);
void TIMER1_OC_Init(void);	// Timer 1 (Output Compare mode)

void DisplayInitScreen(void);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

uint16_t GOAL_DEG=360;	 //목표값
//uint16_t STEP_DEG= 7.5;	 //스탭 각
uint16_t GOAL_RPM=100;   //목표 RPM


UINT16 INT_COUNT;
float fARR;
int main(void)
{
	_GPIO_Init();
	_EXTI_Init();
	LCD_Init();	
	DelayMS(10);
	//BEEP();
   	DisplayInitScreen();	// LCD 초기화면
	GPIOG->ODR &= 0xFF00;// 초기값: LED0~7 Off

	TIMER1_OC_Init();	// TIM1 Init (Output Compare mode: CCI 발생)                  
 
	while(1)
	{
		// SW4~7을 사용하여 펄스 출력 및 방향 제어
		switch(KEY_Scan())
		{
                        case SW0_PUSH  : 	//SW7 (펄스 출력)
                                GOAL_RPM +=90;
                                fARR = (1000000*STEP_DEG)/(12*GOAL_RPM);  // us 단위                                      //속도 명령이 들어오면 arr로 변화시키는 코드
                                TIM1->ARR = fARR-1;    // 주기 = 1us * fARR = fARR us
                                break;
                        case SW1_PUSH  : 	//SW7 (펄스 출력)
                                GOAL_RPM -=90;
                                fARR = (1000000*STEP_DEG)/(12*GOAL_RPM);  // us 단위                                      //속도 명령이 들어오면 arr로 변화시키는 코드
                                TIM1->ARR = fARR-1;    // 주기 = 1us * fARR = fARR us
                                break;
                        case SW2_PUSH  : 	//SW7 (펄스 출력)
                                GOAL_RPM +=50;
                                fARR = (1000000*STEP_DEG)/(12*GOAL_RPM);  // us 단위                                      //속도 명령이 들어오면 arr로 변화시키는 코드
                                TIM1->ARR = fARR-1;    // 주기 = 1us * fARR = fARR us
                                break;
                        case SW3_PUSH  : 	//SW7 (펄스 출력)
                                GOAL_RPM-=50;
                                fARR = (1000000*STEP_DEG)/(12*GOAL_RPM);  // us 단위                                      //속도 명령이 들어오면 arr로 변화시키는 코드
                                TIM1->ARR = fARR-1;    // 주기 = 1us * fARR = fARR us
                                break;
			case SW7_PUSH  : 	//SW7 (펄스 출력)
				GPIOG->ODR &= 0x0F;	// LED4~7 OFF
				GPIOG->ODR |= 0x80;	// LED7 ON
				TIM1->CCER |= (1<<4*(3-1));// CC3E Enable                               //init  에서 멈춰있었으니까 시작시켜야한다.
				TIM1->CR1 |= (1<<0);	// TIM1 Enable                                          //init  에서 멈춰있었으니까 시작시켜야한다.
			break;
			case SW4_PUSH  : 	//SW4 (Direction 변경: L-CCW)
				GPIOG->ODR &= 0x0F;	// LED4~7 OFF
				GPIOG->ODR |= 0x10;	// LED4 ON
 				GPIOE->ODR |= 0x4000; 	// Dir. CCW  변경 (PE14)
			break;
			case SW5_PUSH  : 	//SW5 (Direction 변경: H-CW)
				GPIOG->ODR &= 0x0F;	// LED4~7 OFF
				GPIOG->ODR |= 0x20;	// LED5 ON
				GPIOE->ODR &= ~0x4000; 	// Dir. CW  변경 (PE14)
			break;	
			case SW6_PUSH  : 	//SW6 (RESET)
				GPIOG->ODR &= 0x0F;	// LED4~7 OFF
				GPIOG->ODR |= 0x40;	// LED6 ON
				GPIOE->ODR |= 0x8000; 	// RESET (PE15)
			break;	
		}
	}
}


void TIMER1_OC_Init(void) //TIM1_CH3 CC 
{
	// PE13: TIM1_CH3
	// PE13을 출력설정하고 Alternate function(TIM1_CH3)으로 사용 선언
	RCC->AHB1ENR   |= (1<<4);   // RCC_AHB1ENR GPIOE Enable
	GPIOE->MODER   |= (2<<2*13);   // GPIOE PIN13 Output Alternate function mode               
	GPIOE->OSPEEDR |= (3<<2*13);   // GPIOE PIN13 Output speed (100MHz High speed)
	GPIOE->OTYPER  = 0x00000000;   // GPIOE PIN13 Output type push-pull (reset state)
	GPIOE->PUPDR   |= (1<<2*13);   // GPIOE PIN13 Pull-up
	GPIOE->AFR[1]  |= (1<<4*(13-8)); // Connect TIM1 pins(PE13) to AF1(TIM1/2)

	// PE14 : Step Driver Direction (GPIO)
	GPIOE->MODER   |= (1<<(2*14));  
	GPIOE->OSPEEDR |= (3<<(2*14));
	GPIOE->PUPDR   |= (1<<(2*14));  
	GPIOE->ODR     |= (1<<14); 

	// PE15 : Step Driver Reset (GPIO)
	GPIOE->MODER   |= (1<<(2*15));  
	GPIOE->OSPEEDR |= (3<<(2*15));
	GPIOE->PUPDR   |= (1<<(2*15));  
	GPIOE->ODR     |= (1<<15); 
   
	// Timerbase Mode
	RCC->APB2ENR   |= 0x01;// RCC_APB1ENR TIMER1 Enable

	TIM1->PSC = 168-1;   // Prescaler 168,000,000Hz/168= 1MHz (1us)

	fARR = (1000000*STEP_DEG)/(12*GOAL_RPM);  // us 단위                                      //속도 명령이 들어오면 arr로 변화시키는 코드
	TIM1->ARR = fARR-1;    // 주기 = 1us * fARR = fARR us
	
	TIM1->CR1 &= ~(1<<4);   // DIR: Countermode = Upcounter (reset state)
	TIM1->CR1 &= ~(3<<8);   // CKD: Clock division = 1 (reset state)
	TIM1->CR1 &= ~(3<<5);    // CMS(Center-aligned mode Sel): No(reset state)

	TIM1->EGR |= (1<<0);    // UG: Update generation 
    
	// Output/Compare Mode
	TIM1->CCER &= ~(1<<4*(3-1));   // CC3E: OC3 Active                                              //처음에 disable 시켜놨따. ==> 왜지
	TIM1->CCER |= (1<<(4*(3-1)+1));  // CC3P: OCPolarity_Active High                        //펄스를 반전시켜서 내보낸다. -->rising edge라서 두 번 출력하는

	TIM1->CCR3 = 100;   // TIM1_Pulse                                                                       //arr이 변하는데 ccr이 arr보다 커지는 경우도 생기니까  최대한 작게 해놔야한다.

	TIM1->BDTR |= (1<<15);  // main output enable                                                   //고급제어 타이머라서 외부핀 출력하려면  enable 시켜야한다.
   
	TIM1->CCMR2 &= ~(3<<8*0); // CC3S(CC1channel): Output 
	TIM1->CCMR2 &= ~(1<<3); // OC3PE: Output Compare 3 preload disable
	TIM1->CCMR2 |= (3<<4);   // OC3M: Output Compare 3 Mode : toggle        

	TIM1->CR1 &= ~(1<<7);   // ARPE: Auto reload preload disable
	TIM1->DIER |= (1<<3);   // CC3IE: Enable the Tim1 CC3 interrupt
   
	NVIC->ISER[0] |= (1<<27); // TIM1_CC
	TIM1->CR1 &= ~(1<<0);   // CEN: Disable the Tim1 Counter                                //처음에 멈춰있음==> 왜지
}


void TIM1_CC_IRQHandler(void)      //RESET: 0
{
	if ((TIM1->SR & 0x08) != RESET)	// CC3 interrupt flag 
	{
		TIM1->SR &= ~0x08;	// CC3 Interrupt Claer
        	INT_COUNT++;
        	if (INT_COUNT >= 2*GOAL_DEG/STEP_DEG)  // 출력 펄스수 제어 
        	{
           		TIM1->CCER &= ~(1<<4*(3-1));// CC3E Disable 
           		TIM1->CR1 &= ~(1<<0); // TIM1 Disable
           		INT_COUNT= 0;
        	}
	}
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

void _EXTI_Init(void)
{
	RCC->AHB1ENR 	|= 0x0080;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x4000;	// Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&= 0x0000FFFF;	// GPIOH PIN8~PIN15 Input mode (reset state)				 
	
	SYSCFG->EXTICR[2] |= 0x0077; 	// EXTI8,9에 대한 소스 입력은 GPIOH로 설정 (EXTICR3) (reset value: 0x0000)	
	
	EXTI->FTSR |= 0x000100;		// Falling Trigger Enable  (EXTI8:PH8)
	EXTI->RTSR |= 0x000200;		// Rising Trigger  Enable  (EXTI9:PH9) 
	EXTI->IMR  |= 0x000300;  	// EXTI8,9 인터럽트 mask (Interrupt Enable)
		
	NVIC->ISER[0] |= (1<<23);   	// Enable Interrupt EXTI8,9 Vector table Position 참조
}

void EXTI9_5_IRQHandler(void)		// EXTI 5~9 인터럽트 핸들러
{
	if(EXTI->PR & 0x0100) 		// EXTI8 nterrupt Pending?
	{
		EXTI->PR |= 0x0100; 	// Pending bit Clear
	}
	else if(EXTI->PR & 0x0200) 	// EXTI9 Interrupt Pending?
	{
        EXTI->PR |= 0x0200; 	// Pending bit Clear
	}
}

void BEEP(void)			// Beep for 20 ms 
{ 	GPIOF->ODR |= 0x0200;	// PF9 'H' Buzzer on
	DelayMS(20);		// Delay 20 ms
	GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
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

	LCD_DisplayText(0,0,"STEP Motor Control");  // Title
	LCD_DisplayText(1,0,"Position/Speed");  // Title
	LCD_DisplayText(2,0,"TIM1(CH3) OC MODE");  // Title

	LCD_SetBackColor(RGB_YELLOW);	//글자배경색 : Yellow
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
		{   key_flag = 1;
            DelayMS(10);
            return key;
        }
	}
}
