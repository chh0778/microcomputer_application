#include "stm32f4xx.h"
#include "GLCD.h"

void _GPIO_Init(void);
uint16_t KEY_Scan(void);

void BEEP(void);
void DisplayInitScreen(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

uint8_t	SW0_Flag, SW1_Flag;

int main(void)
{
	_GPIO_Init(); // GPIO (LED & SW) 초기화
 
	DelayMS(100);

	GPIOD->ODR = 0xAA;	// LED 초기값: LED0~7 Off
	GPIOB->ODR &= ~0x4081;	
	while(1)
	{
		switch(KEY_Scan())
		{
			case 0x000E : 	//SW0
				if (SW0_Flag==0) {
					GPIOD->ODR |= 0x00FF; // ALL LED ON		
					SW0_Flag = 1;
				}
				else {
					GPIOD->ODR &= ~0x00FF; // ALL LED OFF
					SW0_Flag = 0;
				}
			break;
		}  // switch(KEY_Scan())
		if (SW1_Flag==0) {
			GPIOB->ODR |= 0x4081; // LED0,7,14 ON		
			SW1_Flag = 1;
			DelayMS(500);
		}
		else {
			GPIOB->ODR &= ~0x4081; // LED0,7,14 OFF
			SW1_Flag = 0;
			DelayMS(500);
		}

	}  // while(1)
}


void _GPIO_Init(void)
{
	// LED (GPIO D) 설정
	RCC->AHB1ENR	|=  0x00000008;	// RCC_AHB1ENR : GPIOD(bit#3) Enable							
	GPIOD->MODER 	|=  0x00005555;	// GPIOD 0~7 : Output mode (0b01)						
	GPIOD->OTYPER	&= ~0x00FF;	// GPIOD 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOD->OSPEEDR 	|=  0x00005555;	// GPIOD 0~7 : Output speed 25MHZ Medium speed 

	// SW (GPIO G) 설정
	RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	&= ~0x000000FF;	// GPIOG 0~3 : Input mode (reset state)						
	GPIOG->PUPDR 	&= ~0x000000FF;	// GPIOG 0~3 : Floating input (No Pull-up, pull-down) :reset state

	// LED (GPIO B) 설정 
    	RCC->AHB1ENR	|=  0x00000002; // RCC_AHB1ENR : GPIOB(bit#1) Enable							
	GPIOB->MODER 	|=  0x10004001;	// GPIOB 0,7,14 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x4081;	// GPIOB 0,7,14 : Push-pull  	
 	GPIOF->OSPEEDR 	|=  0x10004001;	// GPIOB 0,7,14 : Output speed 25MHZ Medium speed 

}	

uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{ 
	uint16_t key;
	key = GPIOG->IDR & 0x000F;	// any key pressed ?
	if(key == 0x000F)		// if no key, check key off
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
        		return 0x000F;
      		else			// if new key,delay for debounce
		{	key_flag = 1;
			DelayMS(10);
 			return key;
        	}
	}
}

void BEEP(void)			/* beep for 30 ms */
{ 	
	GPIOF->ODR |=  0x0200;	// PF9 'H' Buzzer on
	DelayMS(30);		// Delay 30 ms
	GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
}

void DelayMS(unsigned short wMS)
{
    register unsigned short i;
    for (i=0; i<wMS; i++)
        DelayUS(1000);         		// 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
    volatile int Dly = (int)wUS*17;
    for(; Dly; Dly--);
}


