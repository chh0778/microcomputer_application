//////////////////////////////////////////////////////////////////////////
// I2C ����� �̿��� ���ӵ����� ����
//  SCL pin:  PH8 (I2C2_SCL)
//  SDA pin: PH5 (I2C2_SDA)
//  I2C mode: MASTER
// ���ӵ�����(LIS2HH12, Slave mode) X,Y,Z ���� 250ms���� ������ LCD�� ǥ�� 
//////////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"
#include "ACC_I2C.h"

void DisplayTitle(void);
void _GPIO_Init(void);
void I2C2_Init(void);
void TIMER3_Init(void);
void Display_Process(UINT16 *pBuf);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

//// void ACC_I2C_Process(UINT16 *pBuf);  // ACC_I2C.c (ACC_I2C.h) 
//// void ACC_I2C_Init(void) // ACC_I2C.c (ACC_I2C.h)
//// void LCD_Init(void); // GLCD.c (GLCD.h)

UINT8 bControl;

int main(void)
{
        UINT16 buffer[3];
    
        LCD_Init();		// LCD ���� �Լ�
        DelayMS(10);		// LCD���� ������
        DisplayTitle();		// LCD �ʱ�ȭ�鱸�� �Լ�
    
       	_GPIO_Init();		// LED, SW �ʱ�ȭ
	I2C2_Init();		//I2C2 �ʱ�ȭ
	ACC_I2C_Init();		//
        TIMER3_Init();		// ���ӵ����� ��ĵ �ֱ� ����
   
        while(1)
        {
                if(bControl)
                {
                        bControl = FALSE; 
			ACC_I2C_Process(&buffer[0]);	//  I2C����� �̿��Ͽ� ���ӵ����� ����
                        Display_Process(&buffer[0]);	// �������� LCD�� ǥ��			
                }
        }
}

////////////////////////////////////////////////////////////////////////////////////
// Master mode, Full-duplex, 8bit frame(MSB first), 
// fPCLK/32 Baud rate, Software slave management EN
void I2C2_Init(void)
{
	/*!< Clock Enable  *********************************************************/
        RCC->APB1ENR 	|= (1<<22);	// 0x00400000, I2C2 Clock EN
        RCC->AHB1ENR 	|= (1<<7);	// 0x0080, GPIOH Clock EN		
		
	/*!< I2C2 pins configuration ************************************************/
		
	/*!< I2C2 SCL pin(PH4) configuration : I2C SCL */
        GPIOH->MODER 	|= (2<<(2*4)); 		// 0x00000200, PH4 Alternate function mode
        GPIOH->OTYPER 	|=  1<<4; 		// 0010, PH4 Output type open drain
        GPIOH->OSPEEDR	|= (3<<(2*4));		// 0x00000300, PH4 Output speed (100MHz)
        GPIOH->PUPDR 	&=  ~(1<<(2*4)); 	// 0x00000000, PH4 No pull-up, pull-down
        GPIOH->AFR[0] 	|= (4<<(4*4));		// 0x00040000, Connect PH4 to AF4(I2C2)
		
	/*!< I2C2 SDA pin(PH5) configuration : I2C SDA */    
        GPIOH->MODER 	|= (2<<(2*5));		// 0x00000800, PH5 Alternate function mode
        GPIOH->OTYPER	|= 1<<5;		// 0x0020, PH5 Output type open drain
        GPIOH->OSPEEDR	|= (3<<(2*5));		// 0x00000C00, PH5 Output speed (100MHz)
        GPIOH->PUPDR 	&=  ~(1<<(2*5)); 	// 0x00000000, PH5 No pull-up, pull-down
        GPIOH->AFR[0] 	|= (4<<(4*5));		// 0x00400000, Connect PH5 to AF4(I2C2)
		
	I2C_InitTypeDef I2C_InitStructure;

	/* I2C2 configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;	//standard mode
  
	/* Initialize the I2C2 peripheral */
	I2C_Init(I2C2, &I2C_InitStructure);
  
	/* Enable the I2C2 peripheral */
	I2C_Cmd(I2C2, ENABLE);
  }
	

void TIMER3_Init(void)	// ���ӵ����� ���� �ֱ� ����: 250ms
{
        RCC->APB1ENR 	|= (1<<1);	// TIMER3 Clock Enable
     
        TIM3->PSC 	= 8400-1;	// Prescaler 84MHz/8400 = 10KHz (0.1ms)  
        TIM3->ARR 	= 2500-1;	// Auto reload  0.1ms * 2500 = 250ms

        TIM3->CR1	&= ~(1<<4);	// Countermode = Upcounter (reset state)
        TIM3->CR1 	&= ~(3<<8);	// Clock division = 1 (reset state)
        TIM3->EGR 	|=(1<<0);	// Update Event generation    

        TIM3->DIER 	|= (1<<0);	// Enable Tim3 Update interrupt
        NVIC->ISER[0] 	|= ( 1 << 29 );	// Enable Timer3 global Interrupt
        TIM3->CR1 	|= (1<<0);	// Enable Tim3 Counter    
}

void TIM3_IRQHandler(void)	// 250ms int
{
        static UINT16 LED_cnt = 0;
    
        TIM3->SR &= ~(1<<0);		//Interrupt flag Clear

	bControl = TRUE;		// 250ms���� ���� ����
	GPIOG->ODR ^= 0x02;		// Test LED
    
        LED_cnt++;								
        if(LED_cnt >= 2)		// �۵� Ȯ�ο�
        {
                LED_cnt = 0;     		
                GPIOG->ODR ^= 0x01;	// LED0 Toggle 500ms
        }
}

void Display_Process(UINT16 *pBuf)
{
        char str[10];    
    
        sprintf(str, "%5d", pBuf[0]);	//X AXIS (16��Ʈ: 0~65535/2)
        LCD_DisplayText(1,4,str);
    
        sprintf(str, "%5d", pBuf[1]);	//Y AXIS (16��Ʈ: 0~65535/2)
        LCD_DisplayText(2,4,str);
    
        sprintf(str, "%5d", pBuf[2]);	//Z AXIS (16��Ʈ: 0~65535/2)
        LCD_DisplayText(3,4,str);
}

void _GPIO_Init(void)
{
	// LED (GPIO G) ����
	RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) ���� 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) ���� 
	RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x0200;	// GPIOF 9 : Push-pull  	
 	GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 

        GPIOG->ODR &= 0x00;	// LED0~7 Off 
}

void DelayMS(unsigned short wMS)
{
        register unsigned short i;

        for (i=0; i<wMS; i++)
                DelayUS(1000);		//1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
        volatile int Dly = (int)wUS*17;
         for(; Dly; Dly--);
}

void DisplayTitle(void)
{
        LCD_Clear(RGB_WHITE);
        LCD_SetFont(&Gulim8);		//��Ʈ 
        LCD_SetBackColor(RGB_GREEN);
        LCD_SetTextColor(RGB_BLACK);    //���ڻ�
        LCD_DisplayText(0,0,"3AXIS Accelerometer");  // Title
		
        LCD_SetBackColor(RGB_WHITE);    //���ڹ���

	LCD_DisplayText(1,0," X :");	//X AXIS
	LCD_DisplayText(2,0," Y :");	//Y AXIS
	LCD_DisplayText(3,0," Z :");	//Z AXIS
}
