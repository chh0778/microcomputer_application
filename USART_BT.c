//////////////////////////////////////////////////////////////////////
// PC <-> (UART1)Controller(Kit)(UART4) <-> BT module <-> Mobile-phone
// USART1(PC): TX pin: PA9,  RX pin: PA10 
// UART4(BT):  TX pin: PC10, RX pin: PC11, RST pin(GPIO): PC13
// TX: Polling ���, RX: Interrupt ��� 
// pc(�������͹̳�)���� �Է��� ���ڸ� �޾� (lcdǥ�õ�,)
//Mobilephone���� ������
//���ڸ� TX�� ���� PC(Hyper terminal)�� �����ϰ�, 
// PC���� ������ ���ڸ� �޾� LCD�� ǥ��
//
//////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"
#include "default.h"
#include "Util.h"

#include "Que.h"

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

void DisplayTitle(void);
void _GPIO_Init(void);
uint16_t KEY_Scan(void);

void USART1_Init(void);
void UART4_Init(void);
void USART1_BRR_Configuration(uint32_t USART_BaudRate);
void UART4_BRR_Configuration(uint32_t USART_BaudRate);

void SerialSendChar_PC(uint8_t c);
void SerialSendString_PC(char *s);
void SerialSendChar_BT(uint8_t c);
void SerialSendString_BT(char *s);

int IUART_GetSize(UINT8 id);
void IUART_GetData(UINT8 id, uint8 *ch);
void IUART_SendData(UINT8 id, UINT8 ch);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

unsigned char strBuffer[2][100];
unsigned char strBufferIdx[2];
int size;
char data;
char ch;

int main(void)
{
	LCD_Init();	// LCD ���� �Լ�
	DelayMS(100);	// LCD���� ������
    
	_GPIO_Init();
	USART1_Init();
	UART4_Init();
      	Que_Clear(&txQue[1]);	// QUE clear
      	Que_Clear(&rxQue[1]);  
      	Que_Clear(&txQue[2]);	// QUE clear
      	Que_Clear(&rxQue[2]);  

	GPIOG->ODR &= 0x00;	// LED0~7 Off 
	DisplayTitle();	//LCD �ʱ�ȭ�鱸�� �Լ�
	BEEP();

	strBufferIdx[0] = 0;
	strBufferIdx[1] = 0;
	while(1)
	{
		// PC --> BT
	        size = IUART_GetSize(1); // PC�� Ring buffer
        	if(size != 0)
        	{
            		for(int i = 0; i < size; i++)
            		{
                		IUART_GetData(1,&data);
                		IUART_SendData(2,data);
                		if(data == '\r')
                		{
                    			LCD_DisplayText(0,0, "BlueTooth");
                    			LCD_SetFont(&Gulim10);
                    			LCD_SetBackColor(RGB_WHITE);
                    			LCD_SetTextColor(RGB_BLACK);
                    			LCD_DisplayText(1,0,"[PC SEND]");
                    			LCD_DisplayText(2,0,strBuffer[0]);
                  
		                    	strBufferIdx[0] = 0;
              			      	for(int j=0; j<100; j++)
                            			strBuffer[0][j] = 0;
                		}
                		else if(data != '\n')
                		{
                    			strBuffer[0][strBufferIdx[0]++] = data;
                 		}
			}
			UART4->CR1 |= (1<<7);	// TXE interrupt Enable

		} // if(size != 0)

		// BT --> PC
        	size = IUART_GetSize(2);  // BT�� Ring buffer
        	if(size != 0)
        	{
            		for(int i = 0; i < size; i++)
            		{
                		IUART_GetData(2,&data);
                		IUART_SendData(1,data);
                		if(data == '\r')
                		{
                    			LCD_DisplayText(3,0,"[BT SEND]");
                    			LCD_DisplayText(4,0,strBuffer[1]);
                  
              			      	strBufferIdx[1] = 0;
                    			for(int j=0; j<100; j++)
                            		strBuffer[1][j] = 0;
				}
				else if(data != '\n')
				{
					strBuffer[1][strBufferIdx[1]++] = data;
				}
			}
			USART1->CR1 |= (1<<7);	// TXE interrupt Enable
		}  // if(size != 0)    
 	}
}

void USART1_IRQHandler(void)	
{       
	if ( USART1->SR & USART_SR_RXNE ) // USART_SR_RXNE= 1? RX Buffer Full?
    	// #define  USART_SR_RXNE ((uint16_t)0x0020)    //  Read Data Register Not Empty     
	{
		ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// ���ŵ� ���� ����
		Que_PutByte(&rxQue[1] , ch);
                LCD_DisplayChar(1,0,ch); 	// ���ŵ� ���ڸ� LCD�� display
	} 
        // DR �� ������ SR.RXNE bit(flag bit)�� clear �ȴ�. �� clear �� �ʿ���� 

        if ( USART1->SR & USART_SR_TXE ) // USART_SR_TXE= 1? TX Buffer EMPTY?
    	// #define  USART_SR_TXE ((uint16_t)0x0080)    //  WRITE Data Register Empty     
        {   
                if(Que_GetSize(&txQue[1]) != 0) 
                {
                        Que_GetByte(&txQue[1],&ch);
                        USART1->DR = ch;
                } 
                else 
                        USART1->CR1 &= ~(1<<7);  // TXE interrupt Disable                       //��Ȳ�� �°� ���� �״�
        }   	
}

void UART4_IRQHandler(void)	
{       
	if ( (UART4->SR & USART_SR_RXNE) ) // USART_SR_RXNE= 1? RX Buffer Full?
    	// #define  USART_SR_RXNE ((uint16_t)0x0020)    //  Read Data Register Not Empty     
	{
		ch = (uint16_t)(UART4->DR & (uint16_t)0x01FF);	// ���ŵ� ���� ����
		Que_PutByte(&rxQue[2], ch);
                LCD_DisplayChar(2,0,ch); 	// ���ŵ� ���ڸ� LCD�� display
	} 
        // DR �� ������ SR.RXNE bit(flag bit)�� clear �ȴ�. �� clear �� �ʿ���� 
        if ( UART4->SR & USART_SR_TXE ) // USART_SR_TXE= 1? TX Buffer EMPTY?
    	// #define  USART_SR_TXE ((uint16_t)0x0080)    //  WRITE Data Register Empty     
        {   
                if(Que_GetSize(&txQue[2]) != 0) 
                {
                        Que_GetByte(&txQue[2],&ch);
                        UART4->DR = ch;
                } 
                else 
                        UART4->CR1 &= ~(1<<7);  // TXE interrupt Disable
        }  
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
}

void USART1_Init(void)
{
	// USART1 : TX(PA9)
	RCC->AHB1ENR	|= (1<<0);	// RCC_AHB1ENR GPIOA Enable
	GPIOA->MODER	|= (2<<2*9);	// GPIOA PIN9 Output Alternate function mode					
	GPIOA->OSPEEDR	|= (3<<2*9);	// GPIOA PIN9 Output speed (100MHz Very High speed)
	GPIOA->AFR[1]	|= (7<<4);	// Connect GPIOA pin9 to AF7(USART1)
    
	// USART1 : RX(PA10)
	GPIOA->MODER 	|= (2<<2*10);	// GPIOA PIN10 Output Alternate function mode
	GPIOA->OSPEEDR	|= (3<<2*10);	// GPIOA PIN10 Output speed (100MHz Very High speed
	GPIOA->AFR[1]	|= (7<<8);	// Connect GPIOA pin10 to AF7(USART1)

	RCC->APB2ENR	|= (1<<4);	// RCC_APB2ENR USART1 Enable
    
	USART1_BRR_Configuration(9600); // USART1 Baud rate Configuration_pc�� ����� 9600
    
	USART1->CR1	&= ~(1<<12);	// USART_WordLength 8 Data bit
	USART1->CR1	&= ~(1<<10);	// NO USART_Parity

	USART1->CR1	|= (1<<2);	// 0x0004, USART_Mode_RX Enable
	USART1->CR1	|= (1<<3);	// 0x0008, USART_Mode_Tx Enable
	USART1->CR2	&= ~(3<<12);	// 0b00, USART_StopBits_1
	USART1->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
    
	USART1->CR1 	|= (1<<5);	// 0x0020, RXNE interrupt Enable
	USART1->CR1 	&= ~(1<<7);	// 0x0080, TXE interrupt Disable  ó���� tx ���ͷ�Ʈ�� /���д�

	NVIC->ISER[1]	|= (1<<(37-32));// Enable Interrupt USART1 (NVIC 37��)
	USART1->CR1 	|= (1<<13);	//  0x2000, USART1 Enable
}

// BlueTooth(UART4) Init
void UART4_Init(void)
{
	// UART4 : TX(PC10)
	RCC->AHB1ENR	|= (1<<2);	// RCC_AHB1ENR GPIOC Enable
	GPIOC->MODER	|= (2<<2*10);	// GPIOC PIN10 Output Alternate function mode					
	GPIOC->OSPEEDR	|= (3<<2*10);	// GPIOC PIN10 Output speed (100MHz Very High speed)
	GPIOC->AFR[1]	|= (8<<8);	// Connect GPIOC pin10 to AF8(UART4)
    
	// UART4 : RX(PC11)
	GPIOC->MODER 	|= (2<<2*10);	// GPIOC PIN11 Output Alternate function mode
	GPIOC->OSPEEDR	|= (3<<2*10);	// GPIOC PIN11 Output speed (100MHz Very High speed
	GPIOC->AFR[1]	|= (8<<12);	// Connect GPIOC pin11 to AF8(UART4)

	// UART4 : RST(PC13) : GPIO
	GPIOC->MODER	|= (1<<2*13);	// GPIOC PIN13 Output mode					
	GPIOC->OSPEEDR	|= (3<<2*13);	// GPIOC PIN10 Output speed (100MHz Very High speed)
	GPIOC->ODR 	|= (1<<13);	// BT Reset

	RCC->APB1ENR	|= (1<<19);	// RCC_APB1ENR UART4 Enable
    
	UART4_BRR_Configuration(9600); // USART Baud rate Configuration
    
	UART4->CR1	&= ~(1<<12);	// USART_WordLength 8 Data bit
	UART4->CR1	&= ~(1<<10);	// NO USART_Parity

	UART4->CR1	|= (1<<2);	// 0x0004, USART_Mode_RX Enable
	UART4->CR1	|= (1<<3);	// 0x0008, USART_Mode_Tx Enable
	UART4->CR2	&= ~(3<<12);	// 0b00, USART_StopBits_1
	UART4->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
    
	UART4->CR1 	|= (1<<5);	// 0x0020, RXNE interrupt Enable
	UART4->CR1 	&= ~(1<<7);	// 0x0080, TXE interrupt Disable

	NVIC->ISER[1]	|= (1<<(52-32));// Enable Interrupt UART4 (NVIC 52��)
	UART4->CR1 	|= (1<<13);	//  0x2000, UART4 Enable
}

void SerialSendChar_PC(uint8_t Ch) // 1���� ������ �Լ�
{
	// USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty? 
	// TX buffer Empty���� ������ ��� ���(�۽� ������ ���±��� ���)
        while((USART1->SR & USART_SR_TXE) == RESET); 
	USART1->DR = (Ch & 0x01FF);	// ���� (�ִ� 9bit �̹Ƿ� 0x01FF�� masking)
}
void SerialSendString_PC(char *str) // �������� ������ �Լ�
{
	while (*str != '\0') // ���Ṯ�ڰ� ������ ������ ����, ���Ṯ�ڰ� �����Ŀ��� ������ �޸� ���� �߻����ɼ� ����.
	{
		SerialSendChar_PC(*str);// �����Ͱ� ����Ű�� ���� �����͸� �۽�
		str++; 			// ������ ��ġ ����
	}
}
void SerialSendChar_BT(uint8_t Ch) // 1���� ������ �Լ�
{
	// USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty? 
	// TX buffer Empty���� ������ ��� ���(�۽� ������ ���±��� ���)
        while((UART4->SR & USART_SR_TXE) == RESET); 
	UART4->DR = (Ch & 0x01FF);	// ���� (�ִ� 9bit �̹Ƿ� 0x01FF�� masking)
}
void SerialSendString_BT(char *str) // �������� ������ �Լ�
{
	while (*str != '\0') // ���Ṯ�ڰ� ������ ������ ����, ���Ṯ�ڰ� �����Ŀ��� ������ �޸� ���� �߻����ɼ� ����.
	{
		SerialSendChar_BT(*str);// �����Ͱ� ����Ű�� ���� �����͸� �۽�
		str++; 			// ������ ��ġ ����
	}
}

// USART1 Baud rate ����
void USART1_BRR_Configuration(uint32_t USART_BaudRate)
{ 
	uint32_t tmpreg = 0x00;
	uint32_t APB2clock = 84000000;	//PCLK2_Frequency
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;

	// Find the integer part 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
        //  #define  USART_CR1_OVER8 ((uint16_t)0x8000) // USART Oversampling by 8 enable   
	{       // USART1->CR1.OVER8 = 1 (8 oversampling)
		// Computing 'Integer part' when the oversampling mode is 8 Samples 
		integerdivider = ((25 * APB2clock) / (2 * USART_BaudRate));  // ���Ŀ� 100�� ���� ����(�Ҽ��� �ι�°�ڸ����� �����ϱ� ����)  
	}
	else  // USART1->CR1.OVER8 = 0 (16 oversampling)
	{	// Computing 'Integer part' when the oversampling mode is 16 Samples 
		integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));  // ���Ŀ� 100�� ���� ����(�Ҽ��� �ι�°�ڸ����� �����ϱ� ����)    
	}
	tmpreg = (integerdivider / 100) << 4;
  
	// Find the fractional part 
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

	// Implement the fractional part in the register 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0)	
	{	// 8 oversampling
		tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
	}
	else	// 16 oversampling
	{
		tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
	}

	// Write to USART BRR register
	USART1->BRR = (uint16_t)tmpreg;
}

// UART4 Baud rate ����
void UART4_BRR_Configuration(uint32_t USART_BaudRate)
{ 
	uint32_t tmpreg = 0x00;
	uint32_t APB1clock = 42000000;	//PCLK1_Frequency
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;

	// Find the integer part 
	if ((UART4->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
        //  #define  USART_CR1_OVER8 ((uint16_t)0x8000) // USART Oversampling by 8 enable   
    	{       // UART4->CR1.OVER8 = 1 (8 oversampling)
		// Computing 'Integer part' when the oversampling mode is 8 Samples 
		integerdivider = ((25 * APB1clock) / (2 * USART_BaudRate));  // ���Ŀ� 100�� ���� ����(�Ҽ��� �ι�°�ڸ����� �����ϱ� ����)  
	}
	else    // UART4->CR1.OVER8 = 0 (16 oversampling)
	{	// Computing 'Integer part' when the oversampling mode is 16 Samples 
		integerdivider = ((25 * APB1clock) / (4 * USART_BaudRate));  // ���Ŀ� 100�� ���� ����(�Ҽ��� �ι�°�ڸ����� �����ϱ� ����)    
	}
	tmpreg = (integerdivider / 100) << 4;
  
	// Find the fractional part 
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

	// Implement the fractional part in the register 
	if ((UART4->CR1 & USART_CR1_OVER8) != 0)	
	{	// 8 oversampling
		tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
	}
	else	// 16 oversampling
	{
		tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
	}

	// Write to UART BRR register
	UART4->BRR = (uint16_t)tmpreg;
}

void DisplayTitle(void)
{	
        LCD_Clear(RGB_WHITE);
	LCD_SetFont(&Gulim8);
	LCD_SetBackColor(RGB_GREEN);	//����
	LCD_SetTextColor(RGB_BLACK);	//���ڻ�
	LCD_DisplayText(0,0,"BLUETOOTH");

	LCD_SetBackColor(RGB_WHITE);	//���ڹ���
}

int IUART_GetSize(UINT8 id)
{
	return Que_GetSize(&rxQue[id]);
}
void IUART_GetData(UINT8 id, uint8 *ch)
{
	Que_GetByte(&rxQue[id],ch);
}
void IUART_SendData(UINT8 id, UINT8 ch)
{   
	Que_PutByte(&txQue[id],ch);
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