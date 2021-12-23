//////////////////////////////////////////////////////////////////////
// PR2: �ݵ�ü ���� �����
// ������: 2017130036_������
// �ֿ䳻��
//�����۰� ���ͼ� ���� ȭ�а����� ��ġ�� �ݵ�ü ���������
//process chamber ���θ� �����ϴ� ���μ��� ���� ���α׷�
//�ֿ� ����
//1. �����Է¸��---> ADC1_IN1 ���
//�������׿� ���� ��ȭ�ϴ� ������ 0~4095�� ������ ������ ��ȭ��Ű�� ADC1�� ���
//������ ������ �̿��Ͽ� 0~4095�� ���� 7����Ͽ� �µ�����, �з°���, O2������ �ɸ��� �ð����� ��ȯ�ϰ�
//10����Ͽ� PLASMA�������� PWM�� Duty Ratio�� �����Ѵ�.

//2. ����ġ�� sw4,5,6,7 (GPIOH), LED�� 0������ 7�� ���(GPIOG) ����Ѵ�.
//SW4: ���� �Է¸�忡�� �Է°� ���� �� ����
//SW5: ��庯ȯ(�Է�<->����)
//SW6: �� ��忡�� ���� ���μ��� ����
//SW7: �����Է¸�忡�� �Է��� �Ķ���� ����

//3. FRAM�� �̿��Ͽ� �����Է¸���� ���� �ð��� ����
//SW4�� ������ �����Է¸�忡�� �ش� ������ �ð��� �����Ѵ�.
//������ ������ ���α׷��� ��������� �� LCDȭ�鿡 ����ϱ����� ����Ѵ�.
//������ ������ ���� ������ �� �� ��ǥ ���� �ð��� LCD ȭ�鿡 ����ϱ� ���� ����Ѵ�.

//4. Ÿ�̸� ���
//TIM3�� ����Ͽ� 0.1s ���� ���ϴ� �����ð��� LCDȭ�鿡 ���÷����ϱ�
//TIM2�� ����Ͽ� PLASMA������ Duty Ratio�� ���� pwm ��ȣ �߻��� ���� ���
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

//flag �����
// ��Ȳ flag
//0: �Է¸�� ���, 1: ������ ���, 2: �����Է¸��, 3: ������
int flag=0;

//0: �����Է� ����, 1: �µ��� �Է�, 2: �з°� �Է�, 3: O2�� �Է�, 4: plasma ����
int processing_op =0;
//���μ����� ������ �˷��ִ� flag
//0: ������, 1: �µ� ���μ���, 2: �з� ���μ���, 3: o2 ���μ���, 4:�ö�� ���μ���
int process_done=0;

//fram ���� ���ǿ� ���� ���� �ð��� 10�� ���� ���� �����Ѵ�.
//�µ���           //�з°�             // ��Ұ�          //plasma duty ratio
   int te;              int pr;                 int o2;                  int dr;

//������ ������ �����Ű�� flag
//0: ������ �ƴ�, 1: ������
int process_flag=0;

//0.1s ���ͷ�Ʈ���� 1�� �����Ѵ�.
//������ ���� �����߿� �ð������� LCDȭ�鿡 ǥ���ϱ� ���� ���
int cnt=0;

void ADC_IRQHandler(void)
{
  ADC1->SR &= ~(1<<1);		// EOC flag clear
  
  ADC_Value = ADC1->DR;		// Reading ADC result
  
  //ADC�� ���� �µ���, �з°�, O2��, PLASMA ��
  te=5*((int)(ADC_Value/585.1)+2);
  pr=5*((int)(ADC_Value/585.1)+1);
  o2=5*((int)(ADC_Value/585.1));
  dr=(int)(ADC_Value/410);
  
  //�����Է¸�忡��
  if(processing_op==1) {           //�µ��� �Է�
    LCD_SetTextColor(RGB_RED);	//���ڻ�
    LCD_DisplayChar(3,4,te/10 + 0x30);
    LCD_DisplayText(3,5,".");
    LCD_DisplayChar(3,6,te%10 + 0x30);
  }
  else if(processing_op==2){       //�з°� �Է�
    LCD_SetTextColor(RGB_RED);	//���ڻ�
    LCD_DisplayChar(4,4,pr/10 + 0x30);
    LCD_DisplayText(4,5,".");
    LCD_DisplayChar(4,6,pr%10 + 0x30);
  }
  else if(processing_op==3)       //o2�� �Է�
  {
    LCD_SetTextColor(RGB_RED);	//���ڻ�
    LCD_DisplayChar(5,4,o2/10 + 0x30);
    LCD_DisplayText(5,5,".");
    LCD_DisplayChar(5,6,o2%10 + 0x30);
  }
  else if(processing_op==4)       //plasma��
  {
    LCD_SetTextColor(RGB_RED);	//���ڻ�
    LCD_DisplayChar(6,4,dr+ 0x30);
  }
  
  ADC1->CR2 |=  (1<<30); 	// swstart adc
}

int main(void)
{
  LCD_Init();   // LCD ���� �Լ�
  DelayMS(100);   // LCD���� ������
  _GPIO_Init(); //GPIO �ʱ�ȭ
  _ADC_Init();
  Fram_Init();                    // FRAM �ʱ�ȭ H/W �ʱ�ȭ
  Fram_Status_Config();   // FRAM �ʱ�ȭ S/W �ʱ�ȭ
  DisplayOn();   //LCD �ʱ�ȭ�鱸�� �Լ�
  TIMER3_Init();		// ����Ÿ�̸�(TIM3) �ʱ�ȭ : up counting mode
  TIMER2_PWM_Init();		// TIM4 Init (PWM mode)
  
  _EXTI_Init();	// EXTI �ʱ�ȭ
  GPIOG->ODR &= 0x00;   // LED0~7 Off 
  GPIOG->ODR |= 0x01;   // LED1 on
  
  while(1)
  {
    if(process_flag==1)                       //�����忡�� SW7�� ������ if�� ����
    {
      flag=3;                 //���� �� �����
      GPIOG->ODR |= 0x04;   // LED2 on
      LCD_DisplayText(2,13,"W");
      GPIOG->ODR |= 0x08;   // LED3 on
      DelayMS(500);
      
//���� ����
      process_done=1;                           //�µ����� ���� ���¸� �����ϴ�flag                  
      
      Process_Time_Setting();                   //���� LCD ȭ�� ����
      
//�µ� ����
      GPIOG->ODR |= 0x10;                    // LED4 on
      LCD_DisplayText(3,0,"*");
      while(process_done!=2){}                  //�µ����� �ð��� �Ǳ� ������ while���� ����
      
      LCD_DisplayText(3,0," ");
      GPIOG->ODR &= ~0x10;                 //LED4 OFF
      
//�з� ���μ���
      LCD_DisplayText(4,0,"*"); 
      GPIOG->ODR |= 0x20;                    // LED5 on
      while(process_done!=3){}                  //�з°��� �ð��� �Ǳ� ������ while���� ����
      
      LCD_DisplayText(4,0," ");
      GPIOG->ODR &= ~0x20;                    // LED5 off
      
//O2 ���μ��� & �ö�� ���μ���
      LCD_DisplayText(5,0,"*");
      LCD_DisplayText(6,0,"*");
      GPIOG->ODR |= 0x40;                   // LED6 oN 
      GPIOG->ODR |= 0x80;                   // LED7 oN 
      
      int i=Fram_Read(1206);
      TIM2->CCR3 = i*2000;                       //�ö�� ������ �ش��ϴ� Duty Ratio ����
      
      while(process_done!=0){}                  //o2 % �ö�� ���� �ð��� �Ǳ� ������ while���� ����
      
      TIM2->CCR3  = 0;		        // �ö�� ���� ����
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
  //sw4 �����Է¸�� ����
  if(EXTI->PR & (1<<12))                   // EXTI12Interrupt Pending(�߻�) ����?
  {
    EXTI->PR |= (1<<12); 	
    if(flag==2){                       //���� �Է� ���
      if(processing_op==1) {           //�µ����� �ð�
        Fram_Write(1203,te); 
      }
      else if(processing_op==2){       //�з°��� �ð�
        Fram_Write(1204,pr); 
      }
      else if(processing_op==3) {      //o2���� �ð�
        Fram_Write(1205,o2); 
      }
      else if(processing_op==4)  {     //plasma Duty Ratio ��
        Fram_Write(1206, dr); 
      }
    }
  }
  //sw5 �Է�<-->���� ��� ��ȯ
  else if(EXTI->PR & (1<<13))                   // EXTI13 Interrupt Pending(�߻�) ����?
  {
    EXTI->PR |= (1<<13); 
    if(flag==0){                            //�Է� ���--> ���� ���
      flag=1; 
      GPIOG->ODR &= ~0x01;   // LED0 off
      GPIOG->ODR |= 0x02;   // LED1 on
      LCD_SetTextColor(RGB_RED);
      LCD_DisplayText(2,5,"R");                 //������'R'���� LCD�� ���
    }
    else if(flag==1){                       //���� ��� --> �Է� ���
      flag=0;
      GPIOG->ODR |= 0x01;   // LED0 on
      GPIOG->ODR &= ~0x02;   // LED1 off
      LCD_SetTextColor(RGB_RED);
      LCD_DisplayText(2,5,"I");                 //�Է¸��'I'���� LCD�� ���
    }
  }
  //sw6 ��� ����
  else if(EXTI->PR & (1<<14))                   // EXTI14 Interrupt Pending(�߻�) ����?
  {
    EXTI->PR |= (1<<14); 	
    if(flag==0){                            //�Է� ��� --> �����Է¸��
      flag=2;
      GPIOG->ODR |= 0x04;   // LED2 on
    }
    else if(flag==1){                       //���� ��� --> ����
      process_flag=1;                           //main �� �ȿ� �ִ� ���� ��ɾ �����Ű�� ���� flag on
    }
    else if(flag==2){                       //���� �Է� ��� --> �Է´��
      flag=0;
      GPIOG->ODR &= ~0x04;   // LED2 off
      GPIOG->ODR &= ~0xF0;   // LED4~7 off
      LCD_DisplayText(3,0,"*");
      LCD_DisplayText(4,0," ");
      LCD_DisplayText(5,0," ");
      LCD_DisplayText(6,0," ");
      processing_op =0;
    }
    else if(flag==3){                       //���� (�������� ��) --> ������
      flag=1;
      GPIOG->ODR &= ~0x04;          // LED2 off
      LCD_DisplayText(3,0,"*");           //LCD �ʱ�ȭ��ó�� * ǥ��
    }
  }
  //sw7 �����Է¸�忡�� �׸� ����
  else if(EXTI->PR & (1<<15))                   // EXTI15 Interrupt Pending(�߻�) ����?
  {
    EXTI->PR |= (1<<15); 	
    if(flag==2){                       //���� �Է� ��� ����
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

// 0.1s Interrupt, ���� ���� �� LCD ȭ�鿡 �ð� ����� ���� TIM
void TIM3_IRQHandler(void)  	
{
  
  TIM3->SR &= ~(1<<0);	// Interrupt flag Clear
  if(process_done==1){    //�µ� ���μ��� ����
    
    LCD_DisplayChar(3,9,cnt/10+0x30);
    LCD_DisplayText(3,10,".");
    LCD_DisplayChar(3,11,cnt%10+0x30);
    cnt++;
    if(cnt>Fram_Read(1203)){
      process_done=2;
      cnt=0;
    }
  }
  else if(process_done==2){    //�з� ���μ��� ����
    
    LCD_DisplayChar(4,9,cnt/10+0x30);
    LCD_DisplayText(4,10,".");
    LCD_DisplayChar(4,11,cnt%10+0x30);
    cnt++;
    
    if(cnt>Fram_Read(1204)){
      process_done=3;
      cnt=0;
    }
  }
  else if(process_done==3){    //o2 ���μ��� ����
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

//PLASMA �������� PWM ��ȣ �ܺ������ ���� Ÿ�̸�
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
  
  // PB10�� ��¼����ϰ� Alternate function(TIM2_CH3)���� ��� ���� : PWM ���
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
  
  TIM2->CCER	&= ~(1<<9);	// CC3P=0: CC3 Output Polarity (OCPolarity_High : OC3���� �������� ���)
  
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
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// (1<<0) ENABLE GPIOA CLK (stm32f4xx.h ����)
  GPIOA->MODER |= (3<<2*1);		// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
  
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	// (1<<8) ENABLE ADC1 CLK (stm32f4xx.h ����)
  
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
  // LED (GPIO G) ����
  RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
  GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
  GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
  GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
  
  // SW (GPIO H) ���� 
  RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
  GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
  GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state
}	

/* EXTI (EXTI12~15(SW4~SW7))�ʱ� ����  */
void _EXTI_Init(void)
{
  RCC->AHB1ENR 	|= 0x00000080;	// RCC_AHB1ENR GPIOH Enable
  RCC->APB2ENR 	|= 0x00004000;	// Enable System Configuration Controller Clock
  
  GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH PIN8~PIN15 Input mode (reset state)		(sw0~sw7)		 
  
  SYSCFG->EXTICR[3] |= 0x7777; 	// EXTI12~15�� ���� �ҽ� �Է��� GPIOH�� ����
  
  
  EXTI->FTSR |= (15<<12);		// EXTI12~15: Falling Trigger Enable
  
  EXTI->IMR  |= (15<<12);  	// EXTI12~15���ͷ�Ʈ mask (Interrupt Enable) ����
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
  LCD_Clear(RGB_YELLOW);                 //��� ��������� �ʱ�ȭ
  LCD_SetFont(&Gulim8);
  LCD_SetBackColor(RGB_YELLOW);   //���� �����
  
  //�̸� �� �й�
  LCD_SetTextColor(RGB_BLUE);   
  LCD_DisplayText(0,0,"LCH 2017130036");
  LCD_DisplayText(1,0,"SPC monitor");
  
  //����
  LCD_SetTextColor(RGB_BLACK);
  LCD_DisplayText(2,0,"Mode:  Wafer:");
  LCD_DisplayText(3,0," TE:   s    s");
  LCD_DisplayText(4,0," PR:   s    s");
  LCD_DisplayText(5,0," O2:   s    s");
  LCD_DisplayText(6,0," DR:");
  
  //�ʱ����
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