#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "arm_math.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "tm_stm32f4_hcsr04.h"
#include "LED_PWM.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"

static void init_systick();
static void delay_ms(uint32_t n);
static volatile uint32_t msTicks; // counts 1 ms timeTicks
volatile char bluetooth_in;

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

// SysTick Handler (Interrupt Service Routine for the System Tick interrupt)
void SysTick_Handler1(void)
{
  msTicks++;
}

// initialize the system tick
void init_systick(void)
{
	SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  if (SysTick_Config(SystemCoreClock / 1000)) { /* SysTick 1 msec interrupts  */
    while (1);                                  /* Capture error              */
  }
}

// pause for a specified number (n) of milliseconds
void delay_ms(uint32_t n)
{
  uint32_t msTicks2 = msTicks + n;
  while(msTicks < msTicks2) ;
}

void init_PWM_pins(){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // enable clock to GPIOD
	for (int i=8; i<=11; i++)
	{
    GPIOD->MODER &= ~(0x3 << (2*i)); // clear the 2 bits corresponding to pin i
    GPIOD->MODER |= (1 << (2*i));    // set pin i to be general purpose output
	}
}

void init_LED_pins()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // enable clock to GPIOD

  for (int i=12; i<=15; i++)
  {
    GPIOD->MODER &= ~(0x3 << (2*i)); // clear the 2 bits corresponding to pin i
    GPIOD->MODER |= (1 << (2*i));    // set pin i to be general purpose output
  }
}

void LED_On(uint32_t i)
{
  GPIOD->BSRRL = 1 << (i);
}

void LED_Off(uint32_t i)
{
  GPIOD->BSRRH = 1 << (i);
}

void toggle_led(){
	for (int i=12; i<=15;i++){
		LED_On(i);
		Delayms(50);
		LED_Off(i);
	}
} 


void init_button()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // enable clock to GPIOA

  GPIOA->MODER &= ~(0x3 << (2*0)); // clear the 2 bits corresponding to pin 0
  // if the 2 bits corresponding to pin 0 are 00, then it is in input mode
}

/*USART1 will be used to connect via Bluetooth*/
uint32_t baudrate=9600;
void init_USART(uint32_t baudrate)
{
	
  GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
  USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
  NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
  
  /* enable APB2 peripheral clock for USART1
  * note that only USART1 and USART6 are connected to APB2
  * the other USARTs are connected to APB1
  */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  
  /* enable the peripheral clock for the pins used by
  * USART1, PA9 for TX and PA10 for RX
  */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  /* This sequence sets up the TX and RX pins
  * so they work correctly with the USART1 peripheral
  */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; 		// Pins 9 (TX) and 10 (RX) are used
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;			// this activates the pullup resistors on the IO pins
  
  GPIO_Init(GPIOA, &GPIO_InitStruct);				// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
  
  /* The RX and TX pins are now connected to their AF
  * so that the USART1 can take over control of the
  * pins
  */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
  
  /* Now the USART_InitStruct is used to define the
  * properties of USART1
  */
  USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;		// we want the data frame size to be 8 bits (standard)
  USART_InitStruct.USART_StopBits = USART_StopBits_1;			// we want 1 stop bit (standard)
  USART_InitStruct.USART_Parity = USART_Parity_No;			// we don't want a parity bit (standard)
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
  USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 		// we want to enable the transmitter and the receiver
  USART_Init(USART1, &USART_InitStruct);				// again all the properties are passed to the USART_Init function which takes care of all the bit setting
  
  
  /* Here the USART1 receive interrupt is enabled
  * and the interrupt controller is configured
  * to jump to the USART1_IRQHandler() function
  * if the USART1 receive interrupt occurs
  */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 		// enables the USART1 receive interrupt
  
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		// we want to configure the USART1 interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	// this sets the priority group of the USART1 interrupts - high priority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		// this sets the subpriority inside the group - high priority
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		// the USART1 interrupts are globally enabled
  NVIC_Init(&NVIC_InitStructure);				// the properties are passed to the NVIC_Init function which takes care of the low level stuff
  
  // finally this enables the complete USART1 peripheral
  USART_Cmd(USART1, ENABLE);
}

/**
 * @brief Set up GPIO pin connected to Bluetooth status line
 *
 * This function sets up the GPIO pin (PD1) as a digital input
 * with an interrupt enabled on any logic change. This pin will 
 * be connected to the status output of the HC-05 Bluetooth module.
 * By reading this input, we can detect whether there is a 
 * Bluetooth  connection (logical 1 on this input) or not (logical 0 on 
 * this input).
 */
void initBtState() {
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;		
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource1);
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;             
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;    
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;            
    EXTI_Init(&EXTI_InitStructure);                         
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;                
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;   
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;         
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               
    NVIC_Init(&NVIC_InitStructure);                                 
}



/**
 * @brief Gives current Bluetooth connection status
 *
 * This function returns 1 if there is currently an active 
 * Bluetooth connection (as indicated by a logical 1 on the 
 * input pin connected to the HC-05 status line). Otherwise, 
 * it returns 0.
 */
int btConnected() {
    return GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1);
}


/**
 * @brief Interrupt handler for Bluetooth status changes
 * 
 * If you have previously called initBtState(), this handler will be
 * triggered when a Bluetooth connection is made or broken.
 */
void EXTI1_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line1) != RESET){
      EXTI_ClearITPendingBit(EXTI_Line1);
      //printf("Bluetooth status : %d\n",btConnected()); //Print BT status when it changes
    }
} 

void USART1_IRQHandler(void){

  // Check the Interrupt status to ensure the Rx interrupt was triggered, not Tx
  if( USART_GetITStatus(USART1, USART_IT_RXNE)){
 
    // Get the byte that was transferred in bluetooth_in
    bluetooth_in = USART1->DR;
	//printf("%c", bluetooth_in);
    
  }

}

void drive_forward(){
	/*set pwm pins 8-11 high and low alternatively to run both motors in same direction-forward */
	LED_Off(8);
	LED_On(9);
	LED_On(10);
	LED_Off(11);
	
	/* Using pin 12 and 15 to drive pwm to motors with 100% dutycycle*/
	set_LED_dutycycle(0,50); 
	set_LED_dutycycle(3,50);
}

void drive_reverse(){
	/*set pwm pins 8-11 high and low alternatively to run both motors in same direction-reverse */
	LED_On(8);
	LED_Off(9);
	LED_Off(10);
	LED_On(11);
	
	/* Using pin 12 and 15 to drive pwm to motors with 100% dutycycle */
	set_LED_dutycycle(0,50); 
	set_LED_dutycycle(3,50);
}

void turn_left(){
	/*set pwm pins 8-11 high and low to run left motor in reverse and right in forward- make left turn */
	set_LED_dutycycle(0,0); 
	set_LED_dutycycle(3,10);
			
	LED_On(8);
	LED_Off(9);
	LED_On(10);
	LED_Off(11);

	set_LED_dutycycle(0,5); 
	set_LED_dutycycle(3,5);
}

void turn_right(){
	/*set pwm pins 8-11 high and low to run left motor in forward and right in reverse- make right turn */
	set_LED_dutycycle(0,10); 
	set_LED_dutycycle(3,0);

	LED_Off(8);
	LED_On(9);
	LED_Off(10);
	LED_On(11);
	
	set_LED_dutycycle(0,5); 
	set_LED_dutycycle(3,5);
}

void halt(){
	LED_Off(8);
	LED_Off(9);
	LED_Off(10);
	LED_Off(11);
	set_LED_dutycycle(0,0); 
	set_LED_dutycycle(3,0);
}

int main(void)
{
  // initialize
  TM_HCSR04_t HCSR04;
  SystemInit();
  //initialise_monitor_handles(); 
  init_systick();
  init_LED_pins(); //Initialize LED pins 
  init_PWM_pins(); //Initialize PWM digital pins for direction control
  initBtState(); //Initialize Bluetooth session 
  init_USART(baudrate); //Initialize USART1 channel with baud 9600

  /* Check Sensor Status first. Sensor is not ready to use */
  /* Connectivity test */
  if(!TM_HCSR04_Init(&HCSR04, GPIOD, GPIO_PIN_0, GPIOC, GPIO_PIN_1))  //Echo PD0, TRIGGER PC1
	{
 
        while (1) 
		{
			toggle_led();
        }
    }
	
  while (1)
  {
	  
	/* Read distance from HCSR04 sensor */
    /* Distance is returned in cm and also stored in structure */
	Delayms(10);
    float distance = TM_HCSR04_Read(&HCSR04);	
	if(btConnected()){
    switch(bluetooth_in)
	{
		case 'w':
			if(distance < 15){
			//drive_reverse();
			halt();
			Delayms(50);
			turn_right();
			Delayms(175);
			//drive_forward();
			//Delayms(250);
			//turn_left();
			//Delayms(150);
			halt();
			}
			else 
			{
			drive_forward();
			} 
			break;

		case 's':
			drive_reverse();
			break;
			
		case 'a':
			turn_left();
			break;
			
		case 'd':
			turn_right();
			break;
			
		case 'z':
			halt();
			break;
		
	}
	}
	else{
		halt();
		bluetooth_in = "z";
	}
  }
}




