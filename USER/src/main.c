#include "stm32f4xx.h"
#include "main.h"
#include "led_init.h"
#include "rcc_init.h"
#include "delay.h"

int main(void)
{

	RCC_Init();
	LED_Init();

	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	
	while (1)
	{
		BLUE_LED_Off();
		GREEN_LED_On();
			Delay_mS(TIMER);
		GREEN_LED_Off();
		ORANGE_LED_On();
			Delay_mS(TIMER);
		ORANGE_LED_Off();
		RED_LED_On();
			Delay_mS(TIMER);
		RED_LED_Off();
		BLUE_LED_On();
			Delay_mS(TIMER);
	}
	
}


