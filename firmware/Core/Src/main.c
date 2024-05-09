/*******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"

/* Shared Variables  ---------------------------------------------------------*/
__IO uint32_t Tick;
__IO uint16_t error;
/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
uint8_t i2c_recBuffer[64];
uint8_t i2c_cnt=0x00;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


/* Private user code ---------------------------------------------------------*/


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){

  /* MCU Configuration--------------------------------------------------------*/

	/* Configure the system clock */
	SysTick_Config(2000); /* 1ms config */
    SystemClock_Config();
    SysTick_Config(16000); /* 1ms config */

  /* Initialize all configured peripherals */
    init();

  /* Infinite loop */
  while (1){

  }

}

/**
  * Brief   This function configures the system clock @16MHz and voltage scale 1
  *         assuming the registers have their reset value before the call.
  *         POWER SCALE   = RANGE 1
  *         SYSTEM CLOCK  = PLL MUL8 DIV2
  *         PLL SOURCE    = HSI/4
  *         FLASH LATENCY = 0
  * Param   None
  * Retval  None
  */
__INLINE void SystemClock_Config(void){

  uint32_t tickstart;
  
  /* (1) Enable power interface clock */
  RCC->APBENR1 |= (RCC_APBENR1_PWREN);
  
  
  /* (2) Enable HSI divided by 4 in RCC->CR, so 48MHz/4=12MHz*/
  RCC->CR |= RCC_CR_HSION | RCC_CR_HSIDIV_1;
  /*
   * 000: 1	  -> 48MHz
   * 001: 2   -> 24MHz
   * 010: 4   -> 12MHz (def)
   * 011: 8   ->  6MHz
   * 100: 16  ->  3MHz
   * 101: 32  ->  1.5MHz
   * 110: 64  ->    750kHz
   * 111: 128 ->    375kHz
   */
  tickstart = Tick;
    
  /* (3) Wait for HSI ready flag and HSIDIV flag */
   while ((RCC->CR & RCC_CR_HSIRDY ) != (RCC_CR_HSIRDY)){
    if ((Tick - tickstart ) > HSI_TIMEOUT_VALUE){
      error = ERROR_HSI_TIMEOUT; /* Report an error */
      return;
    }      
  } 

  tickstart = Tick;

}
/******************************************************************************/
/*            Cortex-M0 Plus Processor Exceptions Handlers                    */
/******************************************************************************/

/* This function handles SysTick Handler.  */
void SysTick_Handler(void){
    Tick++;

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1){
  }
  /* USER CODE END Error_Handler_Debug */
}
/* ********************************************************************************************************** */
/* *********************************** A P P L I C A T I O N  C O D E *************************************** */
/* ********************************************************************************************************** */
void init(void){

    configure_IO();
    ISR_Init();

    I2C1_Configure_Slave();
    USART1_Configure();
    USART2_Configure();
}

void configure_IO(void){
	/* Enable the SYStemConfiguration peripheral clock, this handles interrupts */
	/* THIS NEEDS TO GO FIRST */
    RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;

	/* The I2C pins are on the PA9 and PA10 that are from remapped on PA11 and PA12 */
	SYSCFG->CFGR1 |= (SYSCFG_CFGR1_PA11_RMP|SYSCFG_CFGR1_PA12_RMP);

	/* Enable the peripheral clock of GPIOA (inputs) and GPIOB (heartbeat) */
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

    /* Finally enable the interrupt */
    NVIC_SetPriority(EXTI4_15_IRQn, 0x03); // This is for pins between 4 and 15 which 6 and 7 belong to, set to lowest priority
    NVIC_EnableIRQ(EXTI4_15_IRQn); // Actually enable it

}
/* *************************************** I R Q **************************************************** */

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
