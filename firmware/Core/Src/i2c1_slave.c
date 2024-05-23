/**
 * Class with methods to use the I2C1 peripheral on the STM32L0x0 device
 */
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "i2c1_slave.h"

/* Private variables ---------------------------------------------------------*/



/* *********************************** S E T U P *************************************************** */
/**
 * Brief Init the I2C1 peripheral and the used GPIO's
 */
__INLINE void I2C1_Configure_Slave(void){

  SET_BIT(RCC->APBENR1,RCC_APBENR1_I2C1EN); // Enable the peripheral clock I2C1 do this before GPIO's (?)

  /* GPIO Setup, PA9=SCL and PA10=SDA */
  SET_BIT(RCC->IOPENR,RCC_IOPENR_GPIOAEN);  // Enable the peripheral clock of GPIOA

  /* Change PA9 and PA10 mode to use alternate mode */
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE9|GPIO_MODER_MODE10, GPIO_MODER_MODE9_1|GPIO_MODER_MODE10_1);

  /* First clear the AF setting for pin 9 and 10
   * -> This is in AFR[1] because pin number above 7
   * -> Start with clearing the AF for those pins by doing a reverse and
   * -> Then set AF bits to AF6 (0110 or 0x06)
   */
  GPIOA->AFR[1] = (GPIOA->AFR[1] &~ (GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10)) | (GPIO_AFRH_AFSEL9_1|GPIO_AFRH_AFSEL9_2|GPIO_AFRH_AFSEL10_1|GPIO_AFRH_AFSEL10_2);

  /* I2C needs to be open drain */
  GPIOA->OTYPER |= GPIO_OTYPER_OT9 | GPIO_OTYPER_OT10;

  /* Configure I2C1 as slave */
  I2C1->CR1 = I2C_CR1_ADDRIE; /* Address match interrupt enable */
  I2C1->OAR1 |= (uint32_t)(I2C1_OWN_ADDRESS1 << 1); /* 7-bit address (see .h) */
  I2C1->OAR1 |= I2C_OAR1_OA1EN; /* Enable own address 1 */
  I2C1->OAR2 |= (uint32_t)(I2C1_OWN_ADDRESS2 << 1); /* 7-bit address (see .h) */
  I2C1->OAR2 |= I2C_OAR2_OA2EN; /* Enable own address 2 */
  SET_BIT(I2C1->CR1,I2C_CR1_RXIE | I2C_CR1_TXIE); // Enable Receive and transmit interrupt

  /* Configure Interrupts */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  SET_BIT(I2C1->CR1,I2C_CR1_PE); 		// Enable I2C peripheral
  while((I2C1->CR1 & I2C_CR1_PE) == 0); // Wait for it to be enabled
}

