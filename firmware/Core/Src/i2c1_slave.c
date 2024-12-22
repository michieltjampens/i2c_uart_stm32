/**
 * Class with methods to use the I2C1 peripheral on the STM32C0x device
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

  /* GPIO Setup, PB6=SCL and PB7=SDA */
  SET_BIT(RCC->IOPENR,RCC_IOPENR_GPIOBEN);  // Enable the peripheral clock of GPIOB

  /* Change PA9 and PA10 mode to use alternate mode */
  MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE6|GPIO_MODER_MODE7, GPIO_MODER_MODE6_1|GPIO_MODER_MODE7_1);

  /* First clear the AF setting for pin 6 and 7
   * -> This is in AFR[0] because pin at most  7
   * -> Start with clearing the AF for those pins by doing a reverse and
   * -> Then set AF bits to AF6 (0110 or 0x06)
   */
  MODIFY_REG( GPIOB->AFR[0], GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7, (6 << GPIO_AFRL_AFSEL6_Pos) | ( 6 << GPIO_AFRL_AFSEL7_Pos) );
  /* I2C needs to be open drain */
  SET_BIT( GPIOB->OTYPER, GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7 );

  /* Configure I2C1 as slave */
  I2C1->CR1 = I2C_CR1_ADDRIE; /* Address match interrupt enable */
  I2C1->OAR1 |= (uint32_t)(I2C1_OWN_ADDRESS1 << 1); /* 7-bit address (see .h) */
  SET_BIT( I2C1->OAR1, I2C_OAR1_OA1EN ); /* Enable own address 1 */
  I2C1->OAR2 |= (uint32_t)(I2C1_OWN_ADDRESS2 << 1); /* 7-bit address (see .h) */
  SET_BIT( I2C1->OAR2, I2C_OAR2_OA2EN ); /* Enable own address 2 */
  SET_BIT(I2C1->CR1,I2C_CR1_RXIE | I2C_CR1_TXIE); // Enable Receive and transmit interrupt

  /* Configure Interrupts */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  SET_BIT(I2C1->CR1,I2C_CR1_PE); 		// Enable I2C peripheral
  while((I2C1->CR1 & I2C_CR1_PE) == 0); // Wait for it to be enabled
}

