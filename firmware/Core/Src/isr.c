/*
 * isr.c
 *
 *  Created on: May 9, 2024
 *      Author: Michiel
 */
#include "isr.h"

/* Circular buffer for UART1 OUT*/
uint8_t outputBuffer_USART1[CIRCULAR];
uint8_t *outTemp_USART1;
uint8_t *outStart_USART1;
uint8_t *outEnd_USART1;
uint8_t *outHead_USART1;
uint8_t *outTail_USART1;

/* Circular buffer for UART1 IN*/
uint8_t inputBuffer_USART1[CIRCULAR];
uint8_t *inputTemp_USART1;
uint8_t *inputStart_USART1;
uint8_t *inputEnd_USART1;
uint8_t *inputHead_USART1;
uint8_t *inputTail_USART1;

/* Circular buffer for UART2 OUT*/
uint8_t outputBuffer_USART2[CIRCULAR];
uint8_t *outTemp_USART2;
uint8_t *outStart_USART2;
uint8_t *outEnd_USART2;
uint8_t *outHead_USART2;
uint8_t *outTail_USART2;

/* Circular buffer for UART2 IN*/
uint8_t inputBuffer_USART2[CIRCULAR];
uint8_t *inputTemp_USART2;
uint8_t *inputStart_USART2;
uint8_t *inputEnd_USART2;
uint8_t *inputHead_USART2;
uint8_t *inputTail_USART2;

uint8_t isr_error;

void ISR_Init(){
    /* Initialise the circular buffers */
    inputStart_USART1 = &inputBuffer_USART1[0];
    inputEnd_USART1 =   &inputBuffer_USART1[0];
    inputHead_USART1 =  &inputBuffer_USART1[0];
    inputTail_USART1 =  &inputBuffer_USART1[CIRCULAR-1];

    outStart_USART1 = &outputBuffer_USART1[0];
    outEnd_USART1  =  &outputBuffer_USART1[0];
    outHead_USART1 =  &outputBuffer_USART1[0];
    outTail_USART1 =  &outputBuffer_USART1[CIRCULAR-1];

    inputStart_USART2 = &inputBuffer_USART2[0];
    inputEnd_USART2 =   &inputBuffer_USART2[0];
    inputHead_USART2 =  &inputBuffer_USART2[0];
    inputTail_USART2 =  &inputBuffer_USART2[CIRCULAR-1];

    outStart_USART2 = &outputBuffer_USART2[0];
    outEnd_USART2  =  &outputBuffer_USART2[0];
    outHead_USART2 =  &outputBuffer_USART2[0];
    outTail_USART2 =  &outputBuffer_USART2[CIRCULAR-1];
}

void USART1_IRQHandler(void){
    uint8_t recChar = 0;
    uint8_t ok = 0x01;

    // Check if interrupt is due to transmit complete
    if( USART1->ISR & USART_ISR_TXE_TXFNF ){
    	if( outStart_USART1 != outEnd_USART1 ){ // meaning still data to send
			 USART1->TDR = *outStart_USART1++; // This also clears the complete flag
			 if (outStart_USART1 == outTail_USART1) { // So never write on the tail!
				 outStart_USART1 = outHead_USART1;    // End reached, back to head
			 }
		 }else{ // No more data to send, just clear the flag
			 USART1->ICR = USART_ICR_TCCF; /* Clear transfer complete flag */
		 }
    }
    if( USART1->ISR & USART_ISR_TC ){
    	USART1->ICR = USART_ICR_TCCF; /* Clear transfer complete flag */
    }
    if((USART1->ISR & USART_ISR_RXNE_RXFNE) == USART_ISR_RXNE_RXFNE){ // ISR for received data
        recChar = (uint8_t)(USART1->RDR); /* Receive data, clear flag */
        if( recChar >= 0x20 && recChar <= 0x7F){
            *inputEnd_USART1++ = recChar;
            if (inputEnd_USART1 == inputTail_USART1) { // So never write on the tail!
                inputEnd_USART1 = inputHead_USART1;
            }
        }
        // Can't echo inside the ISR! because the processor doesn't process fast enough?
        //USART1_SendByte(recChar); // This gives issues
    }
    if( ok==0x00 ){ // Meaning ISR was for unknown reason
        isr_error = ERROR_USART_TRANSMIT; /* Report an error */
        NVIC_DisableIRQ(USART1_IRQn); /* Disable USART1_IRQn */
    }
}

void USART2_IRQHandler(void){
    uint8_t recChar = 0;
    uint8_t ok = 0x01;

    // Check if interrupt is due to transmit complete
    if( USART2->ISR & USART_ISR_TXE_TXFNF ){
    	if( outStart_USART2 != outEnd_USART2 ){ // meaning still data to send
			 USART2->TDR = *outStart_USART2++; // This also clears the complete flag

			 if (outStart_USART2 == outTail_USART2) { // So never write on the tail!
				 outStart_USART2 = outHead_USART2;    // End reached, back to head
			 }
		 }else{ // No more data to send, just clear the flag
			 USART2->ICR = USART_ICR_TCCF; /* Clear transfer complete flag */
		 }
    }
    if( USART2->ISR & USART_ISR_TC ){
    	USART2->ICR = USART_ICR_TCCF; /* Clear transfer complete flag */
    }
    if((USART2->ISR & USART_ISR_RXNE_RXFNE) == USART_ISR_RXNE_RXFNE){ // ISR for received data
        recChar = (uint8_t)(USART2->RDR); /* Receive data, clear flag */
        if( recChar >= 0x20 && recChar <= 0x7F){
            *inputEnd_USART2++ = recChar;
            if (inputEnd_USART2 == inputTail_USART2) { // So never write on the tail!
                inputEnd_USART2 = inputHead_USART2;
            }
        }else if(recChar==0x00){
        	ok=0x00;
        }else if( recChar == '\n'){
        }else if( recChar == '\r'){

        }
        // Can't echo inside the ISR! because the processor doesn't process fast enough?
        //USART2_SendByte(recChar); // This gives issues
    }
    if( ok==0x00 ){ // Meaning ISR was for unknown reason
    	isr_error = ERROR_USART_TRANSMIT; /* Report an error */
        NVIC_DisableIRQ(USART2_IRQn); /* Disable USART2_IRQn */
    }
}

void I2C1_IRQHandler(void){

	  if(I2C1->ISR & I2C_ISR_RXNE){ // Receive buffer not empty
		 // i2c_recBuffer[i2c_cnt] = I2C1->RXDR;

		  // Do something?
	  }else if((I2C1->ISR & I2C_ISR_ADDR) == I2C_ISR_ADDR){
		  I2C1->ICR |= I2C_ICR_ADDRCF; /* Clear address match flag */

		  /* Figure out if it's the first or second port */
		  uint8_t match = (I2C1->ISR & I2C_ISR_ADDCODE)/0x10000;

		  /* Check if transfer direction is read (slave transmitter) */
		  if((I2C1->ISR & I2C_ISR_DIR) == I2C_ISR_DIR){
			  I2C1->CR1 |= I2C_CR1_TXIE; /* Set transmit IT */
		  }
	  }else if((I2C1->ISR & I2C_ISR_TXIS) == I2C_ISR_TXIS){
	  		  I2C1->CR1 &=~ I2C_CR1_TXIE; /* Disable transmit IT */
	  }else if(I2C1->ISR & I2C_ISR_NACKF){ /* NACK Received*/
		  SET_BIT(I2C1->ICR, I2C_ICR_NACKCF); // Clear flag
	  }else if(I2C1->ISR & I2C_ISR_STOPF){
		  SET_BIT(I2C1->ICR, I2C_ICR_STOPCF); // Clear flag
	  }else if(I2C1->ISR & I2C_ISR_BERR ){ // misplaced Start or STOP condition
		  SET_BIT(I2C1->ICR, I2C_ICR_BERRCF);
		  isr_error=ERROR_I2C_BERR;
	  }else if(I2C1->ISR & I2C_ISR_ARLO ){ // Arbitration lost
		  SET_BIT(I2C1->ICR, I2C_ICR_ARLOCF);
		  isr_error=ERROR_I2C_ARLO;
	  }else if(I2C1->ISR & I2C_ISR_PECERR){ // PEC Error in reception
		  SET_BIT(I2C1->ICR, I2C_ICR_PECCF);
		  isr_error=ERROR_I2C_PECERR;
	  }else if(I2C1->ISR & I2C_ISR_TIMEOUT){  // Timeout or tLOW detection flag
		  SET_BIT(I2C1->ICR, I2C_ICR_TIMOUTCF);
		  isr_error=ERROR_I2C_TIMEOUT;
		  /* Check address match */
	  }else{
		  isr_error = ERROR_I2C; /* Report an error */
	    NVIC_DisableIRQ(I2C1_IRQn); /* Disable I2C2_IRQn */
	  }
}
