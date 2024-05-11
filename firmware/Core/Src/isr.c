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
uint8_t i2c_state = I2C_IDLE_STATE;
uint8_t uart_select  = 0x00;
uint8_t uart_expected = 0x00;

uint16_t uart1_todo = 0x00;
uint16_t uart2_todo = 0x00;

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

    DMA_Init();
}

void DMA_Init(){
    // Init DMA for I2C->UART
    SET_BIT(RCC->AHBENR,RCC_AHBENR_DMA1EN);				// Enable the clock

    DMAMUX1_Channel0->CCR = 51 << DMAMUX_CxCR_DMAREQ_ID_Pos;

    DMA1_Channel1->CPAR = (uint32_t)&(USART1->TDR);		// Set the destination (works)
    DMA1_Channel1->CMAR = (uint32_t)outStart_USART1;    // Set the start point of the buffer
    //DMA1_Channel1->CNDTR = 5; 							// Set the amount of data to transfer (but unknown at this time)
    DMA1_Channel1->CCR = DMA_CCR_MINC 		// Memory increment
    					| DMA_CCR_DIR 		// Read from memory
						| DMA_CCR_TCIE 		// Transfer complete interrupt enabled
						| DMA_CCR_TEIE;     // Transfer error interrupt enabled
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);		// Enable the irq

    //DMA1_Channel2->CNDTR = 10; 						// Set the amount of data to transfer (but unknown at this time)
    DMA1_Channel2->CPAR = (uint32_t)&(USART2->TDR);		// Set the destination
    DMA1_Channel2->CMAR = (uint32_t)outStart_USART2;    // Set the start point of the buffer
    DMA1_Channel2->CCR  = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_TEIE;
    /*
     * CCR bit set because
     * DMA_CCR_MINC -> Memory increment mode
     * DMA_CCR_DIR  -> Read from memory
     * DMA_CCR_TCIE -> Transfer complete interrupt enable
     *
     * MSIZE/PSIZE  -> Remain at default of 8bits
     * CIRCM/PINC 		-> Remain at default not enabled
     *
     */
}
void USART1_IRQHandler(void){

    uint8_t ok = 0x01;

    if( USART1->ISR & USART_ISR_TC ){
    	USART1->ICR |= USART_ICR_TCCF; /* Clear transfer complete flag */
    }
    if((USART1->ISR & USART_ISR_RXNE_RXFNE) == USART_ISR_RXNE_RXFNE){ // ISR for received data
    	uint8_t recChar = (uint8_t)(USART1->RDR); /* Receive data, clear flag */
    }
    // Check if interrupt is due to transmit complete
   /* if( USART1->ISR & USART_ISR_TXE_TXFNF ){
    	if( outStart_USART1 != outEnd_USART1 ){ // meaning still data to send
			 USART1->TDR = *outStart_USART1++; // This also clears the complete flag
			 uart1_recs--;
			 if (outStart_USART1 == outTail_USART1) { // So never write on the tail!
				 outStart_USART1 = outHead_USART1;    // End reached, back to head
			 }
		 }
    }*/
    if( ok==0x00 ){ // Meaning ISR was for unknown reason
        isr_error = ERROR_USART_TRANSMIT; /* Report an error */
        NVIC_DisableIRQ(USART1_IRQn); /* Disable USART1_IRQn */
    }
}

void USART2_IRQHandler(void){

    uint8_t ok = 0x01;


    if( USART2->ISR & USART_ISR_TC ){
    	USART2->ICR = USART_ICR_TCCF; /* Clear transfer complete flag */
    }
    if((USART2->ISR & USART_ISR_RXNE_RXFNE) == USART_ISR_RXNE_RXFNE){ // ISR for received data
    	uint8_t recChar = (uint8_t)(USART2->RDR); /* Receive data, clear flag */
    }
    // Check if interrupt is due to transmit complete
    if( USART2->ISR & USART_ISR_TXE_TXFNF ){
    	/*if( outStart_USART2 != outEnd_USART2 ){ // meaning still data to send
			 USART2->TDR = *outStart_USART2++; // This also clears the complete flag
			 if (outStart_USART2 == outTail_USART2) { // So never write on the tail!
				 outStart_USART2 = outHead_USART2;    // End reached, back to head
			 }
		 }*/
    }
    if( ok==0x00 ){ // Meaning ISR was for unknown reason
    	isr_error = ERROR_USART_TRANSMIT; /* Report an error */
        NVIC_DisableIRQ(USART2_IRQn); /* Disable USART2_IRQn */
    }
}
void DMA1_Channel1_IRQHandler(void) {  // now it does nothing only clears the flag
	DMA1 -> IFCR |= DMA_IFCR_CHTIF1;
	if(DMA1 -> ISR & (DMA_ISR_TCIF1)) {  // Transfer complete for channel 1
        DMA1 -> IFCR |= DMA_IFCR_CTCIF1; // Clear the transfer complete flag
        CLEAR_BIT(DMA1_Channel1->CCR,DMA_CCR_EN); // Finished, so disable it

        /* Check if a restart is needed */
        if( outStart_USART1 == outEnd_USART1) // No new data added, nothing to do (or did a full lap...)
        	return; // So return

        uint16_t tosend;
        DMA1_Channel1->CMAR = (uint32_t)outStart_USART1; //Update the startpoint

        if( outStart_USART1 > outEnd_USART1 ){ // Meaning new data circled around
        	tosend = outTail_USART1 - outStart_USART1-1;
        	outStart_USART1 = outHead_USART1; // And update the start to the head
        	uart1_todo -= tosend; // Subtract this amount from the amount to send
        }else{ // Meaning new data but not circled around, so send remainder
        	tosend = uart1_todo;
        	uart1_todo = 0; // All data is scheduled to be send, so clear this
        	outStart_USART1 += tosend; // And alter the start accordingly
        }
        DMA1_Channel1->CNDTR = tosend; // So schedule all data received
        SET_BIT(DMA1_Channel1->CCR,DMA_CCR_EN); // Start it again
    }

}
void I2C1_IRQHandler(void){
	  uint8_t temp;

	  if(I2C1->ISR & I2C_ISR_RXNE){ // Receive buffer not empty
		  switch(i2c_state){
		  	  case I2C_REC_DATA_STATE:// Put this first because it's called most often?
					if( uart_select == 0x01 ){ // Comms selected UART1
						*outEnd_USART1++ = I2C1->RXDR;
						if (outEnd_USART1 == outTail_USART1) { // Wrap around
							outEnd_USART1 = outHead_USART1;
						}
						// Now that at least one byte is in the buffer
						// Check if DMA is active, if not activate it
						if( !(DMA1_Channel1->CCR & DMA_CCR_EN) ){
							DMA1_Channel1->CCR |= DMA_CCR_EN;
						}
					}else if( uart_select == 0x02 ){ // Comms selected uart2
						*outEnd_USART2++ = I2C1->RXDR;
						if (outEnd_USART2 == outTail_USART2) { // Wrap around
							outEnd_USART2 = outHead_USART2;
						}
						// Now that atleast one byte is in the buffer
						// Check if DMA is active, if not activate it
						if( !(DMA1_Channel2->CCR & DMA_CCR_EN) ){
							DMA1_Channel2->CCR |= DMA_CCR_EN;
						}
					}else{
						temp = I2C1->RXDR; // Receive the data
					}
					break;
		  	  case I2C_IDLE_STATE:
		  		  	 if( I2C1->RXDR == 1){// Receive the data
		  		  		 i2c_state = I2C_REC_SIZE_STATE;
		  		  	 }else{
		  		  		i2c_state = I2C_CONF_STATE;
		  		  	 }
		  		  	 break;
		  	  case I2C_REC_SIZE_STATE:
		  		  // First check if it's for uart1 or 2
		  		if( uart_select == 0x01 ){
		  			  // Check if DMA is already active
					  if( DMA1_Channel1->CCR & DMA_CCR_EN ){ // It's already active
						uart1_todo += I2C1->RXDR; // Keep track of the amount of data the DMA isn't aware of
					  }else{  // DMA isn't active, so set it up
						  uart_expected	= I2C1->RXDR;
						  if( outStart_USART1 + uart_expected >= outTail_USART1 ){ // If it would go beyond the tail, limit it
							  uint16_t todo = outTail_USART1 - outStart_USART1;
							  DMA1_Channel1->CNDTR = todo; // So schedule the part till the tail
							  uart1_todo += uart_expected - todo; // And add the remainder to the todo
							  outStart_USART1=outHead_USART1; // This means the start will end up at the head
						  }else{ // Still below the tail so schedule the full message
							  DMA1_Channel1->CNDTR = uart_expected; // all data scheduled
							  outStart_USART1 += uart_expected; // Move start to expected end position
						  }
					  }
		  		}else{
					if( DMA1_Channel2->CCR & DMA_CCR_EN ){ // It's already active
						uart2_todo += I2C1->RXDR; // Keep track of the amount of data the DMA isn't aware of
					}else{  // DMA isn't active, so set it up
						uart_expected	= I2C1->RXDR;
						if( outStart_USART2 + uart_expected > outTail_USART2 ){ // If it would go beyond the tail, limit it
							DMA1_Channel2->CNDTR = outTail_USART2 - outStart_USART2;
							 outStart_USART2=outHead_USART2;
						}else{ // If not, schedule the full message
							DMA1_Channel2->CNDTR = uart_expected;
							outStart_USART2 += uart_expected;
						}
					}
		  		}
		  		i2c_state = I2C_REC_DATA_STATE;
		  		break;
		  	  default:
		  		  temp = I2C1->RXDR; // Unknown state, just read it to clear the flag
		  		  break;
		  }
	  }else if((I2C1->ISR & I2C_ISR_ADDR) == I2C_ISR_ADDR){
		  I2C1->ICR |= I2C_ICR_ADDRCF; /* Clear address match flag */

		  /* Figure out if it's the first or second port */
		  // And with the used bits for addcode, then shift it all to a byte instead
		  uint8_t match = (I2C1->ISR & I2C_ISR_ADDCODE)>>17;
		  if( match == 0x1B ){
			  uart_select = 0x01;
		  }else if( match == 0x2B){
			  uart_select = 0x02;
		  }
		  i2c_state = I2C_IDLE_STATE; // Beginning of new comms

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
	  }else if(I2C1->ISR & I2C_ISR_ARLO ){   // Arbitration lost
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
