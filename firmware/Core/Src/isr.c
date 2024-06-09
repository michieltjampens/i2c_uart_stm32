/*
 * isr.c
 *
 *  Created on: May 9, 2024
 *      Author: Michiel
 */
#include "isr.h"

/* Circular buffers for I2C to UART */
uint8_t I2C_to_USART1[CIRCULAR];
uint8_t *outDMA_USART1;
uint8_t *inI2C_USART1;
uint8_t *outHead_USART1;
uint8_t *outTail_USART1;
uint16_t uart1_wait=0;			// Count amount of data received through i2c for uart2
uint16_t uart1_todo = 0x00;		// Keep track of amount of bytes to receive and pass onwards

uint8_t I2C_to_USART2[CIRCULAR];
uint8_t *outDMA_USART2;
uint8_t *inI2C_USART2;
uint8_t *outHead_USART2;
uint8_t *outTail_USART2;
uint16_t uart2_wait=0;			// Count amount of data received through i2c for uart2
uint16_t uart2_todo = 0x00;		// Keep track of amount of bytes to receive and pass onwards

/* Circular buffer for UART to I2C*/
uint8_t  USART1_to_I2C[CIRCULAR];
uint8_t *ui_read_USART1;
uint8_t *ui_write_USART1;
uint8_t *ui_head_USART1;
uint8_t *ui_tail_USART1;

uint8_t USART2_to_I2C[CIRCULAR];
uint8_t *ui_read_USART2;
uint8_t *ui_write_USART2;
uint8_t *ui_head_USART2;
uint8_t *ui_tail_USART2;

/* Other */
uint8_t isr_error;					// Last error store
uint8_t i2c_state = I2C_IDLE_STATE; // State in the i2c data receive flow
uint8_t uart_select  = 0x00;		// UART that is target of the active i2c comms
uint8_t uart_expected = 0x00;		// Amount of data that is expected via i2c

uint8_t USART1_state_req=0;
uint8_t USART2_state_req=0;


uint16_t USART1_cnt=0x8000; // First bit is enable
uint16_t USART2_cnt=0x8000;
uint16_t tempState=0x00;

/* *************************************************************************************** */
/* ******************************* I N I T *********************************************** */
/* *************************************************************************************** */
void ISR_Init(){
    /* Initialise the circular buffers */
    ui_read_USART1  = &USART1_to_I2C[0];
    ui_write_USART1 = &USART1_to_I2C[0];
    ui_head_USART1  = &USART1_to_I2C[0];
    ui_tail_USART1  = &USART1_to_I2C[CIRCULAR-1];

    outDMA_USART1  = &I2C_to_USART1[0];
    inI2C_USART1   = &I2C_to_USART1[0];
    outHead_USART1 = &I2C_to_USART1[0];
    outTail_USART1 = &I2C_to_USART1[CIRCULAR-1];

    ui_read_USART2  = &USART2_to_I2C[0];
    ui_write_USART2 = &USART2_to_I2C[0];
    ui_head_USART2  = &USART2_to_I2C[0];
    ui_tail_USART2  = &USART2_to_I2C[CIRCULAR-1];

    outDMA_USART2  = &I2C_to_USART2[0];
    inI2C_USART2   = &I2C_to_USART2[0];
    outHead_USART2 = &I2C_to_USART2[0];
    outTail_USART2 = &I2C_to_USART2[CIRCULAR-1];

    USART_DMA_Init();
}
void USART_DMA_Init(){
    // Init DMA for I2C->UART
    SET_BIT(RCC->AHBENR,RCC_AHBENR_DMA1EN);				// Enable the clock

    /* USART 1 */
    DMAMUX1_Channel0->CCR = 51; // usart1_tx_dma
    DMA1_Channel1->CPAR = (uint32_t)&(USART1->TDR);		// Set the destination (works)
    DMA1_Channel1->CMAR = (uint32_t)outDMA_USART1;    // Set the start point of the buffer
    DMA1_Channel1->CCR = DMA_CCR_MINC 		// Memory increment
    					| DMA_CCR_DIR 		// Read from memory
						| DMA_CCR_TCIE 		// Transfer complete interrupt enabled
						| DMA_CCR_TEIE;     // Transfer error interrupt enabled
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);		// Enable the irq

    /* USART 2 */
    DMAMUX1_Channel1->CCR = 53; // usart2_tx_dma
    DMA1_Channel2->CPAR = (uint32_t)&(USART2->TDR);		// Set the destination
    DMA1_Channel2->CMAR = (uint32_t)outDMA_USART2;    // Set the start point of the buffer
    DMA1_Channel2->CCR  = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_TEIE;
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);			// Enable the irq
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
/* *************************************************************************************** */
/* ******************************* C H E C K S******************************************** */
/* *************************************************************************************** */
void Check_usart1_out(){
	// If DMA is active, nothing to do so return
	if( DMA1_Channel1->CCR & DMA_CCR_EN )
		return;

	// If there isn't any new data in the buffer, nothing do so so return
	if( outDMA_USART1 == inI2C_USART1 )
		return;

	/* If we get here, it means the DMA isn't active but there's at least one byte in the buffer */
	uint16_t todo = uart1_todo; // Make a backup because isr might overwrite this

	// Find the adres of the last byte, which will be the next start
	uint8_t *newStart_USART1 = outDMA_USART1 + todo;

	// Check if we go beyond or on the tail
	if( newStart_USART1 >= outTail_USART1 ){ // We go beyond, so only start the part till the tail
		todo = outTail_USART1-outDMA_USART1; // Update the start to where the dma will end
		newStart_USART1 = outHead_USART1;    // Update the start to beginning of buffer
	}
	DMA1_Channel1->CNDTR = todo;
	outDMA_USART1 = newStart_USART1; // Update the start to where the dma will end
	uart1_todo -= todo; 			 // substract the portion from the total

	if( todo > 1 ){
		while( uart1_wait <= 1 ); // Wait for another byte, but this somehow blocks?
	}
	SET_BIT(DMA1_Channel1->CCR,DMA_CCR_EN); // Start it again
}
void Check_usart2_out(){
	// If DMA is active, nothing to do so return
	if( DMA1_Channel2->CCR & DMA_CCR_EN )
		return;

	// If there isn't any new data in the buffer, nothing do so so return
	if( outDMA_USART2 == inI2C_USART2 )
		return;

	/* If we get here, it means the DMA isn't active but there's at least one byte in the buffer */
	uint16_t todo = uart2_todo; // Make a backup because isr might overwrite this

	// Find the adres of the last byte, which will be the next start
	uint8_t *newStart_USART2 = outDMA_USART2 + todo;

	// Check if we go beyond or on the tail
	if( newStart_USART2 >= outTail_USART2 ){ // We go beyond, so only start the part till the tail
		todo = outTail_USART2-outDMA_USART2; // Update the start to where the dma will end
		newStart_USART2 = outHead_USART2;    // Update the start to beginning of buffer
	}
	DMA1_Channel2->CNDTR = todo;
	outDMA_USART2 = newStart_USART2; // Update the start to where the dma will end
	uart2_todo -= todo; 			 // substract the portion from the total

	if( todo > 1 ){
		while( uart2_wait <= 1 ); // Wait for another byte, but this somehow blocks?
	}
	SET_BIT(DMA1_Channel2->CCR,DMA_CCR_EN); // Start it again
}
/* *************************************************************************************** */
/* ******************************* U S A R T ********************************************* */
/* *************************************************************************************** */
void USART1_IRQHandler(void){

    uint8_t ok = 0x01;

    if( USART1->ISR & USART_ISR_TC ){
    	USART1->ICR |= USART_ICR_TCCF; /* Clear transfer complete flag */
    }
    /*
     * It can happen that during startup, noise is on the rx which can be seen as data.
     * If this happens both Framing error and Receive not empty are set.
     * So before processing received data, check FE first and clear RX flag if FE is set.
     * If it isn't, process it.
     */
    if((USART1->ISR & USART_ISR_FE) == USART_ISR_FE){
    	USART1->ICR |= USART_ICR_FECF; /* Clear frame error flag */
    	USART1->RQR |= USART_RQR_RXFRQ; /* Clear frame receive flag */
    }else if((USART1->ISR & USART_ISR_RXNE_RXFNE) == USART_ISR_RXNE_RXFNE){ // ISR for received data
    	uint8_t recChar = (uint8_t)(USART1->RDR); /* Receive data, clear flag */
    	*ui_write_USART1++ = recChar;
    	USART1_cnt++;
		if (ui_write_USART1 == ui_tail_USART1) // So never write on the tail!
			ui_write_USART1 = ui_head_USART1;  // End reached, back to head
    }
    if( USART1->ISR & USART_ISR_IDLE ){
    	USART1->ICR |= USART_ICR_IDLECF; /* Clear idle flag */
    	if( ui_read_USART1!=ui_write_USART1 ) // only trigger if there's data
    		TIM3->CR1 |= TIM_CR1_CEN; // Trigger irq
    }
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
    /*
     * It can happen that during startup, noise is on the rx which can be seen as data.
     * If this happens both Framing error and Receive not empty are set.
     * So before processing received data, check FE first and clear RX flag if FE is set.
     * If it isn't, process it.
     */
    if((USART2->ISR & USART_ISR_FE) == USART_ISR_FE){
    	USART2->ICR |= USART_ICR_FECF; /* Clear frame error flag */
    	USART2->RQR |= USART_RQR_RXFRQ; /* Clear frame receive flag */
    }else if((USART2->ISR & USART_ISR_RXNE_RXFNE) == USART_ISR_RXNE_RXFNE){ // ISR for received data
    	uint8_t recChar = (uint8_t)(USART2->RDR); /* Receive data, clear flag */
    	*ui_write_USART2++ = recChar;
    	USART2_cnt++;
		if (ui_write_USART2 == ui_tail_USART2) // So never write on the tail!
			ui_write_USART2 = ui_head_USART2;  // End reached, back to head
    }
    // The receiver is idle
    if( USART2->ISR & USART_ISR_IDLE ){
    	USART2->ICR |= USART_ICR_IDLECF; /* Clear idle flag */
    	if( ui_read_USART2!=ui_write_USART2 ) // only trigger if there's data
    		TIM3->CR1 |= TIM_CR1_CEN; // Trigger irq
    }
    if( ok==0x00 ){ // Meaning ISR was for unknown reason
    	isr_error = ERROR_USART_TRANSMIT; /* Report an error */
        NVIC_DisableIRQ(USART2_IRQn); /* Disable USART2_IRQn */
    }
}
/* *************************************************************************************** */
/* ******************************* D M A ************************************************* */
/* *************************************************************************************** */
void DMA1_Channel1_IRQHandler(void) {  // now it does nothing only clears the flag
	DMA1 -> IFCR |= DMA_IFCR_CHTIF1;
	if(DMA1 -> ISR & (DMA_ISR_TCIF1)) {  // Transfer complete for channel 1
        DMA1 -> IFCR |= DMA_IFCR_CTCIF1; // Clear the transfer complete flag
        CLEAR_BIT(DMA1_Channel1->CCR,DMA_CCR_EN); // Finished, so disable it
    	// Update the dma start position
    	DMA1_Channel1->CMAR = (uint32_t)outDMA_USART1;
    }
}
void DMA1_Channel2_3_IRQHandler(void) {  // now it does nothing only clears the flag
	DMA1 -> IFCR |= DMA_IFCR_CHTIF2;
	if(DMA1 -> ISR & (DMA_ISR_TCIF2)) {  // Transfer complete for channel 2
        DMA1 -> IFCR |= DMA_IFCR_CTCIF2; // Clear the transfer complete flag
        CLEAR_BIT(DMA1_Channel2->CCR,DMA_CCR_EN); // Finished, so disable it
    	// Update the dma start position
    	DMA1_Channel2->CMAR = (uint32_t)outDMA_USART2;
    }
}
/* *************************************************************************************** */
/* ******************************* I 2 C ************************************************* */
/* *************************************************************************************** */
void I2C1_IRQHandler(void){

	  if(I2C1->ISR & I2C_ISR_RXNE){ // Receive buffer not empty
		  uint8_t temp = I2C1->RXDR; // Read it directly so RXNE is cleared

		  switch(i2c_state){
		  	  case I2C_TO_UART_DATA:// Put this first because it's called most often?
					if( uart_select == 0x01 ){ // Comms selected UART1
						*inI2C_USART1++ = temp;
						if (inI2C_USART1 == outTail_USART1) { // Wrap around
							inI2C_USART1 = outHead_USART1;
						}
						uart1_wait++;
					}else if( uart_select == 0x02 ){ // Comms selected uart2
						*inI2C_USART2++ = temp;
						if (inI2C_USART2 == outTail_USART2) { // Wrap around
							inI2C_USART2 = outHead_USART2;
						}
						uart2_wait++;
					}
					break;
		  	  case I2C_TO_UART_SIZE:
			  		 // First check if it's for uart1 or 2
			  		if( uart_select == 0x01 ){
			  			uart1_todo += temp; // Keep track of the amount of data the DMA isn't aware of
			  			uart1_wait=0;
			  			if( temp==255 ){

			  			}
			  		}else{
			  			uart2_todo += temp; // Keep track of the amount of data the DMA isn't aware of
			  			uart2_wait=0;
			  			if( temp==255 ){

						}
			  		}
			  		i2c_state = I2C_TO_UART_DATA;
		  		  break;
		  	  case I2C_IDLE_STATE:
		  		  	  i2c_state = temp;
		  		  	  if(i2c_state == I2C_REC_STATUS){
		  		  		  // First check if it's for uart1 or 2
		  		  		  if( uart_select == 0x01 ){
		  		  			  tempState = USART1_cnt;
		  		  		  }else{
		  		  			  tempState = USART2_cnt;
		  		  		  }
				  	  }
					break;
		  	  default:
		  		  // Unknown state
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

		  /* Check if transfer direction is read (slave transmitter) */
		  if((I2C1->ISR & I2C_ISR_DIR) == I2C_ISR_DIR){ // If reading, prepare the first byte
			  I2C1->CR1 |= I2C_CR1_TXIE; /* Set transmit IT */
			  if( i2c_state==I2C_REC_STATUS ){
				  I2C1 ->TXDR = tempState/256; // MSB first
			  }else if( i2c_state==UART_TO_I2C_DATA ){
				  if( uart_select == 0x01 ){
					  I2C1 ->TXDR = *ui_read_USART1++;
					  USART1_cnt--;
					  if( ui_read_USART1 == ui_tail_USART1 ){
						  ui_read_USART1 = ui_head_USART1;
					  }
				  }else if( uart_select == 0x02 ){
					  I2C1 ->TXDR = *ui_read_USART2++;
					  USART2_cnt--;
					  if( ui_read_USART2 == ui_tail_USART2 ){
						  ui_read_USART2 = ui_head_USART2;
					  }
				  }
			  }else{
				  I2C1 ->TXDR = 0xAA;
			  }
		  }else{
			  i2c_state = I2C_IDLE_STATE; // Beginning of new comms
		  }
	  }else if((I2C1->ISR & I2C_ISR_TXIS) == I2C_ISR_TXIS){
		  I2C1->CR1 &=~ I2C_CR1_TXIE; /* Disable transmit IT */
		  if( i2c_state==I2C_REC_STATUS ){
			  I2C1 ->TXDR = tempState%256;
		  }else if( i2c_state==UART_TO_I2C_DATA ){
			  if( uart_select == 0x01 ){
				  I2C1 ->TXDR = *ui_read_USART1++;
				  USART1_cnt--;
				  if( ui_read_USART1 == ui_tail_USART1 ){
					  ui_read_USART1 = ui_head_USART1;
				  }
			  }else{
				  I2C1 ->TXDR = *ui_read_USART2++;
				  USART2_cnt--;
				  if( ui_read_USART2 == ui_tail_USART2 ){
					  ui_read_USART2 = ui_head_USART2;
				  }
			  }
		  }else{
			  I2C1 ->TXDR = 0xAA;
		  }
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
