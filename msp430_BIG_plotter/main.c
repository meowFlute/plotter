#include <msp430.h> 

/*
 * main.c
 */
volatile unsigned char transmit_buffer_clear;

int main(void) {
	//---------------------------------------------- Initialize
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    //Change DCO frequency to 16 MHz
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL  = CALDCO_16MHZ;
	
	//set both masterclock AND sub master clock sources to DCO
	//make the masterclock oscilate at a period of DCO/1 and the SMCLK at DCO/1
	BCSCTL2 = DIVS_0;

	// I/0 functionality on all pins
	P1SEL  = 0x00;					// Set P1.0-P1.7 to I/O functionality
	P1SEL2 = 0x00;
	P2SEL  = 0x00;					// Set P1.0-P1.7 to I/O functionality
	P2SEL2 = 0x00;

	// set all pins to the output direction by default (less power consumption so long as they aren't connected to anything)
	P1DIR = 0b00111111;				// Set P1.0-P1.5 to output direction (1.6 & 1.7 are i2c)
	P2DIR = 0xff;					// Set P2.0-P2.7 to output direction (unused)

	// i2c initialization
	UCB0CTL1 |= UCSWRST;   // ensure that this bit is set first
	UCB0CTL0 = UCMODE_3;   // I2C mode with 7-bit addressing
	UCB0CTL1 |= UCSSEL_3;  // SMCLK as BITCLK source
	UCB0BR0 = 40;		   // Set i2c baud rate to 16MHz / 40 = 400kHz
	UCB0BR1 = 0;
	UCB0I2CIE |= 0x0F;     // enable start interrupts, receive interrupts, NACK interrupts, and arbitration lost interrupts
	IE2 |= BIT3 | BIT2;    // enable transmit and receive interrupts on UCB0
	transmit_buffer_clear = 0;
	UCB0CTL1 ^= UCSWRST;   // toggle that bad boy off and you're off to the races

	//enable interrupts (they won't run unless you do this)
	_EINT();

	//---------------------------------------------- execution
	// After initialization, master transmitter mode is initiated by:
	//	1) writing the desired slave address to the UCB0I2CSA register
	//		a) The MPU6050 address is 0x68 if ADO is held low
	//	2) selecting the size of the slave address with the UCSLA10
	//		a) already done when I set UCB0CTL0 above
	//  3) setting UCTR for transmitter mode
	//  4) and setting UCTXSTT to generate a start condition
	UCB0I2CSA = 0x68; //step 1 (2 done)
	UCB0CTL1 |= UCTR | UCTXSTT; // steps 3 and 4
	// The USCI module checks if the bus is available (bit 4 of status register??? idk), generates a start condition, and transmits the slave address.
	// The UCBxTXIFG bit is set when the START condition is generated and the first data to be transmitted can be written
	// and the first data to be transmitted can be written into UCB0TXBUF.
	// As soon as the slave acknowledges the address the UCTXSTT bit is cleared
	while(!transmit_buffer_clear){/*wait for the transmit buffer to be transferred*/}
	UCB0TXBUF = 0x6B; //write the address of the memory location we want to write to on the i2c device
	while(UCB0CTL1 & UCTXSTT == UCTXSTT){/*wait for ACK*/}


	return 0;
}


#pragma vector=USCIAB0RX_VECTOR
__interrupt void i2c_stateChangeInterrupt()
{
	// start flag
	/* Start condition detected interrupt. This flag is set when the I2C module detects a START condition together
	UCSTTIFG with its own address while in slave mode. UCSTTIFG is used in slave mode only and is automatically
	cleared when a STOP condition is received.*/
	if(UCB0STAT & UCSTTIFG == UCSTTIFG)
	{

		// clear UCSTTIFG
		UCB0STAT ^= UCSTTIFG;
	}
	// stop flag
	/* Stop condition detected interrupt. This flag is set when the I2C module detects a STOP condition while in
	UCSTPIFG slave mode. UCSTPIFG is used in slave mode only and is automatically cleared when a START condition is
	received. */
	if(UCB0STAT & UCSTPIFG == UCSTPIFG)
	{
		// clear UCSTPIFG
		UCB0STAT ^= UCSTPIFG;
	}
	// lost arbitration flag
	/* Arbitration-lost. Arbitration can be lost when two or more transmitters start a transmission simultaneously, or
	UCALIFG when the USCI operates as master but is addressed as a slave by another master in the system. The UCALIFG flag
	is set when arbitration is lost. When UCALIFG is set the UCMST bit is cleared and the I2C controller becomes a slave.*/
	if(UCB0STAT & UCALIFG == UCALIFG)
	{
		// clear UCALIFG
		UCB0STAT ^= UCALIFG;
	}
	// not-acknowledge flag
	/* Not-acknowledge interrupt. This flag is set when an acknowledge is expected but is not received.
	UCNACKIFG is automatically cleared when a START condition is received. */
	if(UCB0STAT & UCNACKIFG == UCNACKIFG)
	{
		// clear UCALIFG
		UCB0STAT ^= UCNACKIFG;
	}
}


#pragma vector=USCIAB0TX_VECTOR
__interrupt void i2c_TX_RX_interrupt()
{
	//transmit: this is set when the transmit buffer is empty
	if(IFG2 & UCB0TXIFG == UCB0TXIFG)
	{
		// the transmit flag is set
		transmit_buffer_clear = 1;
		//clear flag on way out
		IFG2 ^= UCB0TXIFG;
	}
	//receive: this is set when a complete byte is received
	if(IFG2 & UCB0RXIFG == UCB0RXIFG)
	{
		// the receive flag is set


		//clear flag on way out
		IFG2 ^= UCB0TXIFG;
	}
}
}
