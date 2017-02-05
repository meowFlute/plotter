#include <msp430.h> 

/*
 * main.c
 */
#define transmit_clear (UCB0TXIFG << 4)
#define byte_received  (UCB0RXIFG << 4)

volatile unsigned char i2c_status; //0 b TX, RX, something, something, NACK, STOP, START, LOST ARB
volatile unsigned char buffer[14]; //for reading the full sensor state

volatile short accelX;
volatile short accelY;
volatile short accelZ;

volatile short temp;

volatile short gyroX;
volatile short gyroY;
volatile short gyroZ;

short convert16bitSignedValue(unsigned char msb, unsigned char lsb);
short convertTwosComplement(unsigned char msb, unsigned char lsb);

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

	// according to the MSP430G2553 datasheet USCI mode is P1SEL = 1 and P1SEL2 = 1 for P1.6 and P1.7
	P1SEL |= BIT6 | BIT7;
	P1SEL2 |= BIT6 | BIT7;

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
	i2c_status &= ~(transmit_clear); //clear the TX bit
	UCB0CTL1 ^= UCSWRST;   // toggle that bad boy off and you're off to the races

	//enable interrupts (they won't run unless you do this)
	_EINT();

	//---------------------------------------------- execution
	//-----------MASTER TRANSMITTER MODE
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
	while(i2c_status & transmit_clear != transmit_clear){/*wait for the transmit buffer to be transferred*/}
	UCB0TXBUF = 0x6B; //write the address of the memory location we want to write to on the i2c device
	i2c_status &= ~(transmit_clear); //clear the transmit clear flag once loaded
	while(UCB0CTL1 & UCTXSTT == UCTXSTT){/*wait for ACK*/}
	/* The data written into UCBxTXBUF is transmitted if arbitration is not lost during transmission of the slave
	address. UCBxTXIFG is set again as soon as the data is transferred from the buffer into the shift register.
	If there is no data loaded to UCBxTXBUF before the acknowledge cycle, the bus is held during the
	acknowledge cycle with SCL low until data is written into UCBxTXBUF. Data is transmitted or the bus is
	held as long as the UCTXSTP bit or UCTXSTT bit is not set.*/
	while(i2c_status & transmit_clear != transmit_clear)
	{
		/*wait for the transmit buffer to be transferred*/
		//also check for arbitration being lost
		if(i2c_status & UCALIFG == UCALIFG)
			return -1; //abort mission
	}
	UCB0TXBUF = 0x02; //load normal powerstate into buffer
	i2c_status &= ~(transmit_clear); //clear transmit clear state
	while(UCB0CTL1 & UCTXSTT == UCTXSTT){/*wait for ACK*/}
	while(i2c_status & transmit_clear != transmit_clear)
	{
		/*wait for the transmit buffer to be transferred*/
		//also check for arbitration being lost
		if(i2c_status & UCALIFG == UCALIFG)
			return -1; //abort mission
	}
	/*
	Setting UCTXSTP will generate a STOP condition after the next acknowledge from the slave. If UCTXSTP
	is set during the transmission of the slave’s address or while the USCI module waits for data to be written
	into UCBxTXBUF, a STOP condition is generated even if no data was transmitted to the slave. When
	transmitting a single byte of data, the UCTXSTP bit must be set while the byte is being transmitted, or
	anytime after transmission begins, without writing new data into UCBxTXBUF. Otherwise, only the
	address will be transmitted. When the data is transferred from the buffer to the shift register, UCBxTXIFG
	will become set indicating data transmission has begun and the UCTXSTP bit may be set. */
	UCB0CTL1 |= UCTXSTP; // right after the transmit bit is set
	i2c_status &= ~(transmit_clear);

	//set 200 Hz Sample Rate
	UCB0CTL1 |= UCTXSTT;
	while(i2c_status & transmit_clear != transmit_clear){} //transmit buffer open
	UCB0TXBUF = 0x19; // address of the register that you're writing to
	i2c_status &= ~(transmit_clear);
	while((i2c_status & transmit_clear != transmit_clear)&&(UCB0CTL1 & UCTXSTT == UCTXSTT))
	{
			if(i2c_status & UCALIFG == UCALIFG)
				return -1; //abort mission
	}
	UCB0TXBUF = 0x04; // message you're inserting into the register
	i2c_status &= ~(transmit_clear);
	while((i2c_status & transmit_clear != transmit_clear)&&(UCB0CTL1 & UCTXSTT == UCTXSTT))
	{
			if(i2c_status & UCALIFG == UCALIFG)
				return -1; //abort mission
	}
	UCB0CTL1 |= UCTXSTP; // right after the transmit bit is set
	i2c_status &= ~(transmit_clear);

	//set lowpass bandwidth
	UCB0CTL1 |= UCTXSTT;
	while(i2c_status & transmit_clear != transmit_clear){} //transmit buffer open
	UCB0TXBUF = 0x1A; // address of the register that you're writing to
	i2c_status &= ~(transmit_clear);
	while((i2c_status & transmit_clear != transmit_clear)&&(UCB0CTL1 & UCTXSTT == UCTXSTT))
	{
			if(i2c_status & UCALIFG == UCALIFG)
				return -1; //abort mission
	}
	UCB0TXBUF = 0x03; // message you're inserting into the register
	i2c_status &= ~(transmit_clear);
	while((i2c_status & transmit_clear != transmit_clear)&&(UCB0CTL1 & UCTXSTT == UCTXSTT))
	{
			if(i2c_status & UCALIFG == UCALIFG)
				return -1; //abort mission
	}
	UCB0CTL1 |= UCTXSTP; // right after the transmit bit is set
	i2c_status &= ~(transmit_clear);

	//set gyro full scale range
	UCB0CTL1 |= UCTXSTT;
	while(i2c_status & transmit_clear != transmit_clear){} //transmit buffer open
	UCB0TXBUF = 0x1B; // address of the register that you're writing to
	i2c_status &= ~(transmit_clear);
	while((i2c_status & transmit_clear != transmit_clear)&&(UCB0CTL1 & UCTXSTT == UCTXSTT))
	{
			if(i2c_status & UCALIFG == UCALIFG)
				return -1; //abort mission
	}
	UCB0TXBUF = 0x08; // message you're inserting into the register
	i2c_status &= ~(transmit_clear);
	while((i2c_status & transmit_clear != transmit_clear)&&(UCB0CTL1 & UCTXSTT == UCTXSTT))
	{
			if(i2c_status & UCALIFG == UCALIFG)
				return -1; //abort mission
	}
	UCB0CTL1 |= UCTXSTP; // right after the transmit bit is set
	i2c_status &= ~(transmit_clear);

	//set accelerometer full scale range
	UCB0CTL1 |= UCTXSTT;
	while(i2c_status & transmit_clear != transmit_clear){} //transmit buffer open
	UCB0TXBUF = 0x1C; // address of the register that you're writing to
	i2c_status &= ~(transmit_clear);
	while((i2c_status & transmit_clear != transmit_clear)&&(UCB0CTL1 & UCTXSTT == UCTXSTT))
	{
			if(i2c_status & UCALIFG == UCALIFG)
				return -1; //abort mission
	}
	UCB0TXBUF = 0x08; // message you're inserting into the register
	i2c_status &= ~(transmit_clear);
	while((i2c_status & transmit_clear != transmit_clear)&&(UCB0CTL1 & UCTXSTT == UCTXSTT))
	{
			if(i2c_status & UCALIFG == UCALIFG)
				return -1; //abort mission
	}
	UCB0CTL1 |= UCTXSTP; // right after the transmit bit is set
	i2c_status &= ~(transmit_clear);

	//-----------MASTER RECEIVER MODE

	while(1)
		{
		//on the mpu6050 you have to transmit the address of the register you want to receive from
		UCB0CTL1 |= UCTXSTT;
		while(i2c_status & transmit_clear != transmit_clear){} //transmit buffer open
		UCB0TXBUF = 0x3B; // address of the register that you're reading from
		i2c_status &= ~(transmit_clear);
		while((i2c_status & transmit_clear != transmit_clear)&&(UCB0CTL1 & UCTXSTT == UCTXSTT))
		{
				if(i2c_status & UCALIFG == UCALIFG)
					return -1; //abort mission
		}
		/* After initialization, master receiver mode is initiated by writing the desired slave address to the
		UCBxI2CSA register, selecting the size of the slave address with the UCSLA10 bit, clearing UCTR for
		receiver mode, and setting UCTXSTT to generate a START condition. */
		// already set - UCB0I2CSA = 0x68; //MPU6050 address
		UCB0CTL1 &= ~(UCTR); //clear the UCTR bit
		UCB0CTL1 |= UCTXSTT; //set the start condition
		/* The USCI module checks if the bus is available, generates the START condition, and transmits the slave
		address. As soon as the slave acknowledges the address the UCTXSTT bit is cleared. */
		while(UCB0CTL1 & UCTXSTT == UCTXSTT)
		{
			/*wait for ACK*/
			if(i2c_status & UCNACKIFG == UCNACKIFG)
			{
				/*If the slave does not acknowledge the transmitted address the not-acknowledge interrupt flag
				UCNACKIFG is set. The master must react with either a STOP condition or a repeated START condition.*/
				UCB0CTL1 |= UCTXSTT;
			}
		}
		unsigned char numBytesRead = 0;
		while(numBytesRead < 14)
		{
			while(i2c_status & byte_received != byte_received){/*wait for byte*/}
			buffer[numBytesRead] = UCB0RXBUF; //read the byte in
			i2c_status &= ~byte_received; //clear the flag
			numBytesRead++; //increment the number of bytes read
		}
		UCB0CTL1 |= UCTXSTP; //set the stop condition

		//--------------------Process the data
		accelX = convertTwosComplement(buffer[0], buffer[1]);
		accelY = convertTwosComplement(buffer[2], buffer[3]);
		accelX = convertTwosComplement(buffer[4], buffer[5]);

		temp = convert16bitSignedValue(buffer[6], buffer[7]);

		gyroX = convertTwosComplement(buffer[8], buffer[9]);
		gyroY = convertTwosComplement(buffer[10], buffer[11]);
		gyroZ = convertTwosComplement(buffer[12], buffer[13]);
	}
	return 0; //exit
}

short convert16bitSignedValue(unsigned char msb, unsigned char lsb)
{
	return (short)(((msb & 0xFF) << 8) | (lsb & 0xFF));
}

short convertTwosComplement(unsigned char msb, unsigned char lsb)
{
	int t = msb * 0x100L + lsb;
	if (t >= 32768)
		t -= 65536;
	return (short)t;
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
		i2c_status |= UCSTTIFG;
		// clear UCSTTIFG
		UCB0STAT &= ~UCSTTIFG;
	}
	// stop flag
	/* Stop condition detected interrupt. This flag is set when the I2C module detects a STOP condition while in
	UCSTPIFG slave mode. UCSTPIFG is used in slave mode only and is automatically cleared when a START condition is
	received. */
	if(UCB0STAT & UCSTPIFG == UCSTPIFG)
	{
		i2c_status |= UCSTPIFG;
		// clear UCSTPIFG
		UCB0STAT &= ~UCSTPIFG;
	}
	// lost arbitration flag
	/* Arbitration-lost. Arbitration can be lost when two or more transmitters start a transmission simultaneously, or
	UCALIFG when the USCI operates as master but is addressed as a slave by another master in the system. The UCALIFG flag
	is set when arbitration is lost. When UCALIFG is set the UCMST bit is cleared and the I2C controller becomes a slave.*/
	if(UCB0STAT & UCALIFG == UCALIFG)
	{
		i2c_status |= UCALIFG;
		// clear UCALIFG
		UCB0STAT &= ~UCALIFG;
	}
	// not-acknowledge flag
	/* Not-acknowledge interrupt. This flag is set when an acknowledge is expected but is not received.
	UCNACKIFG is automatically cleared when a START condition is received. */
	if(UCB0STAT & UCNACKIFG == UCNACKIFG)
	{
		i2c_status |= UCNACKIFG;
		// clear UCALIFG
		UCB0STAT &= ~UCNACKIFG;
	}
}


#pragma vector=USCIAB0TX_VECTOR
__interrupt void i2c_TX_RX_interrupt()
{
	//transmit: this is set when the transmit buffer is empty
	if(IFG2 & UCB0TXIFG == UCB0TXIFG)
	{
		//move it over to the left 4 bits
		i2c_status |= UCB0TXIFG << 4;
		//clear flag on way out
		IFG2 &= ~UCB0TXIFG;
	}
	//receive: this is set when a complete byte is received
	if(IFG2 & UCB0RXIFG == UCB0RXIFG)
	{
		//move it over to the left 4 bits
		i2c_status |= UCB0RXIFG << 4;

		//clear flag on way out
		IFG2 &= ~UCB0TXIFG;
	}
}
