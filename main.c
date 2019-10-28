#include <avr/io.h>
#include <util/delay.h>
#include <compat/twi.h>
#include "uart.c"
#define MAX_TRIES 50
#define IODIR 0x00           // MCP23008 I/O Direction Register
#define GPIO  0x12           // MCP23008 General Purpose I/O Register
#define OLAT  0x0A           // MCP23008 Output Latch Register
#define MCP_ID 0x40
#define I2C_START 0
#define I2C_DATA 1
#define I2C_DATA_ACK 2
#define I2C_STOP 3
#define ACK 1
#define NACK 0
#define DATASIZE 32

/*parameters*/
  long int ac5;
   int ac6;
   int mc;
   int md;
/*        */
int32_t computeB5(int32_t UT) {
	int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
	int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
	return X1 + X2;
}
/* START I2C Routine */
unsigned char i2c_transmit(unsigned char type) {
	switch(type) {
		case I2C_START:    // Send Start Condition
		TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
		break;
		case I2C_DATA:     // Send Data with No-Acknowledge
		TWCR = (1 << TWINT) | (1 << TWEN);
		break;
		case I2C_DATA_ACK: // Send Data with Acknowledge
		TWCR = (1 << TWEA) | (1 << TWINT) | (1 << TWEN);
		break;
		case I2C_STOP:     // Send Stop Condition
		TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
		return 0;
	}
	// Wait for TWINT flag set on Register TWCR
	while (!(TWCR & (1 << TWINT)));
	// Return TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
	return (TWSR & 0xF8);
}
char i2c_start(unsigned int dev_id, unsigned int dev_addr, unsigned char rw_type)
{
	unsigned char n = 0;
	unsigned char twi_status;
	char r_val = -1;
	i2c_retry:
	if (n++ >= MAX_TRIES) return r_val;
	// Transmit Start Condition
	twi_status=i2c_transmit(I2C_START);

	// Check the TWI Status
	if (twi_status == TW_MT_ARB_LOST) goto i2c_retry;
	if ((twi_status != TW_START) && (twi_status != TW_REP_START)) goto i2c_quit;
	// Send slave address (SLA_W)
	TWDR = (dev_id & 0xF0) | (dev_addr & 0x0E) | rw_type;
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	// Check the TWSR status
	if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST)) goto i2c_retry;
	if (twi_status != TW_MT_SLA_ACK) goto i2c_quit;
	r_val=0;
	i2c_quit:
	return r_val;
}
char i2c_start_R(unsigned int device)
{
	unsigned char n = 0;
	unsigned char twi_status;
	char r_val = -1;
	i2c_retry:
	if (n++ >= MAX_TRIES) return r_val;
	// Transmit Start Condition
	twi_status=i2c_transmit(I2C_START);

	// Check the TWI Status
	if (twi_status == TW_MT_ARB_LOST) goto i2c_retry;
	if ((twi_status != TW_START) && (twi_status != TW_REP_START)) goto i2c_quit;
	// Send slave address (SLA_W)
	TWDR = device;
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	// Check the TWSR status
	if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST)) goto i2c_retry;
	if (twi_status != TW_MT_SLA_ACK) goto i2c_quit;
	r_val=0;
	i2c_quit:
	return r_val;
}
void i2c_stop(void)
{
	unsigned char twi_status;
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_STOP);
}
char i2c_write(char data)
{
	unsigned char twi_status;
	char r_val = -1;
	// Send the Data to I2C Bus
	TWDR = data;
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	// Check the TWSR status
	if (twi_status != TW_MT_DATA_ACK) goto i2c_quit;
	r_val=0;
	i2c_quit:
	return r_val;
}
long int i2c_read(long int *data,char ack_type)
{
	unsigned char twi_status;
	char r_val = -1;

	if (ack_type) {
		// Read I2C Data and Send Acknowledge
		twi_status=i2c_transmit(I2C_DATA_ACK);
		if (twi_status != TW_MR_DATA_ACK) goto i2c_quit;
		} else {
		// Read I2C Data and Send No Acknowledge
		twi_status=i2c_transmit(I2C_DATA);
		if (twi_status != TW_MR_DATA_NACK) goto i2c_quit;
	}
	// Get the Data
	*data=TWDR;
	r_val=0;
	i2c_quit:
	return r_val;
}

void i2c_init(void)
{
	// Initial ATMega328P TWI/I2C Peripheral
	TWSR = 0x00;         // Select Prescaler of 1
	// SCL frequency = 11059200 / (16 + 2 * 48 * 1) = 98.743 kHz
	TWBR = 0x30;        // 48 Decimal
}
void read_params()
{
	int msb;
	int lsb;

	printf("Starting to read params\n"); //read AC5 msb,
	i2c_start_R(0xEE); // type write
	i2c_write(0xB2);
	i2c_stop();
	
	//restart transmission
	i2c_start_R(0xEF); //type read
	i2c_read(&msb,NACK);
	i2c_stop();
	
	i2c_start_R(0xEE); //read AC5 lsb,
	i2c_write(0xB3);
	i2c_stop();
	
	//restart transmission
	i2c_start_R(0xEF);
	i2c_read(&lsb,NACK);
	i2c_stop();
	ac5 = (msb << 8) + lsb;
	printf("AC5 : %d\n", ac5); //AC6 start
	
	i2c_start_R(0xEE);
	i2c_write(0xB4);
	i2c_stop();
	
	//restart transmission
	i2c_start_R(0xEF);
	i2c_read(&msb,NACK);
	i2c_stop();
	
	i2c_start_R(0xEE); //read AC6 lsb,
	i2c_write(0xB5);
	i2c_stop();
	
	//restart transmission
	i2c_start_R(0xEF);
	i2c_read(&lsb,NACK);
	i2c_stop();
	
	ac6 = (msb << 8) + lsb;
	printf("AC6 : %d\n", ac6); //start MC
	i2c_start_R(0xEE);
	i2c_write(0xBC);
	i2c_stop();
	
	//restart transmission
	i2c_start_R(0xEF);
	i2c_read(&msb,NACK);
	i2c_stop();
	
	i2c_start_R(0xEE); //read MC lsb,
	i2c_write(0xBD);
	i2c_stop();
	
	//restart transmission
	i2c_start_R(0xEF);
	i2c_read(&lsb,NACK);
	i2c_stop();
	mc = (msb << 8) + lsb;
	printf("MC : %d\n", mc);
	
	i2c_start_R(0xEE);
	i2c_write(0xBE); //MD MSB
	i2c_stop();
	
	//restart transmission
	i2c_start_R(0xEF);
	i2c_read(&msb,NACK);
	i2c_stop();
	
	i2c_start_R(0xEE); //read MC lsb,
	i2c_write(0xBF);
	i2c_stop();
	
	//restart transmission
	i2c_start_R(0xEF);
	i2c_read(&lsb,NACK);
	i2c_stop();
	md = (msb << 8) + lsb;	
	printf("MD : %d\n", md);
}
long int Read_RAW()
{
	long int data_msb;
	long int data_lsb;
	// Start the I2C Write Transmission
	i2c_start_R(0xEE);
	// Read data from MCP23008 Register Address
	i2c_write(0xF4);
	i2c_write(0x2E);
	//printf("[DEBUG] writing to 0x2E\n");
	// Stop I2C Transmission
	i2c_stop();
	_delay_ms(25);
	//printf("[DEBUG] delay of 5\n");
	// Re-Start the I2C Read Transmission
	
	i2c_start_R(0xEE); // type write
	i2c_write(0xF6);
	i2c_stop();
	
	//restart transmission
	i2c_start_R(0xEF); //type read
	i2c_read(&data_msb,NACK);
	i2c_stop();
	
	i2c_start_R(0xEE); //read AC5 lsb,
	i2c_write(0xF7);
	i2c_stop();
	
	//restart transmission
	i2c_start_R(0xEF);
	i2c_read(&data_lsb,NACK);
	i2c_stop();
	
	return (data_msb << 8) + data_lsb;
}
void Write_MCP(unsigned char reg_addr,unsigned char data)
{
	// Start the I2C Write Transmission
	i2c_start_R(MCP_ID);
	// Sending the Register Address
	i2c_write(reg_addr);
	// Write data to MCP23008 Register
	i2c_write(data);
	// Stop I2C Transmission
	i2c_stop();
}

int main(void) {
    serial_init();
	i2c_init();	
	read_params();
	Write_MCP(IODIR,0x00);
	//printf("%li", Read_RAW());
	

	float res;
	int32_t B5;
	while (1) {
		long int raw_UT = Read_RAW();
		B5 = computeB5((int32_t)raw_UT);
		res = (B5+8) >> 4;
       printf("Temperature: %d , %.1f\n",(int)res/10, res/10);
		Write_MCP(GPIO,(int)res/10);
       _delay_ms(3000);
       
	}
	return 0; // never reached
}