#include "tm4c123gh6pm.h"

void EnableI2CModule0(void);

int main()
{
	EnableI2CModule0();
}

void EnableI2CModule0(void)
{
	volatile int Delay=0;
	SYSCTL_RCGCI2C_R|=0x00000001; //set i2c module 0 clock active
	Delay=SYSCTL_RCGCI2C_R; //delay allow clock to stabilize 
	SYSCTL_RCGCGPIO_R |=0x00000002; //i2c module 0 is portB so activate clock for port B
	Delay = SYSCTL_RCGCGPIO_R; //delay allow clock to stabilize 
	GPIO_PORTB_AFSEL_R|= 0x0000000C; //enable alternate functions for PB2 and PB3
	GPIO_PORTB_ODR_R |= 0x00000008; //set PB3 (I2C SDA)  for open drain
	//might need to add digital enable here or before setting open drain 
	GPIO_PORTB_PCTL_R |=0x03;
	I2C0_MCR_R |= 0x00000010; //intialize mcr rigester with that value given in datasheet
	I2C0_MTPR_R |= 0x00000009; //set SCL clock
	//end intialization code after this is sending code
	I2C0_MSA_R = 0x000000A6; //set slave address
	I2C0_MDR_R = 0x00000000; //place data to send mdr register
	I2C0_MCS_R = 0x00000007; //stop start run
	while((I2C0_MCS_R &= 0x00000001)==1); //poll busy bit 
	
	
	
}