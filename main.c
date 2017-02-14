#include "tm4c123gh6pm.h"
#include "stdint.h"

void EnableI2CModule0(void);
int8_t ReadRegister(int8_t RegisterAddress);
void PLL_Init(void);
void WriteRegister(int8_t RegisterAddress,int8_t Data);
int16_t ReadAccelX(void);
int16_t ReadAccelY(void);
int16_t ReadAccelZ(void);
volatile float X_Axis1,X_Axis2,Y_Axis1,Y_Axis2,Z_Axis1,Z_Axis2=0;
volatile int16_t RawX_Axis1,RawX_Axis2,RawY_Axis1,RawY_Axis2,RawZ_Axis1,RawZ_Axis2=0;

int main()	
{
	volatile long temp;
	PLL_Init();
	EnableI2CModule0();
	temp=ReadRegister(0x00);
	WriteRegister(0x31,0x0B);
	temp=ReadRegister(0x31);	
	WriteRegister(0x2D,0x08);
	temp=ReadRegister(0x2D);
	while(1)
	{
		/*X_Axis1 = RawX_Axis1 * 0.00390625;
		RawX_Axis2=ReadRegister(0x33);
		X_Axis2 = RawX_Axis2 * 0.00390625;
		RawY_Axis1=ReadRegister(0x34);
		Y_Axis1 = RawY_Axis1 * 0.00390625;
		RawY_Axis2=ReadRegister(0x35);
		Y_Axis2 = RawY_Axis2 * 0.00390625;
		RawZ_Axis1=ReadRegister(0x36);
		Z_Axis1 = RawZ_Axis1 * 0.00390625;
		RawZ_Axis2=ReadRegister(0x37);
		Z_Axis2 = RawZ_Axis2 * 0.00390625;
		*/
		RawX_Axis1=ReadAccelX();
		X_Axis1 = RawX_Axis1 * 0.00390625;
		RawY_Axis1=ReadAccelY();
		Y_Axis1 = RawY_Axis1 * 0.00390625;
		RawZ_Axis1=ReadAccelZ();
		Z_Axis1 = (RawY_Axis1 * 0.00390625)+1;
	}
}
void PLL_Init(void){
  // 0) Use RCC2
  SYSCTL_RCC2_R |=  0x80000000;  // USERCC2
  // 1) bypass PLL while initializing
  SYSCTL_RCC2_R |=  0x00000800;  // BYPASS2, PLL bypass
  // 2) select the crystal value and oscillator source
  SYSCTL_RCC_R = (SYSCTL_RCC_R &~0x000007C0)   // clear XTAL field, bits 10-6
                 + 0x00000540;   // 10101, configure for 16 MHz crystal
  SYSCTL_RCC2_R &= ~0x00000070;  // configure for main oscillator source
  // 3) activate PLL by clearing PWRDN
  SYSCTL_RCC2_R &= ~0x00002000;
  // 4) set the desired system divider
  SYSCTL_RCC2_R |= 0x40000000;   // use 400 MHz PLL
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~ 0x1FC00000)  // clear system clock divider
                  + (4<<22);      // configure for 80 MHz clock
  // 5) wait for the PLL to lock by polling PLLLRIS
  while((SYSCTL_RIS_R&0x00000040)==0){};  // wait for PLLRIS bit
  // 6) enable use of PLL by clearing BYPASS
  SYSCTL_RCC2_R &= ~0x00000800;
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
	GPIO_PORTB_DEN_R |= 0xFF; //Enable digital on Port B
	GPIO_PORTB_PCTL_R |=0x03;
	I2C0_PP_R |= 0x01;
	I2C0_MTPR_R = 0x00000039; //set SCL clock
	I2C0_MCR_R |= 0x00000010; //intialize mcr rigester with that value given in datasheet
}
int8_t ReadRegister(int8_t RegisterAddress)
{
	volatile int32_t mcs = I2C0_MCS_R ;
	volatile int8_t result=0;
	I2C0_MSA_R = 0x000000A6; //write operation
	I2C0_MDR_R = RegisterAddress; //place data to send mdr register
	I2C0_MCS_R = 0x00000007; //stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	I2C0_MSA_R = 0x000000A7; // read operation
	I2C0_MCS_R = 0x00000007; // stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	result = I2C0_MDR_R;
	return result;
}

void WriteRegister(int8_t RegisterAddress,int8_t Data)
{
	volatile int32_t mcs = I2C0_MCS_R ;
	I2C0_MSA_R = 0x000000A6; //write operation
	I2C0_MDR_R = RegisterAddress; //place register address to set in mdr register
	I2C0_MCS_R = 0x00000003; //burst send ( multiple bytes send ) 
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	I2C0_MDR_R = Data; //place data to be sent in  mdr register
	I2C0_MCS_R = 0x00000005; // transmit followed by stop state 
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
}
int16_t ReadAccelX(void)
{
	volatile int16_t result=0,LSB=0,MSB=0;
	I2C0_MSA_R = 0x000000A6; //write operation
	I2C0_MDR_R = 0x32; //data x0
	I2C0_MCS_R = 0x00000007; //stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	I2C0_MSA_R = 0x000000A7; // read operation
	I2C0_MCS_R = 0x00000007; // stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	LSB = I2C0_MDR_R;
	//result above is LSB of accel x
	//repeat to get MSB
	I2C0_MSA_R = 0x000000A6; //write operation
	I2C0_MDR_R = 0x33; //data x1
	I2C0_MCS_R = 0x00000007; //stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	I2C0_MSA_R = 0x000000A7; // read operation
	I2C0_MCS_R = 0x00000007; // stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit
	MSB=I2C0_MDR_R;
	MSB= MSB <<8;
	result = result | LSB;
	result = result | MSB;
return result;	
}
int16_t ReadAccelY(void)
{
	volatile int16_t result=0,LSB=0,MSB=0;
	I2C0_MSA_R = 0x000000A6; //write operation
	I2C0_MDR_R = 0x34; //data Y0
	I2C0_MCS_R = 0x00000007; //stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	I2C0_MSA_R = 0x000000A7; // read operation
	I2C0_MCS_R = 0x00000007; // stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	LSB = I2C0_MDR_R;
	//result above is LSB of accel Y
	//repeat to get MSB
	I2C0_MSA_R = 0x000000A6; //write operation
	I2C0_MDR_R = 0x35; //data Y1
	I2C0_MCS_R = 0x00000007; //stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	I2C0_MSA_R = 0x000000A7; // read operation
	I2C0_MCS_R = 0x00000007; // stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit
	MSB=I2C0_MDR_R;
	MSB= MSB <<8;
	result = result | LSB;
	result = result | MSB;
return result;	
}
int16_t ReadAccelZ(void)
{
	volatile int16_t result=0,LSB=0,MSB=0;
	I2C0_MSA_R = 0x000000A6; //write operation
	I2C0_MDR_R = 0x36; //data Z0
	I2C0_MCS_R = 0x00000007; //stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	I2C0_MSA_R = 0x000000A7; // read operation
	I2C0_MCS_R = 0x00000007; // stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	LSB = I2C0_MDR_R;
	//result above is LSB of accel Z
	//repeat to get MSB
	I2C0_MSA_R = 0x000000A6; //write operation
	I2C0_MDR_R = 0x37; //data xZ1
	I2C0_MCS_R = 0x00000007; //stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit 
	I2C0_MSA_R = 0x000000A7; // read operation
	I2C0_MCS_R = 0x00000007; // stop start run
	while((I2C0_MCS_R & 0x00000001)==1); //poll busy bit
	MSB=I2C0_MDR_R;
	MSB= MSB <<8;
	result = result | LSB;
	result = result | MSB;
return result;	
}
