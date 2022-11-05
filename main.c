#include "stm32f4xx.h"
#include "BufferComandos.h"
/*Actual configuración: 
	+Captura de 800x800 pixeles
	+Omitir negros de la trama
	+Oneshot mode
	+10 bits por pixel
*/
	
/*1 para encender
	2 para apagar
	r/R para reset
	c/C capturar imagen
	t/T leer temperatura sensor
	k/K Habilitar clk del sensor*/
#define AFRL  AFR[0]
#define AFRH  AFR[1]
/*Defines for TIMER2*/
#define TIMER_2_ENABLE   TIM2->CR1 |=1
#define TIMER_2_DISABLE	 TIM2->CR1 &=~1
#define TIMER_2_PREGUNTA (TIM2->SR & 1)  
#define TIMER_2_RESET	TIM2->SR &= 0xFFFE

/*Defines for TIMER3*/
#define TIMER_3_ENABLE   	TIM3->CR1 |=1
#define TIMER_3_DISABLE	 	TIM3->CR1 &=~1
#define TIMER_3_PREGUNTA (TIM3->SR & 1)  
#define TIMER_3_RESET			TIM3->SR &= 0xFFFE

/*Defines for USART2*/
#define USART2_RX_FLAG	   (USART2->SR & 0x10)
#define USART2_RX_DATA	    USART2->DR
#define USART2_TX_FLAG		 (USART2->SR &  0x0040) 
#define USART2_TX_FLAG_R    USART2->SR &= 0xFFBF
#define USART2_TX_DATA	    USART2->DR

/*Defines for FSMC*/
#define FSMC_SRAM_ADDRESS_BK1     0x64000000
#define FSMC_SRAM_ADDRESS_BK2     0x68000000
#define FSMC_BANK_1_SRAM2_ENABLE				FSMC_Bank1->BTCR[2]	 |=   0x1
#define FSMC_BANK_1_SRAM2_DISABLE				FSMC_Bank1->BTCR[2]	 &= ~(0x1)
#define FSMC_BANK_1_SRAM3_ENABLE				FSMC_Bank1->BTCR[4]	 |=   0x1
#define FSMC_BANK_1_SRAM3_DISABLE				FSMC_Bank1->BTCR[4]	 &= ~(0x1)
#define FSMC_BANK_1_SRAM2_WR_ENABLE			FSMC_Bank1->BTCR[2]	 |=   0x1000
#define FSMC_BANK_1_SRAM2_WR_DISABLE		FSMC_Bank1->BTCR[2]	 &= ~(0x1000)
#define FSMC_BANK_1_SRAM3_WR_ENABLE			FSMC_Bank1->BTCR[4]	 |=   0x1000
#define FSMC_BANK_1_SRAM3_WR_DISABLE		FSMC_Bank1->BTCR[4]	 &= ~(0x1000)


/*Defines for DCMI*/
#define DCMI_DR_ADDRESS       0x50050028

/*Defines for SPI_1*/
#define	SPI_1_ENABLE			GPIOA->BSRRH |= 0x8000 //NSS PIN A15 CLEAR
#define SPI_1_DISABLE 		GPIOA->BSRRL |= 0x8000 //NSS PIN A15 SET
#define SPI_1_CLK_RISE		GPIOA->BSRRL |= 0x0020 //CLK PIN A5 SET
#define SPI_1_CLK_FALL		GPIOA->BSRRH |= 0x0020 //CLK PIN A5 CLEAR

/*-----------------------------------------------------------------------------*/
//ESTADOS DEL SENSOR
#define POWER_OFF										0
#define LOW_POWER_STANDBY		 				1
#define	STANDBY_1										2
#define	INTERMEDIATE_STANDBY 				3
#define	STANDBY_2										4
#define	IDLE												5
#define	RUNNING											6

//DEFINE SENSOR
#define SENSOR_CLOCK_ENABLE 	GPIOC->BSRRL=0x0020
#define SENSOR_CLOCK_DISABLE 	GPIOC->BSRRH=0x0020
#define SENSOR_SET						GPIOB->BSRRH=0x0002
#define SENSOR_RESET					GPIOB->BSRRL=0x0002
#define SENSOR_3V3_ENABLE			GPIOD->BSRRL=0x0004
#define SENSOR_3V3_DISABLE		GPIOD->BSRRH=0x0004
#define SENSOR_1V8_ENABLE			GPIOD->BSRRL=0x0008
#define SENSOR_1V8_DISABLE		GPIOD->BSRRH=0x0008
/*-----------------------------------------------------------------------------*/
//Program_flags_0

#define FLAG_0_POWER_UP			 0x01	//bit 0 => Control secuencia de encendido sensor
#define FLAG_0_POWER_OFF		 0x02	//bit 1 => Control secuencia de apagado sensor
#define FLAG_0_CAPTURE_IMAGE 0x04 //bit 2 => Control captura de imagen
#define FLAG_0_SENSOR_TEMP	 0x08 //bit 3 => Control temperatura del sensor
#define FLAG_0_CLK_SENSOR_EN 0x10 //bit 4 => Bandera provisional para habilitar clk del sensor
//Program_flags_1
//Banderas para el control de memorias
#define FLAG_1_SRAM_2_ENABLE 0x01
#define FLAG_1_SRAM_3_ENABLE 0x02
//Program defines
#define CAPTURAR 		1
#define IMAGEN_SOLICITADA 2
/*-----------------------------------------------------------------------------*/
int32_t frame_buffer[30000];
void SystemInit(void);
void SPI_Config_Sensor_Send (int addr,int data);//Function to send data for configure the sensor
int  SPI_Config_Sensor_Read (int addr);//Function to send data for read data from sensor registers
char Command_Handler_Recepcion( char flag );
void Command_Handler( void );
void Capturar_Imagen (void);
void Enviar_Imagen(void);

const uint16_t VITA1300_EnableClockManagment_1_Config[][2]=
{
	{2, 0x0003},		// Color sensor parallel mode selection
	{32, 0x200C},		// Configure clcok managment
	{20, 0x0000},		// Configure clcok managment
	{16, 0x0007}		// Configure PLL bypass mode
};
const uint16_t VITA1300_DisableClockManagment_1_Config[][2]=
{
	{16, 0x0000},		// Disable PLL
	{8, 0x0099},		// Soft reset PLL
	{20, 0x0000}		// Configure clock management
};
const uint16_t VITA1300_EnableClockManagment_2_Config[][2]=
{
	{9, 0x0000},		// Release clock generatr soft reset
	{32, 0x200E},		// Enable logic clock
	{34, 0x0001}		// Enable logic blocks
};
const uint16_t VITA1300_DisableClockManagment_2_Config[][2]=
{
	{34, 0x0000},		// Disable logic blocks
	{32, 0x200c},		// Disable logic clock
	{9, 0x0009}			// Soft reset clock generator
};
const uint16_t VITA1300_Required_Register_Upload_Config[][2]=
{
	{41, 0x085a},		// Configure image core						0
	{129, 0xC001},	// 10-bit mode           					1
	{65, 0x288b},		// Configure CP biasing						2
	{66, 0x53c5},		// Configure AFE biasing					3
	{67, 0x0344},		// Configure MUX biasing					4
	{68, 0x0085},		// Configure LVDS biasing					5
	{70, 0x4800},		// Configure AFE biasing					6
	{128, 0x4710},	// Configure black calibration		7
	{197, 0x0103},	// Configure black calibration		8
	{176, 0x00f5},	// Configure AEC									9
	{180, 0x00fd},	// Configure AEC									10
	{181, 0x0144},	// Configure AEC									11
	{387, 0x549f},	// Configure sequencer						12
	{388, 0x549f},	// Configure sequencer						13
	{389, 0x5091},	// Configure sequencer						14
	{390, 0x1011},	// Configure sequencer						15
	{391, 0x111f},	// Configure sequencer						16
	{392, 0x1110},	// Configure sequencer						17
	{431, 0x0356},	// Configure sequencer						18
	{432, 0x0141},	// Configure sequencer						19
	{433, 0x214f},	// Configure sequencer						20
	{434, 0x214a},	// Configure sequencer						21
	{435, 0x2101},  // Configure sequencer						22
	{436, 0x0101},	// Configure sequencer						23
	{437, 0x0b85},	// Configure sequencer						24
	{438, 0x0381},	// Configure sequencer						25
	{439, 0x0181},	// Configure sequencer						26
	{440, 0x218f},	// Configure sequencer						27
	{441, 0x218a},	// Configure sequencer						28
	{442, 0x2101},	// Configure sequencer						29
	{443, 0x0100},	// Configure sequencer						30
	{447, 0x0b55},	// Configure sequencer						31
	{448, 0x0351},	// Configure sequencer						32
	{449, 0x0141},	// Configure sequencer						33
	{450, 0x214f},	// Configure sequencer						34
	{451, 0x214a},	// Configure sequencer						35
	{452, 0x2101},	// Configure sequencer						36
	{453, 0x0101},	// Configure sequencer						37
	{454, 0x0b85},	// Configure sequencer						38
	{455, 0x0381},	// Configure sequencer						39
	{456, 0x0181},	// Configure sequencer						40
	{457, 0x218f},	// Configure sequencer						41
	{458, 0x218a},	// Configure sequencer						42
	{459, 0x2101},	// Configure sequencer						43
	{460, 0x0100},	// Configure sequencer						44
	{469, 0x2184},	// Configure sequencer						45
	{472, 0x1347},	// Configure sequencer						46
	{476, 0x2144},	// Configure sequencer						47
	{480, 0x8d04},	// Configure sequencer						48
	{481, 0x8501},	// Configure sequencer						49
	{484, 0xcd04},	// Configure sequencer						50
	{485, 0xc501},	// Configure sequencer						51
	{489, 0x0be2},	// Configure sequencer						52
	{493, 0x2184},	// Configure sequencer						53
	{496, 0x1347},	// Configure sequencer						54
	{500, 0x2144},	// Configure sequencer						55
	{504, 0x8d04},	// Configure sequencer						56
	{505, 0x8501},	// Configure sequencer						57
	{508, 0xcd04},	// Configure sequencer						58
	{509, 0xc501}		// Configure sequencer						59
};
const uint16_t VITA1300_SofT_PowerUp_Config[][2]=
{
	{32, 0x200f}, 	// Enable analog clock distribution
	{10, 0x0000},		// Release soft reset state
	{64, 0x0001},		// Enable biasing block
	{72, 0x0203},		// Enable charge pump
	{40, 0x0003},		// Enable column multiplexer
	{48, 0x0001},		// Enable AFE
	{112, 0x0000}		// Configure I/O
};
const uint16_t VITA1300_SofT_PowerDown_Config[][2]=
{
	{112, 0x0000},	// Disable LVDS transmitters
	{48, 0x0000},		// Disable AFE
	{40, 0x0000},		// Disable column multiplexer
	{72, 0x0200},		// Disable charge pump
	{64, 0x0000},		// Disable biasing block
	{10, 0x0999}		// Soft reset
};
const uint16_t VITA1300_Enable_Sequencer_Config[][2]=
{
	{192, 0x0001}		// Enable sequencer --> 192[0]
};
const uint16_t VITA1300_Disable_Sequencer_Config[][2]=
{
	{192, 0x0000}		// Disable sequencer --> 192[0]
};

char oneshot = 0;
int contOneshot = 500;

int main(void)
{
	int i=0;
	int temperatura=0;
	int error=0;
	char state=POWER_OFF;
	char program_flags_0=0;
	char program_flags_1=0;
	char comando = 0;
	char cnt_10us=0, cnt_power_up=0;
	int dato;
	uint16_t *apuntador = (uint16_t*)(FSMC_SRAM_ADDRESS_BK1);
	//GPIO E4 OUTPUT CSK BUS
	RCC->AHB1ENR	|=	0x10;	//Leer pagina 110 libro DM00031020
	GPIOE->MODER 	&=	~(0x00000300);	//Leer pagina 148 libro DM00031020
	GPIOE->MODER 	|=		0x00000100 ;	//Leer pagina 148 libro DM00031020
	GPIOE->OSPEEDR&=	~(0x00000300);	//Leer pagina 149 libro DM00031020
	GPIOE->OSPEEDR|=		0x00000200 ;	//Leer pagina 149 libro DM00031020
	GPIOE->PUPDR	&=	~(0x00000300);	//Leer pagina 149 libro DM00031020
	GPIOE->PUPDR	|=	  0x00000100 ;	//Leer pagina 149 libro DM00031020
	//GPIO G15 OUTPUT CSK BUS
	RCC->AHB1ENR	|=	0x40;	//Leer pagina 110 libro DM00031020
	GPIOG->MODER 	&=	~(0xC0000000);	//Leer pagina 148 libro DM00031020
	GPIOG->MODER 	|=		0x40000000 ;	//Leer pagina 148 libro DM00031020
	GPIOG->OSPEEDR&=	~(0xC0000000);	//Leer pagina 149 libro DM00031020
	GPIOG->OSPEEDR|=		0x80000000 ;	//Leer pagina 149 libro DM00031020
	GPIOG->PUPDR	&=	~(0xC0000000);	//Leer pagina 149 libro DM00031020
	GPIOG->PUPDR	|=	  0x40000000 ;	//Leer pagina 149 libro DM00031020
	//GPIO B1 RESET SENSOR
	RCC->AHB1ENR	|=	0x2;	//Leer pagina 110 libro DM00031020
	GPIOB->MODER 	&=	~(0x0000000C);	//Leer pagina 148 libro DM00031020
	GPIOB->MODER 	|=		0x00000004 ;	//Leer pagina 148 libro DM00031020
	GPIOB->OSPEEDR&=	~(0x0000000C);	//Leer pagina 149 libro DM00031020
	GPIOB->OSPEEDR|=		0x00000008 ;	//Leer pagina 149 libro DM00031020
	GPIOB->PUPDR	&=	~(0x0000000C);	//Leer pagina 149 libro DM00031020
	GPIOB->PUPDR	|=	  0x00000004 ;	//Leer pagina 149 libro DM00031020
	SENSOR_RESET;
	//GPIO D2, D3 3.3 V y 1.8 V
	RCC->AHB1ENR	|=	0x8;	//Leer pagina 110 libro DM00031020
	GPIOD->MODER 	&=	~(0x000000F0);	//Leer pagina 148 libro DM00031020
	GPIOD->MODER 	|=		0x00000050 ;	//Leer pagina 148 libro DM00031020
	GPIOD->OSPEEDR&=	~(0x000000F0);	//Leer pagina 149 libro DM00031020
	GPIOD->OSPEEDR|=		0x000000A0 ;	//Leer pagina 149 libro DM00031020
	GPIOD->PUPDR	&=	~(0x000000F0);	//Leer pagina 149 libro DM00031020
	GPIOD->PUPDR	|=	  0x00000050 ;	//Leer pagina 149 libro DM00031020
	//GPIO C5 Clk 64 MHz
	RCC->AHB1ENR	|=	0x4;	//Leer pagina 110 libro DM00031020
	GPIOC->MODER 	&=	~(0x00000C00);	//Leer pagina 148 libro DM00031020
	GPIOC->MODER 	|=		0x00000400 ;	//Leer pagina 148 libro DM00031020
	GPIOC->OSPEEDR&=	~(0x00000C00);	//Leer pagina 149 libro DM00031020
	GPIOC->OSPEEDR|=		0x00000800 ;	//Leer pagina 149 libro DM00031020
	GPIOC->PUPDR	&=	~(0x00000C00);	//Leer pagina 149 libro DM00031020
	GPIOC->PUPDR	|=	  0x00000400 ;	//Leer pagina 149 libro DM00031020


	/*---------------------------------------------------------------------------*/
	//TIMER 3
	RCC->APB1ENR 	|= 	0x2;
	TIM3->CR1			 =	0x0000;
	TIM3->PSC			 =	839;
	TIM3->ARR			 =	100;
	/*---------------------------------------------------------------------------*/
	//TIMER 2 10us
	RCC->APB1ENR 	|= 	0x1;
	TIM2->CR1			 =	0x0000;
	TIM2->PSC			 =	419;
	TIM2->ARR			 =	1;
	/*---------------------------------------------------------------------------*/
	//UART2 115200bauds/s 
	RCC->AHB1ENR	|=	 0x1;	//Leer pagina 110 libro DM00031020
	RCC->APB1ENR	|=	 0x20000;
	GPIOA->AFR[0]	&= ~(0x0000FF00);
	GPIOA->AFR[0] |=   0x00007700;  // Leer pagina 141 y 153 libro DM00031020
	GPIOA->MODER 	&= ~(0x000000F0);	//Leer pagina 148 libro DM00031020
	GPIOA->MODER  |=   0x000000A0; // Leer pagina 148 libro DM00031020
	GPIOA->OSPEEDR&= ~(0x000000F0);	//Leer pagina 149 libro DM00031020
	GPIOA->OSPEEDR|=   0x000000A0; // Leer pagina 149 libro DM00031020
	GPIOA->PUPDR	&= ~(0x000000F0);	//Leer pagina 149 libro DM00031020
	GPIOA->PUPDR  |=   0x00000050;  // Leer pagina 150 libro DM00031020
	//UART2 
	USART2->CR2=0;        // Leer pagina 652 libro DM00031020
	USART2->CR3=0;       // Leer pagina 653 libro DM00031020
	USART2->CR1=0xC;     // Leer pagina 649 libro DM00031020
  //USART2->BRR=0x1117;   // Leer pagina 619 libro DM00031020
	USART2->BRR=0x016C;
  USART2->CR1 |= 0x2000; // Leer pagina 649 libro DM00031020
	USART2->CR1|=0xC;   // Leer pagina 649 libro DM00031020
	/*---------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------*/
	//FSMC
	RCC->AHB1ENR  |= 0x00000078; 		//Página 110 libro DM00031020
  RCC->AHB3ENR  |= 0x00000001; 		//Página 113 libro DM00031020
  
/*-- GPIOs Configuration -----------------------------------------------------*/
/*
 +-------------------+--------------------+------------------+------------------+
 | PD0  <-> FSMC_D2  | PE0  <-> FSMC_NBL0 | PF0 <-> FSMC_A0  | PG0 <-> FSMC_A10 |
 | PD1  <-> FSMC_D3  | PE1  <-> FSMC_NBL1 | PF1 <-> FSMC_A1  | PG1 <-> FSMC_A11 |
 | PD4  <-> FSMC_NOE |*PE2  <-> FSMC_A23  | PF2 <-> FSMC_A2  | PG2 <-> FSMC_A12 |
 | PD5  <-> FSMC_NWE | PE3  <-> FSMC_A19  | PF3 <-> FSMC_A3  | PG3 <-> FSMC_A13 |
 | PD8  <-> FSMC_D13 |*PE4  <-> FSMC_A20  | PF4 <-> FSMC_A4  | PG4 <-> FSMC_A14 |
 | PD9  <-> FSMC_D14 |*PE5  <-> FSMC_A21  | PF5 <-> FSMC_A5  | PG5 <-> FSMC_A15 |
 | PD10 <-> FSMC_D15 |*PE6  <-> FSMC_A22  | PF12 <-> FSMC_A6 | PG9 <-> FSMC_NE2 |
 | PD11 <-> FSMC_A16 | PE7  <-> FSMC_D4   | PF13 <-> FSMC_A7 | PG10<-> FSMC_NE3	|
 | PD12 <-> FSMC_A17 | PE8  <-> FSMC_D5   | PF14 <-> FSMC_A8 |------------------+
 | PD13 <-> FSMC_A18 | PE9  <-> FSMC_D6   | PF15 <-> FSMC_A9 |
 | PD14 <-> FSMC_D0  | PE10 <-> FSMC_D7   |------------------+
 | PD15 <-> FSMC_D1  | PE11 <-> FSMC_D8   |
 +-------------------| PE12 <-> FSMC_D9   |
                     | PE13 <-> FSMC_D10  |
                     | PE14 <-> FSMC_D11  |
                     | PE15 <-> FSMC_D12  |
                     +--------------------+
*/
	  
	GPIOD->AFR[0]	&= ~(0x00FF00FF);	
  GPIOD->AFR[0] |=   0x00CC00CC;  		// Leer pagina 141 y 153 libro DM00031020
	GPIOD->AFR[1]	&= ~(0xFFFFFFFF);
	GPIOD->AFR[1] |=   0xCCCCCCCC;  		// Leer pagina 141 y 153 libro DM00031020
	GPIOD->MODER  |=   0xAAAA0A0A; 		// Leer pagina 148 libro DM00031020
	GPIOD->OSPEEDR|=   0xFFFF0F0F; 	// Leer pagina 149 libro DM00031020
	GPIOD->OTYPER |=   0;  					// Leer pagina 148 libro DM00031020
	GPIOD->PUPDR  |=   0x0;  					// Leer pagina 150 libro DM00031020

	GPIOE->AFR[0]	 &= ~(0xFF00F0FF);
	GPIOE->AFR[0]  |=   0xCC00C0CC;  		// Leer pagina 141 y 153 libro DM00031020
	GPIOE->AFR[1]	 &= ~(0xFFFFFFFF);
	GPIOE->AFR[1]  |=   0xCCCCCCCC;  		// Leer pagina 141 y 153 libro DM00031020
	GPIOE->MODER   |=   0xAAAA808A; 		// Leer pagina 148 libro DM00031020
	GPIOE->OSPEEDR |=   0xFFFFC0CF; 	// Leer pagina 149 libro DM00031020
	GPIOE->OTYPER  |=   0;  					// Leer pagina 148 libro DM00031020
	GPIOE->PUPDR   |=   0x0;  					// Leer pagina 150 libro DM00031020

	GPIOF->AFR[0]	&= ~(0x00FFFFFF);
  GPIOF->AFR[0]  =   0x00CCCCCC;  		// Leer pagina 141 y 153 libro DM00031020
	GPIOF->AFR[1]	&= ~(0xFFFF0000);
	GPIOF->AFR[1]  =   0xCCCC0000;  		// Leer pagina 141 y 153 libro DM00031020
	GPIOF->MODER  |=   0xAA000AAA; 		// Leer pagina 148 libro DM00031020
	GPIOF->OSPEEDR|=   0xFF000FFF; 	// Leer pagina 149 libro DM00031020
	GPIOF->OTYPER |=   0;  					// Leer pagina 148 libro DM00031020
	GPIOF->PUPDR  |=   0x0;  					// Leer pagina 150 libro DM00031020

	GPIOG->AFR[0]	&= ~(0x00FFFFFF);
	GPIOG->AFR[0]  =   0x00CCCCCC;  		// Leer pagina 141 y 153 libro DM00031020
	GPIOG->AFR[1]	&= ~(0x00000FF0);
	GPIOG->AFR[1] |=  (0x00000CC0);  // Leer pagina 141 y 153 libro DM00031020
	GPIOG->MODER  |=   0x00280AAA; 		// Leer pagina 148 libro DM00031020
	GPIOG->OSPEEDR|=   0x003C0FFF; 	// Leer pagina 149 libro DM00031020
	GPIOG->OTYPER |=   0;  					// Leer pagina 148 libro DM00031020
	GPIOG->PUPDR  |=   0x0;  					// Leer pagina 150 libro DM00031020
	
/*-- FSMC Configuration ------------------------------------------------------*/
  FSMC_Bank1->BTCR[2]  &= ~(0x0008FF7F); // Leer pagina 1255 libro DM00031020
	FSMC_Bank1->BTCR[2]  |=   0x00001090 ; // Leer pagina 1255 libro DM00031020
	FSMC_Bank1->BTCR[3]  &= ~(0x3FFFFFFF); // Leer pagina 1255 libro DM00031020
	FSMC_Bank1->BTCR[3]  |=   0x000320201 ; // Leer pagina 1255 libro DM00031020
	FSMC_Bank1->BTCR[4]  &= ~(0x0008FF7F); // Leer pagina 1255 libro DM00031020
	FSMC_Bank1->BTCR[4]  |=   0x00001090 ; // Leer pagina 1255 libro DM00031020
	FSMC_Bank1->BTCR[5]  &= ~(0x3FFFFFFF); // Leer pagina 1255 libro DM00031020
	FSMC_Bank1->BTCR[5]  |=   0x00020201 ; // Leer pagina 1255 libro DM00031020
	FSMC_BANK_1_SRAM2_WR_ENABLE;
	FSMC_BANK_1_SRAM3_WR_DISABLE;
	FSMC_BANK_1_SRAM2_ENABLE;
	FSMC_BANK_1_SRAM3_ENABLE;
	program_flags_1 |= FLAG_1_SRAM_2_ENABLE;
	program_flags_1 &= ~(FLAG_1_SRAM_3_ENABLE);
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

 /*DCMI GPIO*/
 
 /*
 +----------------------+---------------------+-------------------+------------------+
 | PA4  <-> DCMI_HSYNC  | PB6  <-> DCMI_D5 		| PC6  <-> DCMI_D0  | PE5  <-> DCMI_D6 |
 | PA6  <-> DCMI_PIXCLK | PB7  <-> DCMI_VSYNC | PC7  <-> DCMI_D1  | PE6  <-> DCMI_D7 |
 | 											|										  | PC8  <-> DCMI_D2  | 								 |
 |											|											| PC9  <-> DCMI_D3  | 								 |
 |											|											| PC10 <-> DCMI_D8  | 								 |
 |											|											| PC11 <-> DCMI_D4  | 								 |
 |											|											| PC12 <-> DCMI_D9 	| 					       |
 +----------------------+---------------------+-------------------+------------------+
*/
 
	RCC->AHB2ENR |= 0x01; 					// Leer pagina 112 libro DM00031020. /* Enable DCMI clock */
  RCC->AHB1ENR |= 0x00400017;     //Enable DMA2 CLK, PORT A,B,C,E
	
	GPIOA->AFR[0]	&= ~(0x0F0F0000);	
  GPIOA->AFR[0] |=   0x0D0D0000;  		// Leer pagina 141 y 153 libro DM00031020
	GPIOA->MODER  |=   0x00002200; 		// Leer pagina 148 libro DM00031020
	GPIOA->OSPEEDR|=   0x00003300; 	// Leer pagina 149 libro DM00031020
	GPIOA->PUPDR  |=   0x00001100;  // Leer pagina 150 libro DM00031020
	
	GPIOB->AFR[0]	&= ~(0xFF000000);	
  GPIOB->AFR[0] |=   0xDD000000;  		// Leer pagina 141 y 153 libro DM00031020
	GPIOB->MODER  |=   0x0000A000; 		// Leer pagina 148 libro DM00031020
	GPIOB->OSPEEDR|=   0x0000F000; 	// Leer pagina 149 libro DM00031020
	GPIOB->PUPDR  |=   0x00005000;  // Leer pagina 150 libro DM00031020
	
	GPIOC->AFR[0]	&= ~(0xFF000000);	
  GPIOC->AFR[0] |=   0xDD000000;  		// Leer pagina 141 y 153 libro DM00031020
	GPIOC->AFR[1]	&= ~(0x000FFFFF);	
  GPIOC->AFR[1] |=   0x000DDDDD;  		// Leer pagina 141 y 153 libro DM00031020	
	GPIOC->MODER  |=   0x02AAA000; 		// Leer pagina 148 libro DM00031020
	GPIOC->OSPEEDR|=   0x03FFF000; 	// Leer pagina 149 libro DM00031020
	GPIOC->PUPDR  |=   0x01555000;  // Leer pagina 150 libro DM00031020
	
	GPIOE->AFR[0]	&= ~(0x0FF00000);	
  GPIOE->AFR[0] |=   0x0DD00000;  		// Leer pagina 141 y 153 libro DM00031020
	GPIOE->MODER  |=   0x00002800; 		// Leer pagina 148 libro DM00031020
	GPIOE->OSPEEDR|=   0x00003C00; 	// Leer pagina 149 libro DM00031020
	GPIOE->PUPDR  |=   0x00001400;  // Leer pagina 150 libro DM00031020
	
	DCMI->CR &= ~(0x7FF);  				// Leer pagina 282 libro DM00031020.  
	DCMI->CR |= 0x0002;  					// Leer pagina 282 libro DM00031020. SNAPSHOT_MODE
	DCMI->CR |= 0x0004; 					// Leer página 281 Libro DM00031020. Activar CROP Feature
	DCMI->CR |= 0x0020; 					// Leer pagina 282 libro DM00031020. Pixel clock polarity = Rising
	/*->->*/
	DCMI->CR &= ~(0x00000C00); //  Leer pagina 282 libro DM00031020. 8 Bits
	DCMI->CR |= 0x0400;					  // Leer pagina 282 libro DM00031020. 10 Bits	
/*Iniciar la captura de la imagen luego de la segunda linea de datos (las dos primeras lineas son negros, por lo tanto la ignoramos)*/
	DCMI->CWSTRTR = 0x00020000;
///*Realizar la captura de un frame de 300x400 pixeles*/ 	
//	DCMI->CWSIZER = 0x12B0190; //Leer página 277 Libro DM00031020. Largo y ancho de la ventana	
///*Realizar la captura de un frame de 600x800 pixeles*/ 	
//	DCMI->CWSIZER = 0x2570320; //Leer página 277 Libro DM00031020. Largo y ancho de la ventana	
/*Realizar la captura de un frame de 600x800 pixeles*/ 	
	DCMI->CWSIZER = 0x3200320; //Leer página 277 Libro DM00031020. Largo y ancho de la ventana
//	
//  /* Configuración del DMA2 Stream1 */
//  //	/* DMA Config */
	DMA2_Stream1->CR  = 0; // Leer pagina 185 libro DM00031020. Reset la configuracion registro
	DMA2_Stream1->NDTR  = 0; // Leer pagina 188 libro DM00031020. Numero de datos a trasnferir
	DMA2_Stream1->PAR  = 0; //  Leer pagina 188 libro DM00031020. Direccion base de memoria de donde los bits seran leidos o escritos
	DMA2_Stream1->M0AR = 0; //  Leer pagina 188 libro DM00031020. 
	DMA2_Stream1->M1AR = 0; //  Leer pagina 189 libro DM00031020. 	
  DMA2_Stream1->FCR = 0x21; //  Leer pagina 190 libro DM00031020. Resetea el registro de control de FIFO	

	DMA2_Stream1->CR |= 0x02000000; //Channel 1 selected
	DMA2_Stream1->CR |= 0x00040000; //Double buffer mode
	DMA2_Stream1->CR |= 0x00020000; //Priority high	
	DMA2_Stream1->CR |= 0x00002000; //Memory data size half-word (16 bits)
	DMA2_Stream1->CR |= 0x00001000; //Peripheral data size word (32 bits)
	DMA2_Stream1->CR |= 0x00000400; //Memory address pointer is incremented after each data transfer
	//DMA2_Stream1->CR |= 0x00000100; //Circular Mode Enable		
	DMA2_Stream1->FCR &= ~(0x00000007);  // Leer pagina 190  libro DM00031020. 
	DMA2_Stream1->FCR |= 0x00000006;  // Leer pagina 190  libro DM00031020. Direct Mode disable and FIFO threshold 3/4
	DMA2_Stream1->PAR = DCMI_DR_ADDRESS; // Leer pagina 188  libro DM00031020.
	DMA2_Stream1->NDTR = 0xFFFF	;  // Leer pagina 188  libro DM00031020.
  DMA2_Stream1->M0AR = FSMC_SRAM_ADDRESS_BK1 ; // Leer pagina 188  libro DM00031020.
	//DMA2_Stream1->M0AR = (uint32_t)frame_buffer;
	DMA2_Stream1->M1AR = (FSMC_SRAM_ADDRESS_BK1 +(0xFFFF * 4 )) ;
	
//	NVIC->IP[57] = 0xD0;
//	NVIC->ISER[1] = 0x2000000;	

/*---------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------*/
	//SPI CONFIG MASTER MODE
	/*
	+----------------------------------------------------------+
	|SPI1_SCK A5 	SPI1_MOSI A7		SPI1_MISO B4		SPI1_NSS A15 |
	+----------------------------------------------------------+
	*/
	RCC->AHB1ENR  |=   0x3;
	GPIOA->MODER	&= ~(0xC000CC00);
	GPIOA->MODER	|=	 0x40004400;
	GPIOA->OSPEEDR&= ~(0xC000CC00);
	GPIOA->OSPEEDR|=   0x80008800;
	GPIOA->OTYPER &= ~(0x000080A0);	
	GPIOA->PUPDR	&= ~(0xC000CC00);
	GPIOA->PUPDR	|=   0x80008800;
	
	GPIOB->MODER	&= ~(0x00000300);
	GPIOB->OSPEEDR&= ~(0x00000300);
	GPIOB->OSPEEDR|=   0x00000100;
	GPIOB->OTYPER &= ~(0x00000010);
	GPIOB->PUPDR	&= ~(0x00000300);
	GPIOB->PUPDR	|=   0x00000200;
	
	/*---------------------------------------------------------------------------*/
	//RTC
	/*---------------------------------------------------------------------------*/
	//HABILAR ESCRITURA RTC
	PWR->CR |= 0x0100;//
	RCC->BDCR |=0x300;//
	RCC->CFGR |= 0x80000; //CLK 1MHz
	for(i=0;i<=100;i++);
	RCC->BDCR |=0x8000;//
	//DESACTIVAR PROTECCIÓN
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	//ENTRAR A MODO DE INICIALIZACIÓN
  RTC->ISR = 0x80;
  //ESPERAR INGRESO A MODO DE INICIALIZACIÓN
	while((RTC->ISR & 0x40)==0);	
	//CONFIGURAR VELOCIDAD 1hz
	RTC->PRER =7999;
	RTC->PRER |= 0x7C0000;
	//CONFIGURAR TIEMPO Y FECHA
	RTC->TR = 0x170500;
	RTC->DR	=	0x147015;
	//ACTIVAR PROTECCIONES
	RTC->ISR &=  ~(0x80);	
	RTC->WPR = 0xFF;
	PWR->CR &= ~(0x0100);//	
	//FIN CONFIGURACIÓN RTC
	/*----------------------------------------------------------------------------*/
	TIMER_3_DISABLE;
	//USART2->DR = '\f';
	while(USART2_TX_FLAG==0);
	USART2_TX_FLAG_R;
	i=0;
	
	//HABILITAR TIMER 2
	TIMER_2_ENABLE; // Leer pagina 392 libro DM00031020
	//SPI ENABLE
	SPI_1_DISABLE;
	//SENSOR EN RESET
	SENSOR_RESET;
	SENSOR_3V3_DISABLE;
	SENSOR_1V8_DISABLE;
	SPI_1_CLK_FALL;
	/*----------------------------------------------------------------------------*/
	/*----------------------------------------------------------------------------*/
  for (i=0;i<=0x100000;i++)
	{
		apuntador[i]=0;
	}
	while(1)
	{		
		if(TIMER_2_PREGUNTA)
		{
			TIMER_2_RESET;
			cnt_10us++;
			Command_Handler();
			dato = frame_buffer[0];
			if(dato == 23)
			{
				frame_buffer[1] = 0;
			}
		}	
		if(USART2_RX_FLAG)
		{
				if((program_flags_0 & FLAG_0_POWER_UP)|(program_flags_0 & FLAG_0_POWER_OFF ))
				{
					comando = USART2_RX_DATA;
					if(USART2_TX_FLAG)
					{
						USART2_TX_FLAG_R;						
						USART2_TX_DATA = 'x';
					}
					if (comando == 'l' | comando == 'L')
					{
						program_flags_0 = 0;
						SENSOR_3V3_DISABLE;
						TIMER_2_RESET;
						while(TIMER_2_PREGUNTA==0);
						TIMER_2_RESET;
						SENSOR_1V8_DISABLE;
						state = POWER_OFF;
					}	
					else if (comando == 'k' | comando == 'K')
					{
						program_flags_0 |= FLAG_0_CLK_SENSOR_EN;
					}	
				}
				else
				{
					comando = USART2_RX_DATA;
					cnt_10us = 0;
					if(comando == '1')
					{
						program_flags_0 |= FLAG_0_POWER_UP;
					}
					else if (comando == '2')
					{
						program_flags_0 |= FLAG_0_POWER_OFF;
					}
					else if (comando == 'r' | comando == 'R')
					{
						program_flags_0 &= ~(FLAG_0_POWER_OFF|FLAG_0_POWER_UP);
						state = LOW_POWER_STANDBY;
					}
					else if (comando == 'c' | comando == 'C')
					{
						program_flags_0 |= FLAG_0_CAPTURE_IMAGE;
					}
					else if (comando == 't' | comando == 'T')
					{
						if((program_flags_0 & FLAG_0_POWER_OFF)|(program_flags_0 & FLAG_0_POWER_UP))
						{
								program_flags_0 |= FLAG_0_SENSOR_TEMP;
						}
						else if(state<LOW_POWER_STANDBY)
						{
								program_flags_0 |= FLAG_0_SENSOR_TEMP;
						}
						else
						{
								temperatura = SPI_Config_Sensor_Read (2);
								if(temperatura)
								USART2_TX_DATA = 'T';
								else
								USART2_TX_DATA = 't';	
						}
					}									
					else
					{
						if(USART2_TX_FLAG)
						{
							USART2_TX_FLAG_R;
							USART2_TX_DATA = 'x';							
						}	
					}
				}
		}
		switch (state)
		{
/*----------------------------------------------------------------------------*/			
			case POWER_OFF:				
				if(program_flags_0 & FLAG_0_POWER_UP)
				{
					if(cnt_10us >= 2)
					{
						cnt_10us =0;
						/*Secuencia de arranque Sensor*/
						/*Power up sequence*/
						/* Por problemas en el oscilador se modifica provisionalmente esta parte del código*/
						/*
//							if(cnt_power_up==0)																				
//								SENSOR_1V8_ENABLE;//HABILITAR 1.8V
//							else if(cnt_power_up==1)
//								SENSOR_3V3_ENABLE;//HABILITAR 3.3V
//							else if(cnt_power_up==2)
//								SENSOR_CLOCK_ENABLE;//HABILITAR CLK 64MHz
//							else if(cnt_power_up==3)
//								SENSOR_SET;
//							else if(cnt_power_up==4)
//							{
//								USART2_TX_DATA = 'E';
//								program_flags_0 &= ~(FLAG_0_POWER_UP);
//								state = LOW_POWER_STANDBY;
//							}
//						cnt_power_up++;*/
						
/*Segmento de código a eliminar cuando el problema del oscilador haya sido solucionado*/
/*----------------------------------------------------------------------------------------------------------*/						
						if(cnt_power_up==0)																				
						{SENSOR_1V8_ENABLE;cnt_power_up++;}//HABILITAR 1.8V
						else if(cnt_power_up==1)
						{SENSOR_3V3_ENABLE;cnt_power_up++;}//HABILITAR 3.3V
						else if(program_flags_0 & FLAG_0_CLK_SENSOR_EN)
						{SENSOR_CLOCK_ENABLE;cnt_power_up++;program_flags_0 &= ~(FLAG_0_CLK_SENSOR_EN);}//HABILITAR CLK 64MHz
						else if(cnt_power_up==3)
						{SENSOR_SET;cnt_power_up++;}
						else if(cnt_power_up==4)
						{
							//USART2_TX_DATA = 'E';
							program_flags_0 &= ~(FLAG_0_POWER_UP);
							state = LOW_POWER_STANDBY;
						}
/*----------------------------------------------------------------------------------------------------------*/						
					}
				}
				else if(program_flags_0 & FLAG_0_POWER_OFF)
				{
					program_flags_0 &= ~(FLAG_0_POWER_OFF);
				}
				if(state != POWER_OFF)
				{
					cnt_power_up = 0;
					cnt_10us = 0;
				}				
			break;
/*----------------------------------------------------------------------------*/				
			case LOW_POWER_STANDBY:
				if(program_flags_0 & FLAG_0_POWER_UP)
				{
					/*Secuencia Enable Clock Management - Part 1*/
					SPI_Config_Sensor_Send (VITA1300_EnableClockManagment_1_Config[cnt_power_up][0],VITA1300_EnableClockManagment_1_Config[cnt_power_up][1]);
					error = SPI_Config_Sensor_Read (VITA1300_EnableClockManagment_1_Config[cnt_power_up][0]);
					if(error == VITA1300_EnableClockManagment_1_Config[cnt_power_up][1])
					{
//						USART2_TX_DATA = 'H';
//						while(USART2_TX_FLAG==0);
//						USART2_TX_FLAG_R;
					}
					else
					{
						USART2_TX_DATA = 'h';
						while(USART2_TX_FLAG==0);
						USART2_TX_FLAG_R;
					}
					cnt_power_up++;
					if(cnt_power_up >= 4)
					{
						cnt_power_up=0;
						//USART2_TX_DATA = '1';
						program_flags_0 &= ~(FLAG_0_POWER_UP);
						state = STANDBY_1;
					}
				}
				else if(program_flags_0 & FLAG_0_POWER_OFF)
				{
					if(cnt_10us >= 2)
					{
						cnt_10us =0;
						/*Secuencia de apagado Sensor*/
//						if(cnt_power_up==0)
//							SENSOR_RESET;
//						else if(cnt_power_up==1)
//							SENSOR_CLOCK_DISABLE;//DESHABILITAR CLK 64MHz
//						else if(cnt_power_up==2)
//							SENSOR_3V3_DISABLE;//DESHABILITAR 3.3V
//						else if(cnt_power_up==3)
//							SENSOR_1V8_DISABLE;//DESHABILITAR 1.8V
//						else if(cnt_power_up==4)
//						{
//							USART2_TX_DATA = 'A';
//							program_flags_0 &= ~(FLAG_0_POWER_OFF);
//							state = POWER_OFF;
//						}	
//						cnt_power_up++;
/*Secuencida de código a eliminar al solucionar el problema del oscilador*/
/*----------------------------------------------------------------------------------------------------------------------*/
						if(cnt_power_up==0)
						{SENSOR_RESET;cnt_power_up++;}
						else if(program_flags_0 & FLAG_0_CLK_SENSOR_EN)
						{SENSOR_CLOCK_DISABLE;cnt_power_up++;program_flags_0 &= ~(FLAG_0_CLK_SENSOR_EN);}//DESHABILITAR CLK 64MHz
						else if(cnt_power_up==2)
						{SENSOR_3V3_DISABLE;cnt_power_up++;}//DESHABILITAR 3.3V
						else if(cnt_power_up==3)
						{SENSOR_1V8_DISABLE;cnt_power_up++;}//DESHABILITAR 1.8V
						else if(cnt_power_up==4)
						{
							USART2_TX_DATA = 'A';
							program_flags_0 &= ~(FLAG_0_POWER_OFF);
							state = POWER_OFF;
						}		
/*----------------------------------------------------------------------------------------------------------------------*/						
					}
				}	

				if(state != LOW_POWER_STANDBY)
				{
					cnt_power_up = 0;
					cnt_10us = 0;
				}				
			break;
/*----------------------------------------------------------------------------*/					
			case 	STANDBY_1:
				if(program_flags_0 & FLAG_0_POWER_UP)
				{
					/*Secuencia Enable Clock Management - Part 2*/
					SPI_Config_Sensor_Send (VITA1300_EnableClockManagment_2_Config[cnt_power_up][0],VITA1300_EnableClockManagment_2_Config[cnt_power_up][1]);
					error = SPI_Config_Sensor_Read (VITA1300_EnableClockManagment_2_Config[cnt_power_up][0]);
					if(error == VITA1300_EnableClockManagment_2_Config[cnt_power_up][1])
					{
//						USART2_TX_DATA = 'H';
//						while(USART2_TX_FLAG==0);
//						USART2_TX_FLAG_R;
					}
					else
					{
						USART2_TX_DATA = 'h';
						while(USART2_TX_FLAG==0);
						USART2_TX_FLAG_R;
					}
					cnt_power_up++;
					if(cnt_power_up >= 3)
					{
						cnt_power_up=0;
						//USART2_TX_DATA = '2';
						SPI_Config_Sensor_Send(256,0x6300);//->800
						SPI_Config_Sensor_Send(258,799);//->800						
//						SPI_Config_Sensor_Send(256,0x6300);//->800
//						SPI_Config_Sensor_Send(258,599);//->600
//						SPI_Config_Sensor_Send(256,0x3100);//->400
//						SPI_Config_Sensor_Send(258,299);//->300
//						SPI_Config_Sensor_Send(256,0x6331);//->400 ---> 0x64*8 = 800   0x32*8 = 400  800-400 = 400
//						SPI_Config_Sensor_Send(257,362);//->362
//						SPI_Config_Sensor_Send(258,661);//->661 ---> 662-362 = 300 pixeles

						program_flags_0 &= ~(FLAG_0_POWER_UP);
						state = INTERMEDIATE_STANDBY;
					}
				}
				else if(program_flags_0 & FLAG_0_POWER_OFF)
				{
					/*Secuencia Disable Clock Management - Part 1*/
					SPI_Config_Sensor_Send (VITA1300_DisableClockManagment_1_Config[cnt_power_up][0],VITA1300_DisableClockManagment_1_Config[cnt_power_up][1]);
					error = SPI_Config_Sensor_Read (VITA1300_DisableClockManagment_1_Config[cnt_power_up][0]);
					if(error == VITA1300_DisableClockManagment_1_Config[cnt_power_up][1])
					{
//						USART2_TX_DATA = 'H';
//						while(USART2_TX_FLAG==0);
//						USART2_TX_FLAG_R;
					}
					else
					{
//						USART2_TX_DATA = 'h';
//						while(USART2_TX_FLAG==0);
//						USART2_TX_FLAG_R;
					}
					cnt_power_up++;
					if(cnt_power_up >= 3)
					{
						cnt_power_up=0;
						//USART2_TX_DATA = '1';
						program_flags_0 &= ~(FLAG_0_POWER_OFF);
						state = LOW_POWER_STANDBY;
					}					
				}
				if(state != STANDBY_1)
				{
					cnt_power_up = 0;
					cnt_10us = 0;
				}	
			break;
/*----------------------------------------------------------------------------*/				
			case INTERMEDIATE_STANDBY:
				if(program_flags_0 & FLAG_0_POWER_UP)
				{
					/*Required Register Upload*/
					SPI_Config_Sensor_Send (VITA1300_Required_Register_Upload_Config[cnt_power_up][0],VITA1300_Required_Register_Upload_Config[cnt_power_up][1]);
					error = SPI_Config_Sensor_Read (VITA1300_Required_Register_Upload_Config[cnt_power_up][0]);
					if(error == VITA1300_Required_Register_Upload_Config[cnt_power_up][1])
					{
//						USART2_TX_DATA = 'H';
//						while(USART2_TX_FLAG==0);
//						USART2_TX_FLAG_R;
					}
					else
					{
						USART2_TX_DATA = 'h';
						while(USART2_TX_FLAG==0);
						USART2_TX_FLAG_R;
					}	
					cnt_power_up++;
					if(cnt_power_up >= 60)
					{
						cnt_power_up=0;
						//USART2_TX_DATA = '3';
						program_flags_0 &= ~(FLAG_0_POWER_UP);
						state = STANDBY_2;
					}						
				}
				else if(program_flags_0 & FLAG_0_POWER_OFF)
				{
					program_flags_0 &= ~(FLAG_0_POWER_OFF);									
				}
				if(state != INTERMEDIATE_STANDBY)
				{
					cnt_power_up = 0;
					cnt_10us = 0;
				}	
			break;
/*----------------------------------------------------------------------------*/				
			case STANDBY_2:
				if(program_flags_0 & FLAG_0_POWER_UP)
				{
					/*Soft Power-Up*/
					SPI_Config_Sensor_Send (VITA1300_SofT_PowerUp_Config[cnt_power_up][0],VITA1300_SofT_PowerUp_Config[cnt_power_up][1]);
					error = SPI_Config_Sensor_Read (VITA1300_SofT_PowerUp_Config[cnt_power_up][0]);
					if(error == VITA1300_SofT_PowerUp_Config[cnt_power_up][1])
					{
//						USART2_TX_DATA = 'H';
//						while(USART2_TX_FLAG==0);
//						USART2_TX_FLAG_R;
					}
					else
					{
						USART2_TX_DATA = 'h';
						while(USART2_TX_FLAG==0);
						USART2_TX_FLAG_R;
					}
					cnt_power_up++;
					if(cnt_power_up >= 7)
					{
						cnt_power_up=0;
						//USART2_TX_DATA = '4';
						program_flags_0 &= ~(FLAG_0_POWER_UP);
						state = IDLE;
					}						
				}
				else if(program_flags_0 & FLAG_0_POWER_OFF)
				{
					/*Secuencia Disable Clock Management - Part 2*/
					SPI_Config_Sensor_Send (VITA1300_DisableClockManagment_2_Config[cnt_power_up][0],VITA1300_DisableClockManagment_2_Config[cnt_power_up][1]);
					error = SPI_Config_Sensor_Read (VITA1300_DisableClockManagment_2_Config[cnt_power_up][0]);
					if(error == VITA1300_DisableClockManagment_2_Config[cnt_power_up][1])
					{
//						USART2_TX_DATA = 'H';
//						while(USART2_TX_FLAG==0);
//						USART2_TX_FLAG_R;
					}
					else
					{
//						USART2_TX_DATA = 'h';
//						while(USART2_TX_FLAG==0);
//						USART2_TX_FLAG_R;
					}
					cnt_power_up++;
					if(cnt_power_up >= 3)
					{
						cnt_power_up=0;
						//USART2_TX_DATA = '2';
						program_flags_0 &= ~(FLAG_0_POWER_OFF);
						state = STANDBY_1;
					}									
				}
				if(state != STANDBY_2)
				{
					cnt_power_up = 0;
					cnt_10us = 0;
				}					
			break;
/*----------------------------------------------------------------------------*/					
			case IDLE:
				if(program_flags_0 & FLAG_0_POWER_UP)
				{
					/*Enable Sequencer*/
					SPI_Config_Sensor_Send (VITA1300_Enable_Sequencer_Config[cnt_power_up][0],VITA1300_Enable_Sequencer_Config[cnt_power_up][1]);
					error = SPI_Config_Sensor_Read (VITA1300_Enable_Sequencer_Config[cnt_power_up][0]);
					if(error == VITA1300_Enable_Sequencer_Config[cnt_power_up][1])
					{
//						USART2_TX_DATA = 'H';
//						while(USART2_TX_FLAG==0);
//						USART2_TX_FLAG_R;
					}
					else
					{
						USART2_TX_DATA = 'h';
						while(USART2_TX_FLAG==0);
						USART2_TX_FLAG_R;
					}
					cnt_power_up++;
					if(cnt_power_up >= 1)
					{
						cnt_power_up=0;
						USART2_TX_DATA = '5';
						program_flags_0 &= ~(FLAG_0_POWER_UP);
						state = RUNNING;
					}						
				}
				else if(program_flags_0 & FLAG_0_POWER_OFF)
				{
					/*Secuencia Soft Power Down*/
					SPI_Config_Sensor_Send (VITA1300_SofT_PowerDown_Config[cnt_power_up][0],VITA1300_SofT_PowerDown_Config[cnt_power_up][1]);
					error = SPI_Config_Sensor_Read (VITA1300_SofT_PowerDown_Config[cnt_power_up][0]);
					if(error == VITA1300_SofT_PowerDown_Config[cnt_power_up][1])
					{
//						USART2_TX_DATA = 'H';
//						while(USART2_TX_FLAG==0);
//						USART2_TX_FLAG_R;
					}
					else
					{
//						USART2_TX_DATA = 'h';
//						while(USART2_TX_FLAG==0);
//						USART2_TX_FLAG_R;
					}
					cnt_power_up++;
					if(cnt_power_up >= 6)
					{
						cnt_power_up=0;
						//USART2_TX_DATA = '3';
						program_flags_0 &= ~(FLAG_0_POWER_OFF);
						state = STANDBY_2;
					}									
				}
				if(state != IDLE)
				{
					cnt_power_up = 0;
					cnt_10us = 0;
				}					
			break;
/*----------------------------------------------------------------------------*/				
			case RUNNING:
				if(program_flags_0 & FLAG_0_POWER_UP)
				{
					program_flags_0 &= ~(FLAG_0_POWER_UP);	
				}
				else if(program_flags_0 & FLAG_0_POWER_OFF)
				{
					/*Disable sequencer*/
					SPI_Config_Sensor_Send (VITA1300_Disable_Sequencer_Config[cnt_power_up][0],VITA1300_Disable_Sequencer_Config[cnt_power_up][1]);
					error = SPI_Config_Sensor_Read (VITA1300_Disable_Sequencer_Config[cnt_power_up][0]);
					if(error == VITA1300_Disable_Sequencer_Config[cnt_power_up][1])
					{
//						USART2_TX_DATA = 'H';
//						while(USART2_TX_FLAG==0);
//						USART2_TX_FLAG_R;
					}
					else
					{
//						USART2_TX_DATA = 'h';
//						while(USART2_TX_FLAG==0);
//						USART2_TX_FLAG_R;
					}
					cnt_power_up++;
					if(cnt_power_up >= 1)
					{
						cnt_power_up=0;
						//USART2_TX_DATA = '4';
						program_flags_0 &= ~(FLAG_0_POWER_OFF);
						state = IDLE;
					}									
				}
				if(state != RUNNING)
				{
					cnt_power_up = 0;
					cnt_10us = 0;
				}	
				program_flags_0 = Command_Handler_Recepcion(program_flags_0);
			break;	
/*----------------------------------------------------------------------------*/				
		}//End case
	}//End while
}


void SystemInit(void)
{
	 int a=0;
   RCC->CR = 0x01;  // Leer pagina 93 libro DM00031020
	 RCC->CR |= 0x10000; // Leer pagina 93 libro DM00031020
	 while((RCC->CR & 0x20000)==0); // Leer pagina 93 libro DM00031020
	 RCC->APB1ENR = 0x10000000; // Leer pagina 113 libro DM00031020
	 RCC->CFGR = 0x00009400; // Leer pagina 97 libro DM00031020
	 RCC->PLLCFGR = 0x07405408; // Leer pagina 95 libro DM00031020
	 RCC->CR |= 0x01000000; // Leer pagina 93 libro DM00031020
	 while((RCC->CR & 0x02000000)==0); // Leer pagina 93 libro DM00031020
	 FLASH->ACR = (0x00000605); // Leer pagina 55 libro DM00031020
	 RCC->CFGR &= 0xFFFFFFFC; // Leer pagina 97 libro DM00031020
	 RCC->CFGR |= 2; // Leer pagina 97 libro DM00031020
	 for (a=0;a<=500;a++);
}

void SPI_Config_Sensor_Send (int addr,int data)
{
	int trans_data=0,i=0,bit2send=0;
	trans_data = ((0x1FF & addr)<<17) | 0x10000 | ((data & 0xFFFF));
	SPI_1_ENABLE;
	TIMER_3_ENABLE;
	TIMER_3_RESET;
	SPI_1_CLK_FALL;
	while(TIMER_3_PREGUNTA==0);
	TIMER_3_RESET;
	while(TIMER_3_PREGUNTA==0);
	TIMER_3_RESET;
	for(i=25;i>=0;i--)
	{
			bit2send = ((trans_data>>i)&1) << 7;				
			SPI_1_CLK_FALL;
			if(bit2send)
			{
				GPIOA->BSRRL |= bit2send;
			}
			else
			{
				GPIOA->BSRRH |= 0x80;
			}	
			while(TIMER_3_PREGUNTA==0);
			TIMER_3_RESET;
			if(bit2send)
			{
				GPIOA->BSRRL |= bit2send;
			}
			else
			{
				GPIOA->BSRRH |= 0x80;
			}		
			SPI_1_CLK_RISE;
			while(TIMER_3_PREGUNTA==0);
			TIMER_3_RESET;
	}
	SPI_1_CLK_FALL;
	GPIOA->BSRRH |= 0x80;
	while(TIMER_3_PREGUNTA==0);
	TIMER_3_RESET;
	while(TIMER_3_PREGUNTA==0);
	TIMER_3_RESET;
	SPI_1_DISABLE;
	TIMER_3_DISABLE;
}

int SPI_Config_Sensor_Read (int addr)
{
	int trans_data=0,i=0,bit2send=0;
	char bit_read=0;
	int  data_read=0;
	trans_data = (((0x1FF & addr)<<1) & 0x3FE);
	SPI_1_ENABLE;
	TIMER_3_ENABLE;
	TIMER_3_RESET;
	SPI_1_CLK_FALL;
	while(TIMER_3_PREGUNTA==0);
	TIMER_3_RESET;
	while(TIMER_3_PREGUNTA==0);
	TIMER_3_RESET;
	for(i=9;i>=0;i--)
	{
			bit2send = ((trans_data>>i)&1) << 7;				
			SPI_1_CLK_FALL;
			if(bit2send)
			{
				GPIOA->BSRRL |= bit2send;
			}
			else
			{
				GPIOA->BSRRH |= 0x80;
			}	
			while(TIMER_3_PREGUNTA==0);
			TIMER_3_RESET;
			if(bit2send)
			{
				GPIOA->BSRRL |= bit2send;
			}
			else
			{
				GPIOA->BSRRH |= 0x80;
			}		
			SPI_1_CLK_RISE;
			while(TIMER_3_PREGUNTA==0);
			TIMER_3_RESET;
	}
	SPI_1_CLK_FALL;
	GPIOA->BSRRH |= 0x80;
	for(i=15;i>=0;i--)
	{
		while(TIMER_3_PREGUNTA==0);
		TIMER_3_RESET;
		SPI_1_CLK_RISE;
		while(TIMER_3_PREGUNTA==0);
		TIMER_3_RESET;
		SPI_1_CLK_FALL;
		bit_read = (GPIOB->IDR & 0x0010)>>4;
		data_read |= bit_read << i; 
	}
	SPI_1_CLK_FALL;
	while(TIMER_3_PREGUNTA==0);
	TIMER_3_RESET;
	while(TIMER_3_PREGUNTA==0);
	TIMER_3_RESET;
	SPI_1_DISABLE;
	TIMER_3_DISABLE;
	return data_read;
}

char Command_Handler_Recepcion(char flag)
{
	char mem_disp;
	char flag_capture=0;
	
	flag_capture = flag;
	if(contOneshot >= 5000000)
	{
		oneshot =0;
		if(flag_capture & FLAG_0_CAPTURE_IMAGE)
		{	
			if(oneshot==0)
			{
				mem_disp = escribir_fifo(CAPTURAR);
				mem_disp = escribir_fifo(IMAGEN_SOLICITADA);
				oneshot=1;
				contOneshot = 0;
			}
			flag_capture &= ~(FLAG_0_CAPTURE_IMAGE);
			return flag_capture;
		}
	}
	else
	{
		contOneshot++;
	}
	return flag_capture;
}

void Command_Handler( void )
{
	var_datos el_datico;
	el_datico	 = leer_cola();	
	if(el_datico.ack )
	{
		 switch(el_datico.contenido )
			{
			case CAPTURAR:
				 
					Capturar_Imagen();
			break;
			case IMAGEN_SOLICITADA:
			
					Enviar_Imagen();
			default:
					USART2->DR = 0x15;
					while(!(USART2->SR&0x40));
		}
	}
}

void Capturar_Imagen (void)
{
	/* Enable DMA transfer */
		DMA2->LIFCR |= 0x0F7D0F7D; //Resetear todas las banderas de interrupcion
    DMA2_Stream1->CR |= 0x01;
		while(!(DMA2_Stream1->CR & 0x01));
    /* Enable DCMI interface */
    DCMI->CR |= 0x4000;
    /* Start Image capture */
    DCMI->CR |= 0x0001;
	  //DCMI->CR &= ~(0x01);	
	
	/*---------New Buffer address-----------*/
	while((DMA2_Stream1->CR & 0x80000) == 0);	// Leer Pagina 281 de Libro DM00031020
	DMA2_Stream1->M0AR = (FSMC_SRAM_ADDRESS_BK1 + 2*(0xFFFF * 4 )) ; // Leer pagina 188  libro DM00031020.
	while((DMA2_Stream1->CR & 0x80000));	// Leer Pagina 281 de Libro DM00031020
	DMA2_Stream1->M1AR = (FSMC_SRAM_ADDRESS_BK1 + 3*(0xFFFF * 4 )) ; // Leer pagina 188  libro DM00031020.
	
		while(DCMI->CR & 0x01);	// Leer Pagina 281 de Libro DM00031020
//		while((DMA2_Stream1->CR & 0x01));	
	  DCMI->CR &= ~(0x01);											// Leer Pagina 281 de Libro DM00031020
//		while(DCMI->CR & 0x01);
		DCMI->CR &= ~(0x4000);
    DMA2_Stream1->CR &= ~(0x01);	
		DMA2_Stream1->M0AR = FSMC_SRAM_ADDRESS_BK1 ; // Leer pagina 188  libro DM00031020.
		DMA2_Stream1->M1AR = (FSMC_SRAM_ADDRESS_BK1 +(0xFFFF * 4 )) ;
}

void Enviar_Imagen(void)
{
	int a = 0, i=0;
	int numero_datos = 480000;
	uint16_t *apuntador1 = (uint16_t*)(FSMC_SRAM_ADDRESS_BK1); 
	for (a=0;a<=50000000;a++);
	
  while (!(USART2->SR & 0x0080));						// Leer pagina 646 libro DM00031020.

	for(i=0;i<numero_datos;i++)
	{
		/*Trasmisión de pixeles de 10 bits*/
		while (!(USART2->SR & 0x0080));					// Leer pagina 646 libro DM00031020.	
		USART2->DR = (apuntador1[i]>>2) & 0xFF;	// Leer pagina 648 libro DM00031020.
		while (!(USART2->SR & 0x0080));					// Leer pagina 646 libro DM00031020.
		/*Transmisión de pixeles de 8 bits*/
//			while (!(USART2->SR & 0x0080));	
//			USART2->DR = (apuntador1[i] & 0xFF);		
//			while (!(USART2->SR & 0x0080));	
//			USART2->DR = ((apuntador1[i]>>8) & 0xFF);
//			while (!(USART2->SR & 0x0080));	
//			USART2->DR = (frame_buffer[i] & 0xFF);		
//			while (!(USART2->SR & 0x0080));	
//			USART2->DR = ((frame_buffer[i]>>8) & 0xFF);
//		  while (!(USART2->SR & 0x0080));	
//			USART2->DR = ((frame_buffer[i]>>16) & 0xFF);
//			while (!(USART2->SR & 0x0080));	
//			USART2->DR = ((frame_buffer[i]>>24) & 0xFF);		
	}
	while (!(USART2->SR & 0x0080));						// Leer pagina 646 libro DM00031020.
}
