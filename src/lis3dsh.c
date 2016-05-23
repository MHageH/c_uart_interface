#include "lis3dsh.h"

void spi_setup(void){
	rcc_periph_clock_enable(RCC_GPIOA);
	/* set SPI pins as CLK, MOSI, MISO */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);
	/* Push Pull, Speed 100 MHz */
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO5 | GPIO6 | GPIO7);
	/* Alternate Function: SPI1 */
	gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);

	/* Enable GPIOE clock. */
	rcc_periph_clock_enable(RCC_GPIOE);
	/* set CS as OUTPUT */
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);
	/* Push Pull, Speed 100 MHz */
	gpio_set_output_options(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO3);
	/* set CS high */
	gpio_set(GPIOE, GPIO3);

	// USART1 setup
	rcc_periph_clock_enable (RCC_USART1);

  	nvic_enable_irq (NVIC_USART1_IRQ);	
  	gpio_mode_setup (GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);	//GPD8 : Tx send from STM32 to ext
  	gpio_mode_setup (GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);	//GPD9 : Rx recieve from ext to STM32
  	gpio_set_output_options (GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO10);
  	gpio_set_af (GPIOA, GPIO_AF7, GPIO9);
  	gpio_set_af (GPIOA, GPIO_AF7, GPIO10);
  	// Setup USART1 parameters. 
  	usart_set_baudrate (USART1, 57600);
  	usart_set_databits (USART1, 8);
  	usart_set_stopbits (USART1, USART_STOPBITS_1);
  	usart_set_mode (USART1, USART_MODE_TX_RX);
  	usart_set_parity (USART1, USART_PARITY_NONE);
  	usart_set_flow_control (USART1, USART_FLOWCONTROL_NONE);
  	// Enable USART1 Receive interrupt. 
  	//usart_enable_rx_interrupt (USART1);
  	// Finally enable the USART. 
  	usart_enable (USART1);

	/* Enable SPI1 clock. */
	rcc_periph_clock_enable(RCC_SPI1);

	/* reset SPI1 */
	spi_reset(SPI1);
	/* init SPI1 master */
	spi_init_master(SPI1,
					SPI_CR1_BAUDRATE_FPCLK_DIV_64,
					SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
					SPI_CR1_CPHA_CLK_TRANSITION_1,
					SPI_CR1_DFF_8BIT,
					SPI_CR1_MSBFIRST);
	/* enable SPI1 first */
	spi_enable(SPI1);
	}
void lis3dsh_init(void){
	int int_reg_value;

	/* init SPI1 */
	spi_setup();

	/* get WHO AM I value */
	int_reg_value = lis3dsh_read_reg(ADD_REG_WHO_AM_I);

	/* if WHO AM I value is the expected one */
	if (int_reg_value == UC_WHO_AM_I_DEFAULT_VALUE) {
		/* set output data rate to 400 Hz and enable X,Y,Z axis */
		lis3dsh_write_reg(ADD_REG_CTRL_4, UC_ADD_REG_CTRL_4_CFG_VALUE);
		/* verify written value */
		int_reg_value = lis3dsh_read_reg(ADD_REG_CTRL_4);
		/* if written value is different */
		if (int_reg_value != UC_ADD_REG_CTRL_4_CFG_VALUE) {
			/* ERROR: stay here... */
			while (1);
		}
	} else {
		/* ERROR: stay here... */
		while (1);
	}
	}
/* Function to write a register to LIS3DSH through SPI  */
void lis3dsh_write_reg(int reg, int data){
	/* set CS low */
	gpio_clear(GPIOE, GPIO3);
	/* discard returned value */
	spi_xfer(SPI1, SET_WRITE_SINGLE_CMD(reg));
	spi_xfer(SPI1, data);
	/* set CS high */
	gpio_set(GPIOE, GPIO3);
	}
/* Function to read a register from LIS3DSH through SPI */
int lis3dsh_read_reg(int reg){
	int reg_value;
	/* set CS low */
	gpio_clear(GPIOE, GPIO3);
	reg_value = spi_xfer(SPI1, SET_READ_SINGLE_CMD(reg));
	reg_value = spi_xfer(SPI1, 0xFF);
	/* set CS high */
	gpio_set(GPIOE, GPIO3);

	return reg_value;
	}
/* Transform a two's complement value to 16-bit int value */
int two_compl_to_int16(int two_compl_value){
	int int16_value = 0;

	/* conversion */
	if (two_compl_value > 32768) {
		int16_value = -(((~two_compl_value) & 0xFFFF) + 1);
	} else {
		int16_value = two_compl_value;
	}

	return int16_value;
	}

void send (uint16_t data) {
	//
	usart_send_blocking(USART1, data);
	}
void send_string (char *s_de_char){
  int j = 0;
  int d;
  do {
      d = s_de_char[j];
      if (d != 0)
        send (d);
      j++;
    }
  while (d != 0);
	}
char* custom_itoa(int i, char b[]){
    char const digit[] = "0123456789";
    char* p = b;
    if(i<0){
        *p++ = '-';
        i *= -1;
    }
    int shifter = i;
    do{ //Move to where representation ends
        ++p;
        shifter = shifter/10;
    }while(shifter);
    *p = '\0';
    do{ //Move back, inserting digits as u go
        *--p = digit[i%10];
        i = i/10;
    }while(i);
    return b;
	}
