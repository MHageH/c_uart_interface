#include "serial_port.h"

mavlink_status_t status;
uint8_t          msgReceived = false;

volatile uint32_t ticks_sec = 0;
extern volatile float seconds;

// Initialisation
void serial_start(void){
    //rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
  	 rcc_periph_clock_enable (RCC_GPIOD);
  	 //rcc_periph_clock_enable (RCC_USART1);
  	 rcc_periph_clock_enable (RCC_USART3);

  	 // LEDs
  	 gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);

  	 //initialisation de port série
  	 nvic_enable_irq (NVIC_USART3_IRQ);	//uart interrupt
  	 // nvic_set_priority(NVIC_USART3_IRQ, 2);
  	 gpio_mode_setup (GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);	//GPD8 : Tx send from STM32 to ext
  	 gpio_mode_setup (GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);	//GPD9 : Rx recieve from ext to STM32
  	 gpio_set_output_options (GPIOD, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO9);
  	 gpio_set_af (GPIOD, GPIO_AF7, GPIO8);
  	 gpio_set_af (GPIOD, GPIO_AF7, GPIO9);
  	 // Setup USART3 parameters. 
  	 usart_set_baudrate (USART3, 57600);
  	 usart_set_databits (USART3, 8);
  	 usart_set_stopbits (USART3, USART_STOPBITS_1);
  	 usart_set_mode (USART3, USART_MODE_TX_RX);
  	 usart_set_parity (USART3, USART_PARITY_NONE);
  	 usart_set_flow_control (USART3, USART_FLOWCONTROL_NONE);
  	 // Enable USART3 Receive interrupt. 
  	 usart_disable_rx_interrupt (USART3);
  	 // Finally enable the USART. 
  	 usart_enable (USART3);

  	 // MOD :: Clock Initialisation
  	 // Enable Timer 2 clock
	 rcc_periph_clock_enable (RCC_TIM2);
	 // Enable Timer 2 interrupt
	 nvic_enable_irq(NVIC_TIM2_IRQ);
	 // nvic_set_priority(NVIC_TIM2_IRQ, 1);
	 // Reset Timer 2 peripheral
	 timer_reset(TIM2);
	 // Timer global mode : No divider, Alignement edge,  Direction up
	 timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	 //prescaler
	 timer_set_prescaler(TIM2, 64); // 255 for 1 Hz LED ON OR OFF : 127 for 1 on ON/OFF
	 //Enable preload
	 timer_disable_preload(TIM2);
	 //Continous mode
	 timer_continuous_mode(TIM2);
	 // Period 
	 timer_set_period(TIM2, 65535); // ( (16 MHz) ?? / (65535 * 255) ~ 1 Hz for LED_ON : 0.5 for LED_ON/OFF :: use prescaler at 127 for F_LED_ON/OFF = 1 Hz )
	 // ARR preload enable
	 timer_disable_preload(TIM2);
	 // Counter enable
	 timer_enable_counter(TIM2);
	 // Enable communication interrupt
	 timer_enable_irq(TIM2, TIM_DIER_CC1IE);

		return;
	}

// Interrupt service routines
void usart1_isr (void){
	//
	}
void usart3_isr(void){
	//
	}
void tim2_isr (void) { 
	ticks_sec++;
	seconds = seconds + 0.25;

		if (timer_get_flag(TIM2, TIM_SR_CC1IF)){
			timer_clear_flag(TIM2, TIM_SR_CC1IF);
		}
	}

// Serial Read
int serial_read_message(mavlink_message_t &message){

   	msgReceived = mavlink_parse_char(MAVLINK_COMM_1, usart_recv_blocking(USART3), &message, &status);

	return msgReceived;
	}

// Serial write
int serial_write_message(const mavlink_message_t &message){
		volatile char buff[300];
		
		int bytesWritten;
		
		unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buff, &message);


		for (int i = 0; i < sizeof(buff); i++) {
                usart_send_blocking(USART3, buff[i]);
                gpio_toggle(GPIOD, GPIO15);
		}
		
		len = 0;

		bytesWritten = len;

		return bytesWritten;
		}

int get_time_sec(struct timeval *tv, struct timezone *tz){

		uint32_t tick = ticks_sec;      /* ms */ // MOD :: each 250 ms
		
		tv->tv_sec = tick / 4; // MOD :: OLD : 1000
		tv->tv_usec = (tick % 250000) * 1000;
		
		return 0;
		}




