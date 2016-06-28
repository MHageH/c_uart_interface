#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#include <cstdlib>
#include <stdio.h>   // Standard input/output definitions
#include <unistd.h>  // UNIX standard function definitions

#include <common/mavlink.h>

#ifdef STM32F4

#include "libopencm3/cm3/common.h"	
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/memorymap.h"
#include "libopencm3/stm32/usart.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/stm32/spi.h"
#include <libopencm3/cm3/nvic.h>

#ifdef __cplusplus
extern "C" {
#endif 

#include <lis3dsh.h>

#ifdef __cplusplus
}
#endif

// Time related structures
struct timeval {
		time_t      tv_sec;     /* seconds */
		suseconds_t tv_usec;    /* microseconds */
		};
struct timezone {
		int tz_minuteswest;     /* minutes west of Greenwich */
		int tz_dsttime;         /* type of DST correction */
		};

#else 

#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>                  /* declaration of bzero() */
#include <fcntl.h>
#include <termios.h>

#include <sys/time.h>
#include <time.h>

#define USART3 3

int init_rs232();
void free_rs232();
void sendcmd(int,char*);

#define BAUDRATE B57600

// Moded for socat bridge
// Modify it back to /dev/ttyUSB0 or the required port
#define RS232_DEVICE "/dev/ttyUSB0" 
//#define RS232_DEVICE "/dev/pts/7"
// 


int usart_recv_blocking(int i);

#endif 

// Initialisation
void serial_start(void);

// Read
int serial_read_message(mavlink_message_t &message);

// Write
int serial_write_message(const mavlink_message_t &message);

// Time related
int get_time_sec(struct timeval *tv, struct timezone *tz);


#endif 
