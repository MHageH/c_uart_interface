# C UART Interface
A heavy modification of the original c_uart_interface_example, works on ARM Cortex-M4 STM32F4

The original c_uart_interface_example can be found here (maintained by Lorenz Meier) :
https://github.com/mavlink/c_uart_interface_example

This modification is in Alpha stage currently, but works fine.

This version is stripped down to bare minimum (All these elements have been removed) : 
- pthreads (All thread dependent functions have been replaced)
- read/write (replaced by usart receive and send functions)
- serial port (replaced with USART interuption)
- All C++ classes 
- try and catch mechanisms
- parse commandline and Ctrl-C handeling functions
- printfs


Dependencies :
- mavlink
- libopencm3
