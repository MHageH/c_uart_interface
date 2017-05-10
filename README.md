# C UART Interface

[![Join the chat at https://gitter.im/MHageH/c_uart_interface](https://badges.gitter.im/MHageH/c_uart_interface.svg)](https://gitter.im/MHageH/c_uart_interface?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)
A heavy modification of the original c_uart_interface_example, works on ARM 
Cortex-M4 STM32F4

## Documentation provided
[In Depth](https://www.gitbook.com/book/mhageh/c-uart-interface-technical-details) explanation of the functionnality of this interface  
This readme serves as a [Usage guide](https://github.com/MHageH/c_uart_interface/blob/master/README.md)  
The FEMTO-ST internship [report](https://github.com/MHageH/Papers/blob/master/Rapport%20Stage%20L3%20Mohamed%20Hage%20Hassan.pdf) (French)  

The original c_uart_interface_example can be found here (maintained by Lorenz 
Meier): [mavlink_control](https://github.com/mavlink/c_uart_interface_example)

## Stable version at Beta Branch 

This version is stripped down to bare minimum (all these elements have been 
removed): 
- pthreads (all thread dependent functions have been replaced)
- read/write (replaced by usart receive and send functions)
- serial port (replaced with USART)
- All C++ classes 
- try and catch mechanisms
- parse commandline and Ctrl-C handeling functions
- printfs (Only for STM32F4 version)

Dependencies :
- mavlink
- libopencm3

## Recent mods 
- Support for PC
- added a separate makefile
- STM32F4 build can simulate movement using the internal acceleration sensor
LIS3DSH (joystick like behavior) 
- Separate source files for each architecture
Fixed mavlink library
- asynchronous read() mechanism using USART interrupts
- Recombine general_read_messages () and read_messages()
- Added initial position acquisation
- Tested on the ground with both STM32F4 and PC architectures
- modified the sequences, work as expected
- Added automatic arm/disarm functions 
- Move all the execution sequences to seperate files
- Concatenate all the repetitive commands into functions 
- Added full source code guide


## TODO
- Reduce timer prescaler
- Make timer_isr dependent functions more general (works with any value of the 
prescaler)
- Add disable_offboard_control () on exit failure
- Employ correction algorithms based on external sensor input (work on 
Sensor_bridge)
- Add support for other boards
- Add activation on request for STM32F4 board

# Requirements
- STM32F4-discovery (or others might be possible with code modification)
- ARM development tools : toolchain, st-link
- mavlink and libopencm3 libraries
- couple of FTDI converters (USB--Serial)
- Pixhawk (altho this can be optional if you want to simulate the drone itself)

# Setup a toolchain for ARM embedded development
For this to compile correctly, you need summon-arm-toolchain. 
A modified summon-arm-toolchain script is available here :
https://github.com/jmfriedt/summon-arm-toolchain

This version have latest gcc and libopencm3 configured for installation.

Install the dependencies :
```
apt-get install libmpc-dev libgmp-dev libmpfr-dev
```

```
 git clone https://github.com/jmfriedt/summon-arm-toolchain
 chmod +x summon-arm-toolchain 
 ./summon-arm-toolchain
```

Define the path to the binaries after the installation is finished :
```
export PATH=/home/<USERNAME>/sat/bin:$PATH
```

You can put this file in .bashrc or .profile to make bash add the path 
each time.

# Setup st-link
Once the toolchain is installed to ~/sat or any other directory you choose, 
proceed to install texane [ST-Link](https://github.com/texane/stlink)

Instruction on the installation are available in the README.md file.

# PX4 Compilation dependencies
Keep in mind that this piece of code (although not complete) interacts mainly 
with PX4 autopilot system.

Everything discussed here can be found on [Dev Guide](http://dev.px4.io/):

```
apt-get install python-serial openocd flex bison libncurses5-dev autoconf
texinfo build-essential libftdi-dev libtool zlib1g-dev python-empy
```

# Simulation dependencies :
```
apt-get install ant protobuf-compiler libeigen3-dev libopencv-dev openjdk-7-jdk
openjdk-7-jre clang-3.5 lldb-3.5
```

# Getting PX4 Firmware 
```
  mkdir -p ~/src
  cd ~/src
  git clone https://github.com/PX4/Firmware.git
  cd Firmware
  git submodule update --init --recursive
```

# Setting up simulation and netcat bridges
Warning : you need at openjdk-7-jdk or openjdk-6-jdk, otherwise, jmavsim 
won't work unless tweaked.

The simulation is based on jMAVsim, which is being regularly updated. This 
application is able to simulate the movement of a drone to a high extent.

First of all: check if the ttyUSB* device is present :
```
 watch -n 1 “ dmesg | tail – 20” 
```

Then setup a physical connection from the converter to the serial port :

(default :/dev/ttyUSB0):                                           
  

               | --------------------------------------- | TX : GPIO8 (PIN PD8)
               | --------------------------------------- | RX : GPIO9 (PIN PD9)
               | --------------------------------------- | GND
                        
The only problem with this one, is that it opens UDP ports on the local 
computer (port 14540 usually). Although our program is waiting on a 
serial-to-USB connection (via a FTDI converter).
Thus, we need to setup a mechanism to redirect all the traffic from and to 
that Serial port, to the UDP port run by the local simulation (OR reprogram 
the jMAVsim launching script to make it pass through a serial connection)

Establish a netcat bridge (bit unreliable):
Open 2 terminals, in the first (1) type: 
```
  nc -l -u -p 14540 127.0.0.1 | cat > /dev/ttyUSB0 
```
In the second (2):
```
  cat < /dev/ttyUSB0 | nc -u 127.0.0.1 14556
```

Do not execute these yet, as you will need to compile and lunch the 
simualation first :
Open a third terminal and go to the PX4 source code
```
  cd ~/src/Firmware 
  make -j(#number_of_computer_cores) posix_sitl_default jmavsim
```

If compiled succefully, a NuttxShell must spawn:
```
  commander arm
  commander disarm
```

Repeat untill you recieve the current home position, then :
```
  param show SYS_COMPANION
  param set SYS_COMPANION 57600 
```
to correctly set the baud rate.

The simulated drone is ready now to recieve commands.

Make sure you are able to compile and flash the STM32F4-discovery (follow 
testing the interface)
Then execute the commands in the terminals in the following orders : (2) 
then (1)

Usually the simulated drone will react and behave exactly as a real drone.

# Testing the PC version on a virtual drone
## Better connection without FTDI Cables
This will test the the PC version of the program without any cables, on the
virtual drone.

Open 3 terminals :
Type : 

In the (1)st terminal :

```
  nc -l -u -p 14540 127.0.0.1 | tee /dev/pts/(n+1) > /dev/null 
```
In the (2)nd terminal :
```
  socat -d -d pty,raw,echo=0 pty,raw,echo=0 
```
In the (3)rd terminal :
```
  cat < /dev/pts/(n+1) | nc -u 127.0.0.1 14556 
```

Execute these in the following order : (2) (3) (1)
Usualy, when you execute the (2) command, socat will open 2 virtual ports :
ex : 
```
  2016/05/27 17:44:53 socat[24892] N PTY is /dev/pts/12
  2016/05/27 17:44:53 socat[24892] N PTY is /dev/pts/13
  2016/05/27 17:44:53 socat[24892] N starting data transfer loop with FDs[3,3] 
and [5,5]
```
Here, n = 12, n+1 = 13

Things to change before launching the main program :
```
- in inc/serial_port.h , change #define RS232_DEVICE to "/dev/pts/n"
```

Then compile and execute :
```
  make -f makefile.pc
  ./mavlink_control
```

# Testing on a real drone :
WARNING : You need to configure the drone correctly with qgroundcontrol before 
doing this, or it will NOT react.

### [Testing footage (with STM32F4 internal accelerometer)](http://sendvid.com/ziebmszp)

Here, you don't need any netcat connection, just plug a serial port connection 
from Telem 2 (on the pixhawk) to the GPIO Pins D8, D9 and GND  on the STM32F4-discovery
(after flashing it) (using a telemetry radio is being worked on, but it's feasable)

Connect the STM32F4-discovery to power then select the sequence to be executed 
in commands () functions in mavlink_control.cpp

Flash the program :
```
st-flash erase v2 0x8000000
make -j4 flash
```
Remove the power cable.

Connect the telemetry link to the STM32F4 to D8, D9 and GND ports, and reconnect the 
power cable.


# Testing the interface

```
  git clone https://github.com/MHageH/c_uart_interface.git
  cd c_uart_interface
  make -j4 makefile.pc 
  ./mavlink_control
```
# Testing the Beta version
```
  git checkout Beta
  git pull origin Beta
```
Connect the STM32F4-discovery now, then:
```
  make
  make flash
```
It should work by now


# Generate Graphs 
To better understand the inner working of such application, as well as a
fast way to start programming it, function call graphs can be automatically
generated :

```
 make graph
```
Requirements :
```
 Graphviz
 egypt
```
