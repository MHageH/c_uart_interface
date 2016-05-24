# C UART Interface
A heavy modification of the original c_uart_interface_example, works on ARM 
Cortex-M4 STM32F4

The original c_uart_interface_example can be found here (maintained by Lorenz 
Meier): https://github.com/mavlink/c_uart_interface_example

This modification is in Beta stage currently.

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
- Support for PC (works, needs developement)
- added a separate makefile
- STM32F4 build can simulate movement using the internal acceleration sensor
LIS3DSH
- Separate source files for each architecture
- Resolved a bug when using the latest mavlink(c_library)

## TODO
- asynchronous read()/write() mechanism using USART interrupts
- Reduce timer prescaler
- Make timer_isr dependent functions more general (works with any value of the 
prescaler)
- Recombine general_read_messages () and read_messages()
- Add support for other boards
- Employ correction algorithms based on external sensor input
 
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
apt-get install libmpc-dev libgmp-dev libmpfr-dev

- git clone https://github.com/jmfriedt/summon-arm-toolchain
- chmod +x summon-arm-toolchain 
- ./summon-arm-toolchain

Define the path to the binaries after the installation is finished :
export PATH=/home/<USERNAME>/sat/bin:$PATH

You can put this file in .bashrc or .profile to make bash add the path 
each time.

# Setup st-link
Once the toolchain is installed to ~/sat or any other directory you choose, 
proceed to install texane st-link:
https://github.com/texane/stlink 

Instruction on the installation are available in the README.md file.

# PX4 Compilation dependencies
Keep in mind that this piece of code (although not complete) interacts mainly 
with PX4 autopilot system.

Everything discussed here can be found on http://dev.px4.io/:

apt-get install python-serial openocd flex bison libncurses5-dev autoconf
texinfo build-essential libftdi-dev libtool zlib1g-dev python-empy

# Simulation dependencies :
apt-get install ant protobuf-compiler libeigen3-dev libopencv-dev openjdk-7-jdk
openjdk-7-jre clang-3.5 lldb-3.5

# Getting PX4 Firmware 
- mkdir -p ~/src
- cd ~/src
- git clone https://github.com/PX4/Firmware.git
- cd Firmware
- git submodule update --init --recursive

# Setting up simulation and netcat bridges
Warning : you need at openjdk-7-jdk or openjdk-6-jdk, otherwise, it jmavsim 
won't work unless tweaked.

The simulation is based on jMAVsim, which is being regularly updated. This 
application is able to simulate the movement of a drone to a high extent.

First of all: check if the ttyUSB* device is present :
- watch -n 1 “ dmesg | tail – 20” 

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
- nc -l -u -p 14540 127.0.0.1 | cat > /dev/ttyUSB0 
In the second (2):
- cat < /dev/ttyUSB0 | nc -u 127.0.0.1 14556

Do not execute these yet, as you will need to compile and lunch the 
simualation first :
Open a third terminal and go to the PX4 source code
- cd ~/src/Firmware 
- make -j(#number_of_computer_cores) posix_sitl_default jmavsim

If compiled succefully, a NuttxShell must spawn:
- commander arm
- commander disarm

Repeat untill you recieve the current home position, then :
- param show SYS_COMPANION
- param set SYS_COMPANION 57600 

to correctly set the baud rate.

The simulated drone is ready now to recieve commands.

Make sure you are able to compile and flash the STM32F4-discovery (follow 
testing the interface)
Then execute the commands in the terminals in the following orders : (2) 
then (1)

Usually the simulated drone will react and behave exactly as a real drone.

# Testing on a real drone :
WARNING : You need to configure the drone correctly with qgroundcontrol before 
doing this, or it will NOT react.

STILL BEING WORKED.

Here, you don't need any netcat connection, just plug a serial port connection 
from Telem 2 (on the pixhawk) to the GPIO Pins on the STM32F4-discovery (after 
flashing it)

Use a FTDI converter to connect to Serial 4 (on the pixhawk), and run this in 
a terminal:
- screen /dev/ttyUSB0 57600 8N1
- You can use a radio connection with this one.

and hit enter, you'll see a NuttxShell prompt waiting.
Tap in: 
- commander arm

You'll see the motors start spining.
Connect the STM32F4-discovery to power now.

IF correctly configured with a GPS, it will start to fly, if not, you'll have 
to enter the offboard control mode yourself :
- commander mode offboard 

# Testing the interface
Make sure to initialise the mavlink submodule

- git clone https://github.com/MHageH/c_uart_interface.git
- cd c_uart_interface
- git submodule init
- git submodule update 

Connect the STM32F4-discovery now, then:

- make
- make flash

It should work by now

# Generate Graphs 
To better understand the inner working of such application, as well as a
fast way to start programming it, function call graphs can be automatically
generated :

- make graph

Requirements :
- Graphviz
- egypt

