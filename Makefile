##########################################################################	
#	
# Porting to STM32F4-discovery done by : 
# 			 Mohamed Hage Hassan, <mohamed.hagehassan@yahoo.com>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
##########################################################################

NAME=mavlink_control

# Toolchain 
TC = arm-none-eabi-
CC = $(TC)gcc
CXX = $(TC)g++
OC = $(TC)objcopy

# Includes
MAVLIB=mavlink/include/mavlink/v1.0
INCLUDE = inc/ 
LIBS = -lopencm3_stm32f4 -lc -lgcc -lnosys

# Sources 
SRC = src/

# Compiler flags
CFLAGS = -Wno-write-strings -fno-common
CFLAGS += -fdump-rtl-expand

# Architecture specific flags
ARCH_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
SPECIFIC_FLAGS = -MD -DSTM32F4

LDSTM32F4 = linker/stm32f4-discovery.ld 

# Linker object
OBJS = $(NAME).o serial_port_stm32.o syscalls.o

# Linker flags 
LD_FLAGS= -Wl,--start-group $(CFLAGS) $(ARCH_FLAGS) $(SPECIFIC_FLAGS) -I$(MAVLIB) $(OBJS) \
-Wl,--end-group -Wl,--print-gc-sections -T$(LDSTM32F4) -nostartfiles 

all: $(NAME).bin

$(NAME).bin: $(NAME).elf
	$(OC) -Obinary $(NAME).elf $(NAME).bin

$(NAME).elf: git_submodule $(SRC)/$(NAME).cpp
	$(CC) $(CFLAGS) $(ARCH_FLAGS) $(SPECIFIC_FLAGS) -c $(SRC)/syscalls.c 
	$(CXX) -I$(MAVLIB) -I$(INCLUDE) $(CFLAGS) $(ARCH_FLAGS) $(SPECIFIC_FLAGS) -c $(SRC)/$(NAME).cpp
	$(CXX) -I$(MAVLIB) -I$(INCLUDE) $(CFLAGS) $(ARCH_FLAGS) $(SPECIFIC_FLAGS) -c $(SRC)/serial_port_stm32.cpp
	$(CXX) $(LD_FLAGS) $(LIBS) -o $(NAME).elf

#	$(CXX) -I$(MAVLIB) $(NAME).cpp serial_port.cpp autopilot_interface.cpp -o $(NAME) -lpthread

flash: $(NAME).bin
	#st-flash erase v2 0x8000000
	st-flash write $(NAME).bin 0x8000000

git_submodule:
	git submodule update --init --recursive

graph: $(NAME).bin
	mkdir Graphs
	mv *.expand Graphs/
	egypt Graphs/*.expand | dot -Tsvg -Grankdir=LR -o Graphs/$(NAME).svg

clean:
	 rm -rf *.o $(NAME).elf *.d Graphs/ *.expand
