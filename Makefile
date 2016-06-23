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
OBJS = $(NAME).o serial_port_stm32.o syscalls.o interface.o lis3dsh.o mfunctions.o

# Linker flags 
LD_FLAGS= -Wl,--start-group $(CFLAGS) $(ARCH_FLAGS) $(SPECIFIC_FLAGS) -I$(MAVLIB) $(OBJS) \
-Wl,--end-group -Wl,--print-gc-sections -T$(LDSTM32F4) -nostartfiles 

all: $(NAME).bin

$(NAME).bin: $(NAME).elf
	$(OC) -Obinary $(NAME).elf $(NAME).bin

$(NAME).elf: $(SRC)/$(NAME).cpp
	$(CC) $(CFLAGS) $(ARCH_FLAGS) $(SPECIFIC_FLAGS) -c $(SRC)/syscalls.c 
	$(CC) -I$(INCLUDE) $(CFLAGS) $(ARCH_FLAGS) $(SPECIFIC_FLAGS) -c $(SRC)/lis3dsh.c 

	$(CXX) -I$(MAVLIB) -I$(INCLUDE) $(CFLAGS) $(ARCH_FLAGS) $(SPECIFIC_FLAGS) -c $(SRC)/$(NAME).cpp
	$(CXX) -I$(MAVLIB) -I$(INCLUDE) $(CFLAGS) $(ARCH_FLAGS) $(SPECIFIC_FLAGS) -c $(SRC)/interface.cpp
	$(CXX) -I$(MAVLIB) -I$(INCLUDE) $(CFLAGS) $(ARCH_FLAGS) $(SPECIFIC_FLAGS) -c $(SRC)/serial_port_stm32.cpp
	$(CXX) -I$(MAVLIB) -I$(INCLUDE) $(CFLAGS) $(ARCH_FLAGS) $(SPECIFIC_FLAGS) -c $(SRC)/mfunctions.cpp
	$(CXX) $(LD_FLAGS) $(LIBS) -o $(NAME).elf

#	$(CXX) -I$(MAVLIB) $(NAME).cpp serial_port.cpp autopilot_interface.cpp -o $(NAME) -lpthread

flash: $(NAME).bin
	#st-flash erase v2 0x8000000
	st-flash write $(NAME).bin 0x8000000

graph: $(NAME).bin
	mkdir Graphs
	mv *.expand Graphs/
	egypt Graphs/*.expand | dot -Tsvg -Grankdir=LR -o Graphs/$(NAME).svg
	eog Graphs/$(NAME).svg

clean:
	 rm -rf *.o $(NAME).elf *.d Graphs/ *.expand *.bin
