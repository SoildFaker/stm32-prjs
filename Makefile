# Makefile for the STM32F103C8
#
# Enbin Li
#

PROJECT = copter

# Project Structure
SRCDIR = project
BINDIR = project/bin
OBJDIR = project/obj
DRVDIR = drivers
LIBDIR = lib/src

# Project target
CPU = cortex-m3

# Sources
SRC = $(wildcard $(SRCDIR)/*.c) $(wildcard $(DRVDIR)/*.c) $(wildcard $(LIBDIR)/*.c)
ASM = $(wildcard $(SRCDIR)/*.s) $(wildcard $(DRVDIR)/*.s)

# Include directories
INCLUDE  = -Icmsis -Ilib/inc -Idrivers 

# Linker 
LSCRIPT = STM32F103XB.ld

# C Flags
GCFLAGS  = -Wall -fno-common -mcpu=$(CPU) -mthumb --specs=nosys.specs -g -Wa,-ahlms=$(addprefix $(OBJDIR)/,$(notdir $(<:.c=.lst)))
GCFLAGS += -O1
GCFLAGS += $(INCLUDE)
LDFLAGS += -T$(LSCRIPT) -mcpu=$(CPU) -mthumb --specs=nosys.specs
LDFLAGS += -fno-math-errno -lm -u _printf_float -u _scanf_float
ASFLAGS += -mcpu=$(CPU)

# Flashing
OCDFLAGS = -f /usr/share/openocd/scripts/interface/stlink-v2.cfg \
		   -f /usr/share/openocd/scripts/target/stm32f1x.cfg \
		   -f openocd.cfg

# Tools
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
AR = arm-none-eabi-ar
LD = arm-none-eabi-ld
GDB = arm-none-eabi-gdb
CGDB = cgdb
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size
OBJDUMP = arm-none-eabi-objdump
OCD = openocd

RM = rm -rf

## Build process

OBJ := $(addprefix $(OBJDIR)/,$(notdir $(SRC:.c=.o)))
OBJ += $(addprefix $(OBJDIR)/,$(notdir $(ASM:.s=.o)))


all:: $(BINDIR)/$(PROJECT).bin

Build: $(BINDIR)/$(PROJECT).bin

install: $(BINDIR)/$(PROJECT).bin
	$(OCD) $(OCDFLAGS)

debug: $(BINDIR)/$(PROJECT).elf
	$(GDB) $(BINDIR)/$(PROJECT).elf

$(BINDIR)/$(PROJECT).hex: $(BINDIR)/$(PROJECT).elf
	$(OBJCOPY) -R .stack -O ihex $(BINDIR)/$(PROJECT).elf $(BINDIR)/$(PROJECT).hex

$(BINDIR)/$(PROJECT).bin: $(BINDIR)/$(PROJECT).elf
	$(OBJCOPY) -R .stack -O binary $(BINDIR)/$(PROJECT).elf $(BINDIR)/$(PROJECT).bin

$(BINDIR)/$(PROJECT).elf: $(OBJ)
	@mkdir -p $(dir $@)
	$(CC) $(OBJ) $(LDFLAGS) -o $(BINDIR)/$(PROJECT).elf
	$(OBJDUMP) -D $(BINDIR)/$(PROJECT).elf > $(BINDIR)/$(PROJECT).lst
	$(SIZE) $(BINDIR)/$(PROJECT).elf

macros:
	$(CC) $(GCFLAGS) -dM -E - < /dev/null

cleanBuild: clean

clean:
	$(RM) $(BINDIR)
	$(RM) $(OBJDIR)

# Compilation
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(GCFLAGS) -c $< -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -o $@ $<


$(OBJDIR)/%.o: $(LIBDIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(GCFLAGS) -c $< -o $@

$(OBJDIR)/%.o: $(LIBDIR)/%.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -o $@ $<

$(OBJDIR)/%.o: $(DRVDIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(GCFLAGS) -c $< -o $@

$(OBJDIR)/%.o: $(DRVDIR)/%.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -o $@ $<
