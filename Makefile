ifndef $(MSPGCCDIR)
	MSPGCCDIR = /opt/ti/msp430-gcc
endif

# Paths
INCLUDES_DIRECTORY = $(MSPGCCDIR)/include
SRC_DIR = ./src
# Device
DEVICE = msp430fr5994

# Compiler options
CC = $(MSPGCCDIR)/bin/msp430-elf-gcc

CFLAGS = -I . -I $(INCLUDES_DIRECTORY) -mmcu=$(DEVICE) -g -mhwmult=f5series
LDFLAGS = -L . -L $(INCLUDES_DIRECTORY)

# MSPDEBUG driver used for installation
DRIVER := tilib

# Compile
%.elf: $(SRC_DIR)/%.c
	@echo "Compiling $< to $@..."
	@$(CC) $(CFLAGS) $(LDFLAGS) $< -o $@

# Upload to board
run.%: %.elf
	@mspdebug $(DRIVER) "prog $<" --allow-fw-update

# Clean output files
clean:
	@echo "Removing all output files..."
	@rm -f *.o *.elf
