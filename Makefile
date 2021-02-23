MODULE_NAME = imx_pwm_audio
OUT = $(MODULE_NAME).ko

# Tools directory is available in https://github.com/Glowforge/kernel-module-glowforge/
TOOLS_DIR = ../tools
SDMA_ASSEMBLER = PERL5LIB=$(TOOLS_DIR) $(TOOLS_DIR)/sdma_asm.pl

KERNEL_SRC ?= /usr/src/kernel

ASM = imx_pwm_audio_sdma.asm

ccflags-y += -Wno-unknown-pragmas

obj-m += $(MODULE_NAME).o

SDMA_SCRIPT = $(ASM)
SDMA_SCRIPT_ASSEMBLED = $(ASM:.asm=.asm.h)

.PHONY: all clean modules_install

all: $(SDMA_SCRIPT_ASSEMBLED)
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

%.asm.h: %.asm
	$(SDMA_ASSEMBLER) $< > $@

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
	rm -f $(SDMA_SCRIPT_ASSEMBLED)
