#
#       !!!! Do NOT edit this makefile with an editor which replace tabs by spaces !!!!    
#
##############################################################################################
# 
# On command line:
#
# make all = Create project
#
# make clean = Clean project files.
#
# To rebuild project do "make clean" and "make all".
#

##############################################################################################
# Start of default section
#

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CP   = $(TRGT)objcopy
CLSS = $(TRGT)objdump
AS   = $(TRGT)as
#gcc -x assembler-with-cpp
BIN  = $(CP) -O ihex 
BINX = $(CP) -O binary 

# Object files directory
ifeq ($(PCB), X12D)
  OBJDIR = x12obj
else
ifeq ($(PCB), TARANIS)
 ifeq ($(REVPLUS), 1)
  OBJDIR = tpobj
 else
  OBJDIR = tobj
 endif
else
ifeq ($(PCB), X9D)
 ifeq ($(REVPLUS), 1)
  OBJDIR = xpobj
 else
  OBJDIR = xobj
 endif
else
ifeq ($(PCB), 9XT)
 OBJDIR = 9xtobj
else
ifeq ($(PCB), X7)
 ifeq ($(T12), 1)
  OBJDIR = t12obj
 else
  OBJDIR = x7obj
 endif
else
 ifeq ($(REVX), 1)
  OBJDIR = robj
 else
 ifeq ($(PCB), X9E)
  OBJDIR = x9eobj
 else
 ifeq ($(PCB), XLITE)
  OBJDIR = lobj
 else
  OBJDIR = obj
 endif
 endif
 endif
endif
endif
endif
endif
endif

MCU  = cortex-m3

# List all default C defines here, like -D_DEBUG=1
DDEFS =

# List all default ASM defines here, like -D_DEBUG=1
DADEFS = 

# List all default directories to look for include files here
DINCDIR = 

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS = 

MEMORIES = sram

EXT = STD

#
# End of default section
##############################################################################################

##############################################################################################
# Start of user section
#

# 
# Define project name and Ram/Flash mode here
PROJECT        = bootloader

ifeq ($(PCB), TARANIS)
RUN_FROM_FLASH = 0
 else
 ifeq ($(PCB), X9D)
  RUN_FROM_FLASH = 0
 else
 ifeq ($(PCB), 9XT)
  RUN_FROM_FLASH = 0
 else
 ifeq ($(PCB), X9E)
  RUN_FROM_FLASH = 0
 else
 ifeq ($(PCB), X7)
  RUN_FROM_FLASH = 0
 else
 ifeq ($(PCB), X12D)
  RUN_FROM_FLASH = 0
 else
 ifeq ($(PCB), XLITE)
   RUN_FROM_FLASH = 0
 else
   RUN_FROM_FLASH = 1
 endif
 endif
 endif
 endif
 endif
 endif
endif

EXTRAINCDIRS =

#
# Define linker script file here
#
ifeq ($(RUN_FROM_FLASH), 1)
 ifeq ($(REVX), 1)
  LDSCRIPT = sam3s8c_boot.ld
  FULL_PRJ = $(PROJECT)_flash8
  EXTRAINCDIRS += include
#  EXTRAINCDIRS += ../
 else
  LDSCRIPT = sam3s4c_boot.ld
  FULL_PRJ = $(PROJECT)_flash4
  EXTRAINCDIRS += include
#  EXTRAINCDIRS += ../
 endif
 TRGT = arm-none-eabi-
 CPPDEFS += -DPCBSKY 
 UDEFS = -Dat91sam3s8
else
 ifeq ($(PCB), X9D)
  ARCH = ARM
  LDSCRIPT = stm32_ramBoot.ld
  TRGT = arm-none-eabi-
  CPPDEFS += -DHSE_VALUE=12000000
  CPPDEFS += -DPCBX9D 
  ifeq ($(REVPLUS), 1)
   FULL_PRJ = $(PROJECT)_ramBootSp
  else
   FULL_PRJ = $(PROJECT)_ramBootS
  endif
  EXTRAINCDIRS += include
#  EXTRAINCDIRS += ../x9d
 else
 ifeq ($(PCB), TARANIS)
  ARCH = ARM
  LDSCRIPT = stm32_ramBoot.ld
  TRGT = arm-none-eabi-
  CPPDEFS += -DHSE_VALUE=12000000
  CPPDEFS += -DPCBX9D 
  CPPDEFS += -DPCBTARANIS 
  ifeq ($(REVPLUS), 1)
   FULL_PRJ = $(PROJECT)_ramBootTp
  else
   FULL_PRJ = $(PROJECT)_ramBootT
  endif
  EXTRAINCDIRS += include
#  EXTRAINCDIRS += ../targets/taranis/STM32F2xx_StdPeriph_Lib_V1.1.0/Libraries/CMSIS/Device/ST/STM32F2xx/Include
 else
 ifeq ($(PCB), 9XT)
  ARCH = ARM
  LDSCRIPT = stm32_ramBoot.ld
  TRGT = arm-none-eabi-
  CPPDEFS += -DHSE_VALUE=12000000
  CPPDEFS += -DPCB9XT 
  CPPDEFS += -DPCBSP 
  FULL_PRJ = $(PROJECT)_ramBoot9xt
  EXTRAINCDIRS += include
 else
  ifeq ($(PCB), X9E)
   ARCH = ARM
   LDSCRIPT = stm32_ramBoot.ld
   TRGT = arm-none-eabi-
   CPPDEFS += -DHSE_VALUE=12000000
   CPPDEFS += -DREV9E 
   CPPDEFS += -DREVPLUS
   CPPDEFS += -DPCBX9D
   FULL_PRJ = $(PROJECT)_ramBootx9e
   EXTRAINCDIRS += include
 else
  ifeq ($(PCB), X7)
   ARCH = ARM
   LDSCRIPT = stm32_ramBoot.ld
   TRGT = arm-none-eabi-
   CPPDEFS += -DHSE_VALUE=12000000
   CPPDEFS += -DPCBX7 
   CPPDEFS += -DPCBX9D
   ifeq ($(T12), 1)
	 FULL_PRJ = $(PROJECT)_ramBootT12
    CPPDEFS += -DPCBT12
	else
	 FULL_PRJ = $(PROJECT)_ramBootx7
	endif
   EXTRAINCDIRS += include
  else
  ifeq ($(PCB), XLITE)
   ARCH = ARM
   LDSCRIPT = stm32_ramBoot.ld
   TRGT = arm-none-eabi-
   CPPDEFS += -DHSE_VALUE=12000000
   CPPDEFS += -DPCBXLITE 
   CPPDEFS += -DPCBX9D
   FULL_PRJ = $(PROJECT)_ramBootL
   EXTRAINCDIRS += include
  else
   ifeq ($(PCB), X12D)
    ARCH = ARM
    LDSCRIPT = stm32f4_ramBoot.ld
    TRGT = arm-none-eabi-
    CPPDEFS += -DHSE_VALUE=12000000
    CPPDEFS += -DPCBX12D
    FULL_PRJ = $(PROJECT)_ramBootx12
    EXTRAINCDIRS += include
    UDEFS = -DSTM32F429_439xx
   endif
   endif
  endif
  endif
 endif
 endif
 endif
endif
ifeq ($(REVPLUS), 1)
  CPPDEFS += -DREVPLUS
endif

# List all user C define here, like -D_DEBUG=1

# Define ASM defines here
UADEFS = 

ifeq ($(PCB), TARANIS)
SRC  = system_stm32f2xx.c \
       stm32f2xx_rcc.c \
       stm32f2xx_gpio.c \
       stm32f2xx_spi.c \
       misc.c \
		 ccsbcs.c \
       usb/usb_core.c \
       usb/usb_dcd.c \
       usb/usb_dcd_int.c \
       usb/usbd_core.c \
       usb/usbd_ioreq.c \
       usb/usbd_req.c \
       usb/usbd_msc_data.c \
       usb/usbd_msc_scsi.c \
       usb/usbd_msc_bot.c \
       usb/usbd_msc_core.c \
       usb/usbd_desc.c \
       usb/usb_bsp.c

CPPSRC = aspi.cpp \
         lcd_driver.cpp \
         driversboot.cpp \
         logicioboot.cpp \
         ff.cpp \
			x9ddiskio.cpp \
         usbd_usr.cpp \
         usbd_storage_msd.cpp \
       	i2c_ee.cpp \
         lcdboot.cpp \
			power.cpp \
         boot.cpp

# List ASM source files here
ASRC = startup_stm32f2xx.s

else


ifeq ($(PCB), X9D)
SRC  = system_stm32f2xx.c \
       stm32f2xx_rcc.c \
       stm32f2xx_gpio.c \
       stm32f2xx_spi.c \
       misc.c \
       usb/usb_core.c \
       usb/usb_dcd.c \
       usb/usb_dcd_int.c \
       usb/usbd_core.c \
       usb/usbd_ioreq.c \
       usb/usbd_req.c \
       usb/usbd_msc_data.c \
       usb/usbd_msc_scsi.c \
       usb/usbd_msc_bot.c \
       usb/usbd_msc_core.c \
       usb/usbd_desc.c \
       usb/usb_bsp.c

CPPSRC = lcdboot.cpp \
         ff.cpp \
         aspi.cpp \
         x9ddiskio.cpp \
         lcd_driver.cpp \
       	i2c_ee.cpp \
         driversboot.cpp \
         logicioboot.cpp \
			ff_lfn.cpp \
         usbd_usr.cpp \
         usbd_storage_msd.cpp \
			power.cpp \
         boot.cpp


# List ASM source files here
ASRC = x9d/startup_stm32f2xx.s

else


ifeq ($(PCB), X7)
SRC  = system_stm32f2xx.c \
       stm32f2xx_rcc.c \
       stm32f2xx_gpio.c \
       stm32f2xx_spi.c \
       misc.c \
       usb/usb_core.c \
       usb/usb_dcd.c \
       usb/usb_dcd_int.c \
       usb/usbd_core.c \
       usb/usbd_ioreq.c \
       usb/usbd_req.c \
       usb/usbd_msc_data.c \
       usb/usbd_msc_scsi.c \
       usb/usbd_msc_bot.c \
       usb/usbd_msc_core.c \
       usb/usbd_desc.c \
       usb/usb_bsp.c

CPPSRC = lcdboot.cpp \
         ff.cpp \
         aspi.cpp \
         x9ddiskio.cpp \
         lcd_driver.cpp \
       	i2c_ee.cpp \
         driversboot.cpp \
         logicioboot.cpp \
			ff_lfn.cpp \
         usbd_usr.cpp \
         usbd_storage_msd.cpp \
			power.cpp \
         boot.cpp


# List ASM source files here
ASRC = x9d/startup_stm32f2xx.s

else

ifeq ($(PCB), XLITE)
SRC  = system_stm32f2xx.c \
       stm32f2xx_rcc.c \
       stm32f2xx_gpio.c \
       stm32f2xx_spi.c \
       misc.c \
       usb/usb_core.c \
       usb/usb_dcd.c \
       usb/usb_dcd_int.c \
       usb/usbd_core.c \
       usb/usbd_ioreq.c \
       usb/usbd_req.c \
       usb/usbd_msc_data.c \
       usb/usbd_msc_scsi.c \
       usb/usbd_msc_bot.c \
       usb/usbd_msc_core.c \
       usb/usbd_desc.c \
       usb/usb_bsp.c

CPPSRC = lcdboot.cpp \
         ff.cpp \
         aspi.cpp \
         x9ddiskio.cpp \
         lcd_driver.cpp \
       	i2c_ee.cpp \
         driversboot.cpp \
         logicioboot.cpp \
			ff_lfn.cpp \
         usbd_usr.cpp \
         usbd_storage_msd.cpp \
			power.cpp \
         boot.cpp


# List ASM source files here
ASRC = x9d/startup_stm32f2xx.s

else

ifeq ($(PCB), 9XT)
SRC  = system_stm32f2xx.c \
       stm32f2xx_rcc.c \
       stm32f2xx_gpio.c \
       stm32f2xx_spi.c \
       misc.c \
       usb/usb_core.c \
       usb/usb_dcd.c \
       usb/usb_dcd_int.c \
       usb/usbd_core.c \
       usb/usbd_ioreq.c \
       usb/usbd_req.c \
       usb/usbd_msc_data.c \
       usb/usbd_msc_scsi.c \
       usb/usbd_msc_bot.c \
       usb/usbd_msc_core.c \
       usb/usbd_desc.c \
       usb/usb_bsp.c

CPPSRC = lcdboot.cpp \
         ff.cpp \
         x9ddiskio.cpp \
         driversboot.cpp \
         logicioboot.cpp \
			ff_lfn.cpp \
         usbd_usr.cpp \
         usbd_storage_msd.cpp \
			power.cpp \
			mega64.cpp \
         boot.cpp

#         aspi.cpp \
#         lcd_driver.cpp \
#       	i2c_ee.cpp \

# List ASM source files here
ASRC = x9d/startup_stm32f2xx.s
else
ifeq ($(PCB), X9E)
SRC  = system_stm32f2xx.c \
       stm32f2xx_rcc.c \
       stm32f2xx_gpio.c \
       stm32f2xx_spi.c \
       misc.c \
       usb/usb_core.c \
       usb/usb_dcd.c \
       usb/usb_dcd_int.c \
       usb/usbd_core.c \
       usb/usbd_ioreq.c \
       usb/usbd_req.c \
       usb/usbd_msc_data.c \
       usb/usbd_msc_scsi.c \
       usb/usbd_msc_bot.c \
       usb/usbd_msc_core.c \
       usb/usbd_desc.c \
       usb/usb_bsp.c

CPPSRC = lcdboot.cpp \
         ff.cpp \
         aspi.cpp \
         x9ddiskio.cpp \
         lcd_driver.cpp \
       	i2c_ee.cpp \
         driversboot.cpp \
         logicioboot.cpp \
			ff_lfn.cpp \
         usbd_usr.cpp \
         usbd_storage_msd.cpp \
			power.cpp \
         boot.cpp


# List ASM source files here
ASRC = x9d/startup_stm32f2xx.s

else
ifeq ($(PCB), X12D)
SRC  = X12D/system_stm32f4xx.c \
		 X12D/stm32f4xx_ltdc.c \
		 X12D/stm32f4xx_dma2d.c \
       X12D/stm32f4xx_rcc.c \
		 X12D/sdram_driver.c \
		 X12D/stm32f4xx_fmc.c \
       X12D/stm32f4xx_gpio.c \
       X12D/pwr_driver.c \
       X12D/misc.c

#       X12D/pwr_driver.c

#SRC  = X12D/system_stm32f4xx.c \
#       X12D/stm32f4xx_spi.c \
#       X12D/misc.c \
#		 X12D/stm32f4xx_dma.c \
#		 X12D/stm32f4xx_ltdc.c \
#       X12D/stm32f4xx_sdio.c \
#		 X12D/sdio_sd.c \
#       X12D/usb_core.c \
#       X12D/usb_dcd.c \
#       X12D/usb_dcd_int.c \
#       X12D/usbd_core.c \
#       X12D/usbd_ioreq.c \
#       X12D/usbd_req.c \
#       X12D/usbd_msc_data.c \
#       X12D/usbd_msc_scsi.c \
#       X12D/usbd_msc_bot.c \
#       X12D/usbd_msc_core.c \
#       X12D/usbd_desc.c \
#       X12D/usb_bsp.c

CPPSRC = logicioboot.cpp \
         X12D/lcd_driver12.cpp \
			X12D/lcdboot12.cpp \
         driversboot.cpp \
         ff.cpp \
			ff_lfn.cpp \
         boot.cpp

#CPPSRC = X12D/lcdboot.cpp \
#         logicioboot.cpp
#         X12D/lcd_driver.cpp \
#         ff.cpp \
#         x12ddiskio.cpp \
#         lcd_driver.cpp \
#         X12D/hdrivers.cpp \
#			ff_lfn.cpp \
#         X12D/usbd_usr.cpp \
#         X12D/led_driver.cpp \
#         usbd_storage_msd.cpp \


# List ASM source files here
ASRC = X12D/startup_stm32f42_43xxx.s

else

# List C source files here
SRC  = core_cm3.c \
       board_lowlevel.c \
       crt.c \
       usb/device/core/USBD_UDP.c usb/device/core/USBDDriver.c usb/device/core/USBDCallbacks.c \
       usb/device/massstorage/MSDDriver.c usb/device/massstorage/MSDDStateMachine.c \
		 usb/device/massstorage/MSDLun.c usb/device/massstorage/MSDDriverDescriptors.c usb/device/massstorage/SBCMethods.c \
       usb/common/core/USBEndpointDescriptor.c usb/common/core/USBGenericRequest.c \
		 usb/common/core/USBFeatureRequest.c usb/common/core/USBInterfaceRequest.c usb/common/core/USBGetDescriptorRequest.c \
       usb/common/core/USBSetAddressRequest.c usb/common/core/USBSetConfigurationRequest.c \
       usb/common/core/USBConfigurationDescriptor.c usb/common/core/USBGenericDescriptor.c \
       MEDSdcard.c \
       vectors_sam3s.c

CPPSRC = lcdboot.cpp \
         ff.cpp \
			diskio.cpp \
         driversboot.cpp \
         logicioboot.cpp \
         sdcard_driver.cpp \
         massstorage.cpp \
         bootsamEeprom.cpp \
			ff_lfn.cpp \
			power.cpp \
         boot.cpp

# List ASM source files here
ASRC =

endif

endif
endif
endif
endif
endif
endif

# List all user directories here
UINCDIR = ./inc
# \
#          ./cmsis/core \
#          ./cmsis/device

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = 

# Define optimisation level here
OPT = -Os

#
# End of user defines
##############################################################################################

INCDIR  = $(patsubst %,-I%,$(DINCDIR) $(UINCDIR))
LIBDIR  = $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR))

ifeq ($(RUN_FROM_FLASH), 0)
DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=0
else
DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=1
endif

ADEFS   = $(DADEFS) $(UADEFS)

# Generate the (raw) object files lists
AXOBJS    = $(ASRC:%.s=%.o)
CXOBJS    = $(SRC:%.c=%.o)
CPPXOBJS  = $(CPPSRC:%.cpp=%.o)

# Remove directory info from the object files lists
AOBJS    = $(patsubst %,$(OBJDIR)/%,$(notdir $(AXOBJS) ))
COBJS    = $(patsubst %,$(OBJDIR)/%,$(notdir $(CXOBJS) ))
CPPOBJS  = $(patsubst %,$(OBJDIR)/%,$(notdir $(CPPXOBJS) ))
DIRS = $(sort $(dir $(AXOBJS) $(CXOBJS) $(CPPXOBJS) ) )

# Search path for sources
VPATH = $(DIRS)

# Generates the dependancy files lists
INCLUDES = $(AOBJS:.o=.o.d) $(COBJS:.o=.o.d) $(CPPOBJS:.o=.o.d)

LIBS    = $(DLIBS) $(ULIBS)
MCFLAGS = -mcpu=$(MCU)

#ASFLAGS = $(MCFLAGS) -g -gdwarf-2 -Wa,-amhls=$(OBJDIR)/$(notdir $(<:.s=.lst)) $(ADEFS)
CPFLAGS = $(MCFLAGS) $(OPT) -gdwarf-2 -mthumb -fomit-frame-pointer -Wall -Wstrict-prototypes -fverbose-asm $(DEFS)
LDFLAGS = $(MCFLAGS) -mthumb -nostartfiles -T$(LDSCRIPT) -Wl,-Map=$(FULL_PRJ).map,--cref,--no-warn-mismatch $(LIBDIR)
CPPFLAGS = $(MCFLAGS) $(OPT) -gdwarf-2 -mthumb -fomit-frame-pointer -Wall -fverbose-asm $(DEFS)

CPPFLAGS += $(CPPDEFS)
CPFLAGS += $(CPPDEFS)
CPFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))

# If REVX board
ifeq ($(REVX), 1)
 CPPDEFS += -DREVX
 EXT_MOD=REVB-X
endif

CPPDEFS += -DCPUARM

CPPDEFS += -DREVB

# If serial Debug required
ifeq ($(DEBUG), 1)
 CPPDEFS += -DDEBUG
endif

CPPDEFS += -DBOOT

# Generate dependency information
CPFLAGS += -MD -MP -MF $(OBJDIR)/$(@F).d
CPPFLAGS += -MD -MP -MF $(OBJDIR)/$(@F).d
CPPFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))

#
# makefile rules
#

all: size_before $(AOBJS) $(COBJS) $(CPPOBJS) $(FULL_PRJ).elf $(FULL_PRJ).hex $(FULL_PRJ).lss $(FULL_PRJ).bin size

#begin:
#	@echo "Includes"
#	@echo $(INCLUDES)
#	@echo
#	@echo $(OBJS) $(COBJS) $(CPPOBJS)
#	@echo
#	@echo $(DIRS)

size_before:
#	arm-none-eabi-size $(FULL_PRJ).elf
	@if test -f $(FULL_PRJ).elf; then arm-none-eabi-size $(FULL_PRJ).elf; fi

size:
#	arm-none-eabi-size $(FULL_PRJ).elf
	@if test -f $(FULL_PRJ).elf; then arm-none-eabi-size $(FULL_PRJ).elf; fi

$(OBJDIR):
	test -d $(OBJDIR) || $(shell mkdir -p $(OBJDIR) )

$(CPPOBJS) : $(OBJDIR)/%.o : %.cpp
	@mkdir -p $(@D)
	$(CC) -c $(CPPFLAGS) -fno-exceptions -I . $(INCDIR) $< -o $@

$(COBJS) : $(OBJDIR)/%.o : %.c
	@mkdir -p $(@D)
	$(CC) -c $(CPFLAGS) -I . $(INCDIR) $< -o $@


$(AOBJS) : $(OBJDIR)/%.o : %.s
	@mkdir -p $(@D)
	$(AS) -c $(ASFLAGS) $< -o $@

%elf: $(AOBJS) $(COBJS) $(CPPOBJS)
	$(CC) $(AOBJS) $(COBJS) $(CPPOBJS) $(LDFLAGS) $(LIBS) -o $@
  
# Create extended listing file from ELF output file.
%.lss: %.elf
	$(CLSS) -h -S $< > $@

%hex: %elf
	$(BIN) $< $@

%bin: %elf
	$(BINX) $< $@

#  $(PROJECT)_flash8
#  $(PROJECT)_flash4
#  $(PROJECT)_ramBootS
#  $(PROJECT)_ramBootT

clean:
	-rm -f obj/*.*
	-rm -f xobj/*.*
	-rm -f robj/*.*
	-rm -f tobj/*.*
	-rm -f tpobj/*.*
	-rm -f xpobj/*.*
	-rm -f 9xtobj/*.*
	-rm -f x9eobj/*.*
	-rm -f x7obj/*.*
	-rm -f *.elf
	-rm -f *.map
	-rm -f *.hex
	-rm -f *.bin
	-rm -f *.lss
#	-rm -fR .dep

# 
# Include the dependency files, should be the last of the makefile
#
#-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# Add in all the dependancy files
-include $(INCLUDES)

# *** EOF ***
