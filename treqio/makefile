PROJECT_NAME := treqio-app

OUTPUT_FILENAME = $(PROJECT_NAME)
export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

SDK_PATH = ../../sdk
TEMPLATE_PATH = $(SDK_PATH)/components/toolchain/gcc
TREQIO_COMMON_PATH = ../../common

ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc"
AS       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as"
AR       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar" -r
LD       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld"
NM       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm"
OBJDUMP  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump"
OBJCOPY  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy"
SIZE    		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size"

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(SDK_PATH)/components/drivers_nrf/clock/nrf_drv_clock.c \
$(SDK_PATH)/components/drivers_nrf/common/nrf_drv_common.c \
$(SDK_PATH)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
$(SDK_PATH)/components/drivers_nrf/hal/nrf_delay.c \
$(SDK_PATH)/components/drivers_nrf/rtc/nrf_drv_rtc.c \
$(SDK_PATH)/components/drivers_nrf/spi_master/spi_master.c \
$(SDK_PATH)/components/drivers_nrf/twi_master/nrf_drv_twi.c \
$(SDK_PATH)/components/drivers_nrf/uart/app_uart.c \
$(SDK_PATH)/components/drivers_nrf/wdt/nrf_drv_wdt.c \
$(SDK_PATH)/components/libraries/button/app_button.c \
$(SDK_PATH)/components/libraries/timer/app_timer.c \
$(SDK_PATH)/components/libraries/util/app_error.c \
$(SDK_PATH)/softdevice/common/softdevice_handler/softdevice_handler.c \
$(SDK_PATH)/components/toolchain/system_nrf51.c \
../src/ble/treqio_ble.c \
../src/bsp/bsp.c \
../src/gsm/treqio_gsm.c \
../src/gps/treqio_gps.c \
../src/memory/treqio_memory.c \
../src/scheduler/app_scheduler.c \
../src/segger_rtt/SEGGER_RTT.c \
../src/segger_rtt/SEGGER_RTT_printf.c \
../src/treqio/treqio_device.c \
../src/treqio-app.c \

# $(TREQIO_COMMON_PATH)/dlogic_treq_gsm.c \
# $(SDK_PATH)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
# $(TREQIO_COMMON_PATH)/dlogic_treq_debug.c \
# $(SDK_PATH)/components/libraries/fifo/app_fifo.c \
# C_SOURCE_FILES += \
# $(SDK_PATH)/components/libraries/util/app_error.c \
# $(SDK_PATH)/components/libraries/scheduler/app_scheduler.c \
# $(SDK_PATH)/components/libraries/timer/app_timer.c \
# $(SDK_PATH)/components/libraries/timer/app_timer_appsh.c \
# $(SDK_PATH)/components/libraries/bootloader_dfu/bootloader.c \
# $(SDK_PATH)/components/libraries/bootloader_dfu/bootloader_settings.c \
# $(SDK_PATH)/components/libraries/bootloader_dfu/bootloader_util.c \
# $(SDK_PATH)/components/libraries/crc16/crc16.c \
# $(SDK_PATH)/components/libraries/bootloader_dfu/dfu_init_template.c \
# $(SDK_PATH)/components/libraries/bootloader_dfu/dfu_single_bank.c \
# $(SDK_PATH)/components/libraries/bootloader_dfu/dfu_transport_serial.c \
# $(SDK_PATH)/components/libraries/hci/hci_mem_pool.c \
# $(SDK_PATH)/components/libraries/hci/hci_slip.c \
# $(SDK_PATH)/components/libraries/hci/hci_transport.c \
# $(SDK_PATH)/components/libraries/util/nrf_assert.c \
# $(SDK_PATH)/components/drivers_nrf/uart/app_uart.c \
# $(SDK_PATH)/components/drivers_nrf/hal/nrf_delay.c \
# $(SDK_PATH)/components/drivers_nrf/common/nrf_drv_common.c \
# $(SDK_PATH)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
# $(SDK_PATH)/components/drivers_nrf/pstorage/pstorage_raw.c \
# ../../../main.c \
# $(SDK_PATH)/components/ble/common/ble_advdata.c \
# $(SDK_PATH)/components/ble/common/ble_conn_params.c \
# $(SDK_PATH)/components/ble/common/ble_srv_common.c \
# $(SDK_PATH)/components/toolchain/system_nrf51.c \
# $(SDK_PATH)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
# $(SDK_PATH)/components/softdevice/common/softdevice_handler/softdevice_handler_appsh.c \

#assembly files common to all targets
ASM_SOURCE_FILES  = $(SDK_PATH)/components/toolchain/gcc/gcc_startup_nrf51.s
# ASM_SOURCE_FILES  = ../src/gcc_startup_nrf51.S

#includes common to all targets
INC_PATHS = -I../src/ble
INC_PATHS += -I../src/bsp
INC_PATHS += -I../src/gps
INC_PATHS += -I../src/gsm
INC_PATHS += -I../src/memory
INC_PATHS += -I../src/scheduler
INC_PATHS += -I../src/sdk
INC_PATHS += -I../src/sdk/config
INC_PATHS += -I../src/segger_rtt
INC_PATHS += -I../src/timer
INC_PATHS += -I../src/treqio
INC_PATHS += -I$(SDK_PATH)/components/ble
INC_PATHS += -I$(SDK_PATH)/components/ble/common
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/clock
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/rtc
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/wdt
INC_PATHS += -I$(SDK_PATH)/components/libraries/util
INC_PATHS += -I$(SDK_PATH)/components/libraries/fifo
INC_PATHS += -I$(SDK_PATH)/components/toolchain
INC_PATHS += -I$(SDK_PATH)/components/toolchain/gcc
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/hal
INC_PATHS += -I$(SDK_PATH)/components/softdevice/s130/headers
INC_PATHS += -I$(SDK_PATH)/components/device
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/uart
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/common
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/gpiote
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/spi_master
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/twi_master
INC_PATHS += -I$(SDK_PATH)/components/libraries/button
INC_PATHS += -I$(SDK_PATH)/components/libraries/timer


OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DSWI_DISABLE0
CFLAGS += -DBOARD_TREQIO
# CFLAGS += -DDEBUG_SEGGER
CFLAGS += -DSCHEDULER_WITH_PRIORITY
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DSPI_MASTER_1_ENABLE
CFLAGS += -DNRF51
CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -DS130
CFLAGS += -DBLE_STACK_SUPPORT_REQD
# CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -Os
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DBOARD_TREQIO
# ASMFLAGS += -DDEBUG_SEGGER
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DSCHEDULER_WITH_PRIORITY
ASMFLAGS += -DSPI_MASTER_1_ENABLE
ASMFLAGS += -DNRF51
ASMFLAGS += -D__HEAP_SIZE=0
ASMFLAGS += -DS130
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
# ASMFLAGS += -DBSP_DEFINES_ONLY
#default target - first one defined
default: clean nrf51822_xxac

debug:	CFLAGS += -DDEBUG_SEGGER

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51822_xxac

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51822_xxac


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf51822_xxac: LINKER_SCRIPT=../src/armgcc_s130_nrf51822_xxaa.ld
nrf51822_xxac: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ""
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ""

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash: clean nrf51822_xxac
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
	/usr/local/bin/nrf -f $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

## Flash softdevice