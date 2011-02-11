#
# Makefile for the linux kernel.
#
#

# Common objects
obj-y 			:= time.o irq.o clock.o serial.o io.o id.o \
			   gpio.o mux.o mux_cfg.o dma.o devices.o usb.o

ifeq ($(CONFIG_MACH_DAVINCI_EVM),y)
ifeq ($(CONFIG_MACH_DM357_EVM),y)
obj-y			+= board-dm357-evm.o i2c-emac.o
else
obj-y			+= board-evm.o i2c-emac.o
endif
endif


# Board specific
obj-$(CONFIG_MACH_DAVINCI_DM355_EVM)  += board-dm355-evm.o
obj-$(CONFIG_DAVINCI_I2C_EXPANDER)      += i2c-client.o
obj-$(CONFIG_MACH_DAVINCI_DM6467_EVM)  += board-dm6467-evm.o i2c-emac.o \
                                       video_hdevm.o
obj-$(CONFIG_DAVINCI_SPI)       	+= davinci_spi_platform.o

# Vlynq specific
obj-$(CONFIG_DAVINCI_VLYNQ)	+= davinci_vlynq_init.o

