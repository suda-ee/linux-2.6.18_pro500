# Makefile for xmon

ifdef CONFIG_PPC64
EXTRA_CFLAGS += -mno-minimal-toc
endif

obj-y			+= xmon.o setjmp.o start.o nonstdio.o

ifdef CONFIG_XMON_DISASSEMBLY
obj-y			+= ppc-dis.o ppc-opc.o
obj-$(CONFIG_PPC_CELL)	+= spu-dis.o spu-opc.o
endif
