#
# Makefile for the MV patches
#
obj-y := $(patsubst %.c,%.o,$(shell (cd $(srctree)/mvl_patches;ls *.c) 2>/dev/null))
p1-objs := $(patsubst %.c,%.o,$(shell (cd $(srctree)/mvl_patches;ls *-0[0-4]*.c) 2>/dev/null)) dummy.o
p2-objs := $(patsubst %.c,%.o,$(shell (cd $(srctree)/mvl_patches;ls *-0[5-9]*.c) 2>/dev/null)) dummy.o
p3-objs := $(patsubst %.c,%.o,$(shell (cd $(srctree)/mvl_patches;ls *-1[0-4]*.c) 2>/dev/null)) dummy.o
p4-objs := $(patsubst %.c,%.o,$(shell (cd $(srctree)/mvl_patches;ls *-1[5-9]*.c) 2>/dev/null)) dummy.o

obj-y := p1.o p2.o p3.o p4.o
