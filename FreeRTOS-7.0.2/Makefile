##
## FILE:        Makefile
##
## DESCRIPTION: Makefile for FreeRTOS.
##
## Copyright (c) 2011 Maksym Galemin
##

LIB = libfreertos.a

ifeq ($(HOST_OS), WINDOWS)
PWDDIR  = $(shell cygpath -m $(shell pwd))
else
PWDDIR  = $(shell pwd)
endif
CFLAGS += -I"$(PWDDIR)/include"

SRCS   += croutine.c
SRCS   += list.c
SRCS   += queue.c
SRCS   += tasks.c
SRCS   += timers.c

ifeq ($(TARGET_CPU_SUBFAMILY), CORTEX_M3)
ifeq ($(LPC1769_MPU), YES)
SRCS   += portable/GCC/ARM_CM3_MPU/port.c
CFLAGS += -I"$(PWDDIR)/portable/GCC/ARM_CM3_MPU"
else
SRCS   += portable/GCC/ARM_CM3/port.c
CFLAGS += -I"$(PWDDIR)/portable/GCC/ARM_CM3"
endif
endif

ifeq ($(MEM_MANG), MEM_SIMPLE)
SRCS   += portable/MemMang/heap_1.c
endif
ifeq ($(MEM_MANG), MEM_OPTIMAL)
SRCS   += portable/MemMang/heap_2.c
endif
ifeq ($(MEM_MANG), MEM_STDLIB)
SRCS   += portable/MemMang/heap_3.c
endif

OBJS    = $(patsubst %.c,%.o,$(SRCS))

all: $(LIB)

.PHONY: all clean

$(LIB): $(OBJS)
	$(AR) $(ARFLAGS) $(LIB) $(OBJS)
	$(RANLIB) $(LIB)

*.o: *.c
	$(CC) $(CFLAGS) c $< -o $@

clean:
	$(RM) *~ $(OBJS) $(LIB)

depend: $(SRCS)
	makedepend $^
