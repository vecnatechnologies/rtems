
EXE_DIR = bin
UNIT_TEST_EXT = arm-bsp-unit-test
#RTEMS_ROOT = ../../../../../../../..
BSP_ROOT       = /opt/rtems-4.11/arm-rtems4.11/stm32f7x

RTEMS_INCLUDE  = $(BSP_ROOT)/lib/include $(BSP_ROOT)/lib/include/rtems
BSP_INCLUDE    = $(BSP_ROOT)/lib/include/bsp
PROJECT_DIR    = ../../stm32f7x

COMMON_FLAGS = -D__ARMEL__ -D__SOFTFP__ -D__rtems__ 
BSP_FLAGS = -D STM32F746xx -D TARGET_STM_PROCESSOR_PREFIX=7xx -DTARGET_STM_PROCESSOR=746

all: $(EXE_DIR)/$(UNIT_TEST_EXT)

CC       ?= gcc
CFLAGS   ?= -Wall -g -fpic $(COMMON_FLAGS) $(BSP_FLAGS)

CXX      ?= g++
CXXFLAGS ?= -Wall -g -fpic $(COMMON_FLAGS) $(BSP_FLAGS)

LDFLAGS  ?= -Wall -g

COBJS = 
CPPOBJS   = uart/uart_test.o

OBJS      = $(COBJS) $(CPPOBJS)

LIBS_UNIT = -lCppUTest -lCppUTestExt
LIBS      = $(LIBS_UNIT)
INCLUDES ?= -I../include -I../uart \
            -I$(RTEMS_INCLUDE)     \
            -I$(BSP_INCLUDE)       \
            -I$(PROJECT_DIR)/uart


$(EXE_DIR)/$(UNIT_TEST_EXT): $(OBJS)
	$(CXX) $(UNIT_TEST_EXT).cpp $(LDFLAGS) $^  $(LIBS) -o $@

# Objects
$(COBJS): %.o: %.c
	$(CC) $(CFLAGS) -c $(INCLUDES) $< -o $@

$(CPPOBJS): %.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $(INCLUDES) $< -o $@

clean:
	rm -f $(OBJS) 

.PHONY: clean libs