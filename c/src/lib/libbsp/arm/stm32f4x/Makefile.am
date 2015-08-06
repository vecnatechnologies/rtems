ACLOCAL_AMFLAGS = -I ../../../../aclocal

include $(top_srcdir)/../../../../automake/compile.am

include_bspdir = $(includedir)/bsp

###############################################################################
#                  Data                                                       #
###############################################################################

dist_project_lib_DATA = bsp_specs

noinst_LIBRARIES = libbspstart.a

libbspstart_a_SOURCES = ../shared/start/start.S

project_lib_DATA =
project_lib_DATA += start.$(OBJEXT)
project_lib_DATA += startup/linkcmds

EXTRA_DIST = startup/linkcmds.stm32f4x
EXTRA_DIST += startup/linkcmds.stm32f105rc

###############################################################################
#                  Header                                                     #
###############################################################################

include_HEADERS =
include_HEADERS += ../../shared/include/tm27.h
include_HEADERS += include/bsp.h

nodist_include_HEADERS =
nodist_include_HEADERS += ../../shared/include/coverhd.h
nodist_include_HEADERS += include/bspopts.h

nodist_include_bsp_HEADERS =
nodist_include_bsp_HEADERS += ../../shared/include/bootcard.h

include_bsp_HEADERS =
include_bsp_HEADERS += ../../shared/include/utility.h
include_bsp_HEADERS += ../../shared/include/irq-generic.h
include_bsp_HEADERS += ../../shared/include/irq-info.h
include_bsp_HEADERS += ../../shared/include/stackalloc.h
include_bsp_HEADERS += ../shared/include/start.h
include_bsp_HEADERS += ../shared/armv7m/include/armv7m-irq.h
include_bsp_HEADERS += include/irq.h
include_bsp_HEADERS += include/stm32f-processor-specific.h

#HAL Device headers
include_bsp_HEADERS += hal/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h
include_bsp_HEADERS += hal/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h
include_bsp_HEADERS += hal/CMSIS/Device/ST/STM32F4xx/Include/system_stm32f4xx.h

# CMSIS header
include_bsp_HEADERS += hal/CMSIS/Include/core_cm4.h
include_bsp_HEADERS += hal/CMSIS/Include/core_cmFunc.h
include_bsp_HEADERS += hal/CMSIS/Include/core_cmInstr.h
include_bsp_HEADERS += hal/CMSIS/Include/core_cmSimd.h

#HAL header files
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_can.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cec.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_conf.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_crc.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cryp_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cryp.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dcmi_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dcmi.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma2d.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_eth.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ramfunc.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_fmpi2c_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_fmpi2c.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_hash_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_hash.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_hcd.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2s_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2s.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_irda.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_iwdg.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_ltdc.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_nand.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_nor.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pccard.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pcd_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pcd.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_qspi.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rng.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rtc.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_sai_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_sai.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_sd.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_sdram.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_smartcard.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_spdifrx.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_spi.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_sram.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim_ex.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_usart.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_wwdg.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_fmc.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_fsmc.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_sdmmc.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_usb.h
include_bsp_HEADERS += hal/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h

# Discovery headers
include_bsp_HEADERS += hal/BSP/STM32F4-Discovery/stm32f4_discovery.h

# Shared driver headers
include_bsp_HEADERS += ../shared/stm32fxxxx/include/hal-can.h
include_bsp_HEADERS += ../shared/stm32fxxxx/include/hal-error.h
include_bsp_HEADERS += ../shared/stm32fxxxx/include/hal-sdram-interface.h
include_bsp_HEADERS += ../shared/stm32fxxxx/include/hal-startup-interface.h
include_bsp_HEADERS += ../shared/stm32fxxxx/include/hal-uart-interface.h
include_bsp_HEADERS += ../shared/stm32fxxxx/include/hal-utils.h

###############################################################################
#                  LibBSP                                                     #
###############################################################################

noinst_LIBRARIES += libbsp.a

libbsp_a_SOURCES =
libbsp_a_CPPFLAGS =
libbsp_a_LIBADD =

# Shared
libbsp_a_SOURCES += ../../shared/bootcard.c
libbsp_a_SOURCES += ../../shared/bspclean.c
libbsp_a_SOURCES += ../../shared/bspgetworkarea.c
libbsp_a_SOURCES += ../../shared/bsplibc.c
libbsp_a_SOURCES += ../../shared/bsppost.c
libbsp_a_SOURCES += ../../shared/bsppretaskinghook.c
libbsp_a_SOURCES += ../../shared/cpucounterread.c
libbsp_a_SOURCES += ../../shared/cpucounterdiff.c
libbsp_a_SOURCES += ../../shared/gnatinstallhandler.c
libbsp_a_SOURCES += ../../shared/sbrk.c
libbsp_a_SOURCES += ../../shared/src/stackalloc.c

# Startup
libbsp_a_SOURCES += ../shared/startup/bsp-start-memcpy.S
libbsp_a_SOURCES += ../shared/stm32fxxxx/startup/hal-startup.c
libbsp_a_SOURCES += ../shared/stm32fxxxx/startup/hal-bsp-starthook.c
libbsp_a_SOURCES += ../shared/stm32fxxxx/startup/hal-bsp-reset.c

# SDRAM
libbsp_a_SOURCES += ../shared/stm32fxxxx/sdram/hal-sdram.c

# CAN
libbsp_a_SOURCES += ../shared/stm32fxxxx/can/hal-can.c

# Util
libbsp_a_SOURCES += ../shared/stm32fxxxx/util/hal-utils.c

# Error
libbsp_a_SOURCES += ../shared/stm32fxxxx/error/hal-error.c

# IRQ
libbsp_a_SOURCES += ../../shared/src/irq-default-handler.c
libbsp_a_SOURCES += ../../shared/src/irq-generic.c
libbsp_a_SOURCES += ../../shared/src/irq-info.c
libbsp_a_SOURCES += ../../shared/src/irq-legacy.c
libbsp_a_SOURCES += ../../shared/src/irq-server.c
libbsp_a_SOURCES += ../../shared/src/irq-shell.c
libbsp_a_SOURCES += ../shared/armv7m/irq/armv7m-irq.c
libbsp_a_SOURCES += ../shared/armv7m/irq/armv7m-irq-dispatch.c

# Console
libbsp_a_SOURCES += ../../shared/console-termios.c
libbsp_a_SOURCES += ../shared/stm32fxxxx/uart/hal-uart.c
libbsp_a_SOURCES += ../shared/stm32fxxxx/uart/hal-uart-interface.c
libbsp_a_SOURCES += ../shared/stm32fxxxx/uart/hal-uart-termios.c

# RTC
libbsp_a_SOURCES += ../shared/stm32fxxxx/rtc/hal-rtc.c
libbsp_a_SOURCES += ../../shared/tod.c

# Clock
libbsp_a_SOURCES += ../../shared/clockdrv_shell.h
libbsp_a_SOURCES += ../shared/armv7m/clock/armv7m-clock-config.c

# Timer
libbsp_a_SOURCES += ../../shared/timerstub.c

# Cache
libbsp_a_SOURCES += ../../../libcpu/shared/src/cache_manager.c
libbsp_a_SOURCES += ../../../libcpu/arm/shared/cache/cache_.h


libbsp_a_CPPFLAGS += -I$(srcdir)/../../../libcpu/arm/shared/include
libbsp_a_CPPFLAGS += -I$(srcdir)/../../../libcpu/arm/shared/stm32fxxxx/include
libbsp_a_CPPFLAGS += -I$(srcdir)/hal/STM32F4xx_HAL_Driver/Inc
libbsp_a_CPPFLAGS += -I$(srcdir)/hal/CMSIS/Device/ST/STM32F4xx/Include
libbsp_a_CPPFLAGS += -I$(srcdir)/hal/CMSIS/Include
libbsp_a_CPPFLAGS += -I$(srcdir)/include
libbsp_a_CPPFLAGS += -I$(srcdir)/../shared/stm32fxxxx/include
libbsp_a_CPPFLAGS += -I$(srcdir)/uart

libbsp_a_CPPFLAGS += -D STM32F407xx -D TARGET_STM_PROCESSOR_PREFIX=4xx -DTARGET_STM_PROCESSOR=407

libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hash_ex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sram.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_irda.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spdifrx.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hash.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_smartcard.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fsmc.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cryp_ex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dcmi.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac_ex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pccard.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s_ex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fmc.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dcmi_ex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_fmpi2c.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rng.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc_ex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_fmpi2c_ex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_ltdc.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sai_ex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hcd.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma2d.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_nor.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cryp.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cec.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_nand.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sai.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_eth.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_qspi.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sdram.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_msp_template.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_wwdg.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sd.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
libbsp_a_SOURCES += hal/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_iwdg.c

# Discovery board bsp file
libbsp_a_SOURCES += hal/BSP/STM32F4-Discovery/stm32f4_discovery.c

###############################################################################
#                  Special Rules                                              #
###############################################################################

DISTCLEANFILES = include/bspopts.h

include $(srcdir)/preinstall.am
include $(top_srcdir)/../../../../automake/local.am
