/**
 * @file shell_bspcmds.c
 *
 * @ingroup shell
 *
 * @brief Boot command implementation. Application and processor specific implementaion.
 *           This command triggers the built-in bootloader of the processor, for enabling
 *           remote firmware update.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#ifdef HAVE_CONFIG_H
//#include "config.h"
#endif
#if 1
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <rtems.h>
#include <rtems/shell.h>


/* Extern processor specific function */
extern void BootloaderInit(void);

 int rtems_shell_main_boot(
  int   argc __attribute__((unused)),
  char *argv[] __attribute__((unused))
)
{

    printf("\nEntering bootloader mode.\n");

    /* Call the processor specific bootloader init function */
    /* Function Should not return */
    BootloaderInit();

    /* If function returned, something went wrong while triggering bootloader */
    printf("Triggering bootloader failed!!\n");

  return -1;
}

rtems_shell_cmd_t rtems_shell_BOOT_Command = {
  "boot",                                     /* name */
  "boot the processor",                       /* usage */
  "user",                                     /* topic */
  rtems_shell_main_boot,                      /* command */
  NULL,                                       /* alias */
  NULL                                        /* next */
};

#define SHELL_BSP_COMMANDS \
&rtems_shell_BOOT_Command


#define CONFIGURE_SHELL_USER_COMMANDS \
        SHELL_BSP_COMMANDS

#define CONFIGURE_SHELL_COMMANDS_INIT
#define CONFIGURE_SHELL_COMMANDS_ALL
#define CONFIGURE_SHELL_COMMANDS_ALL_NETWORKING
#define CONFIGURE_SHELL_MOUNT_MSDOS

#include <rtems/shellconfig.h>

#endif
