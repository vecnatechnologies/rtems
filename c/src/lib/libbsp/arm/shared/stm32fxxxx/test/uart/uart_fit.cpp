/**
 * @file uart_fit.c
 * @author Jay M. Doyle
 *
 * @ingroup test
 *
 * @brief CppUnit test code for testing the uart functionality in
 *   hal-uart*.c
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#include <CppUTest/TestHarness.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <hal-uart-interface.h>
#include <bspopts.h>

extern "C" {
#include <hal-utils.h>
}

#define MAX_NUMBER_OF_UARTS 8

struct uart_config {
  char name[ 16 ];
  int handle;
};

static uart_config configured_uarts[ MAX_NUMBER_OF_UARTS ] = { 0 };
static int         num_uarts = 0;

static bool isCppOutputUart( const char *uart_name )
{
  return ( strcmp( uart_name, CPPUTEST_OUTPUT_DEVICE ) == 0 );
}

TEST_GROUP( hal_uart_fit )
{
  void setup()
  {
    uint32_t i;

    for ( i = 0; i < COUNTOF( configured_uarts ); i++ ) {
      snprintf( (char *) configured_uarts[ num_uarts ].name,
        sizeof( configured_uarts[ num_uarts ].name ), "/dev/ttyS%lu", i );

      // Do not try to re-open the COM port used for the CppUnit output
      if ( isCppOutputUart( (char *) configured_uarts[ num_uarts ].name ) ==
           false ) {
        configured_uarts[ num_uarts ].handle =
          open( (char *) configured_uarts[ num_uarts ].name,
            O_RDWR | O_APPEND );

        // If everything worked correctly then increment the number of buses
        if ( configured_uarts[ num_uarts ].handle > 0 ) {
          num_uarts++;
        }
      }
    }

// This causes problems with the shell
#if 1
    // check for console
    snprintf( (char *) configured_uarts[ num_uarts ].name,
      sizeof( configured_uarts[ num_uarts ].name ), "/dev/console" );

    if ( access( configured_uarts[ num_uarts ].name, F_OK ) != -1 ) {
      configured_uarts[ num_uarts ].handle = 1;
    }

#endif
  }

  void teardown()
  {
    int i;

    // close all uarts
    for ( i = 0; i < num_uarts; i++ ) {
      close( configured_uarts[ num_uarts ].handle );
      configured_uarts[ num_uarts ].handle = 0;
      configured_uarts[ num_uarts ].name[ 0 ] = '\0';
      num_uarts--;
    }
  }
};

int find_uart_by_name( const char *uart_name )
{
  uint32_t i;

  for ( i = 0; i < COUNTOF( configured_uarts ); i++ ) {
    if ( strcmp( configured_uarts[ i ].name, uart_name ) == 0 ) {
      return configured_uarts[ i ].handle;
    }
  }

  return 0;
}

/**
 * @brief Validates that uart1 is visible in file system
 *    if it has been configured in the configure.ac file
 */
TEST( hal_uart_fit, check_ttyS0 )
{
#if STM32_ENABLE_USART_1

  if ( isCppOutputUart( "/dev/ttyS0" ) == false ) {
    if ( uart_used_for_console( STM32F_UART1 ) == false ) {
      CHECK_TEXT( find_uart_by_name(
          "/dev/ttyS0" ) > 0, "Failed to open /dev/ttyS0" );
    } else {
      CHECK_TEXT( find_uart_by_name(
          "/dev/console" ) > 0, "Failed to open /dev/console" );
    }
  }
#endif
}

TEST( hal_uart_fit, check_ttyS1 )
{
#if STM32_ENABLE_USART_2

  if ( isCppOutputUart( "/dev/ttyS1" ) == false ) {
    if ( uart_used_for_console( STM32F_UART2 ) == false ) {
      CHECK_TEXT( find_uart_by_name(
          "/dev/ttyS1" ) > 0, "Failed to open /dev/ttyS1" );
    } else {
      CHECK_TEXT( find_uart_by_name(
          "/dev/console" ) > 0, "Failed to open /dev/console" );
    }
  }
#endif
}

TEST( hal_uart_fit, check_ttyS2 )
{
#if STM32_ENABLE_USART_3

  if ( isCppOutputUart( "/dev/ttyS2" ) == false ) {
    if ( uart_used_for_console( STM32F_UART3 ) == false ) {
      CHECK_TEXT( find_uart_by_name(
          "/dev/ttyS2" ) > 0, "Failed to open /dev/ttyS2" );
    } else {
      CHECK_TEXT( find_uart_by_name(
          "/dev/console" ) > 0, "Failed to open /dev/console" );
    }
  }
#endif
}

TEST( hal_uart_fit, check_ttyS3 )
{
#if STM32_ENABLE_USART_4

  if ( isCppOutputUart( "/dev/ttyS3" ) == false ) {
    if ( uart_used_for_console( STM32F_UART4 ) == false ) {
      CHECK_TEXT( find_uart_by_name(
          "/dev/ttyS3" ) > 0, "Failed to open /dev/ttyS3" );
    } else {
      CHECK_TEXT( find_uart_by_name(
          "/dev/console" ) > 0, "Failed to open /dev/console" );
    }
  }
#endif
}

TEST( hal_uart_fit, check_ttyS4 )
{
#if STM32_ENABLE_USART_5

  if ( isCppOutputUart( "/dev/ttyS4" ) == false ) {
    if ( uart_used_for_console( STM32F_UART5 ) == false ) {
      CHECK_TEXT( find_uart_by_name(
          "/dev/ttyS4" ) > 0, "Failed to open /dev/ttyS4" );
    } else {
      CHECK_TEXT( find_uart_by_name(
          "/dev/console" ) > 0, "Failed to open /dev/console" );
    }
  }
#endif
}

TEST( hal_uart_fit, check_ttyS5 )
{
#if STM32_ENABLE_USART_6

  if ( uart_used_for_console( STM32F_UART6 ) == false ) {
    CHECK_TEXT( find_uart_by_name(
        "/dev/ttyS5" ) > 0, "Failed to open /dev/ttyS5" );
  } else {
    CHECK_TEXT( find_uart_by_name(
        "/dev/console" ) > 0, "Failed to open /dev/console" );
  }
#endif
}

TEST( hal_uart_fit, check_ttyS6 )
{
#if STM32_ENABLE_USART_7

  if ( isCppOutputUart( "/dev/ttyS6" ) == false ) {
    if ( uart_used_for_console( STM32F_UART7 ) == false ) {
      CHECK_TEXT( find_uart_by_name(
          "/dev/ttyS6" ) > 0, "Failed to open /dev/ttyS6" );
    } else {
      CHECK_TEXT( find_uart_by_name(
          "/dev/console" ) > 0, "Failed to open /dev/console" );
    }
  }
#endif
}

TEST( hal_uart_fit, check_ttyS7 )
{
#if STM32_ENABLE_USART_8

  if ( isCppOutputUart( "/dev/ttyS7" ) == false ) {
    if ( uart_used_for_console( STM32F_UART8 ) == false ) {
      CHECK_TEXT( find_uart_by_name(
          "/dev/ttyS7" ) > 0, "Failed to open /dev/ttyS7" );
    } else {
      CHECK_TEXT( find_uart_by_name(
          "/dev/console" ) > 0, "Failed to open /dev/console" );
    }
  }
#endif
}

