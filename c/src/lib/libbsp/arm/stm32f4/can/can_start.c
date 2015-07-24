#include <stdio.h>
#include <rtems/error.h>
#include <rtems/bspIo.h>
#include <rtems/libio.h>
#include <rtems/score/sysstate.h>
#include <dev/can/can.h>

static void
safe_printf (const char *fmt, ...)
{
va_list ap;

	va_start(ap, fmt);
	if ( _System_state_Is_up( _System_state_Get() ) )
		vfprintf( stderr, fmt, ap );
	else
		vprintk( fmt, ap );
	va_end(ap);
}


int bsp_register_can(void) {

  stm32_init_can();
}
