/**
 * @file hal-fatal-error-handler.c
 *
 * @ingroup error
 *
 * @brief Error handler for CPU exceptions
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#include <rtems.h>
#include <rtems/score/userextimpl.h>
#include <rtems/score/cpu.h>

#include <hal-fatal-error-handler.h>


typedef struct
{
	uint64_t reg_value;
}register_double;

void stm32fxxxx_fatal_error_handler(Internal_errors_Source source, bool is_internal, CPU_Exception_frame *frame)
{
	register_double *fpu_reg;

	int *ptr = (int *)frame;
	int count=0;

	printf ("\n --- Hard Fault Occurred!!! :( --- \n");
	printf ("\n RTEMS Error Code : %d \n", source);
	printf (" Caused by RTEMS?: ");
	printf (is_internal == false ? "Nope\n" : "Yup\n");
	printf (" Hard fault caused by vector # %x\n", frame->vector);
	printf ("\n Providing Stacked Register Details... \n");
	printf ("\n General Purpose Registers:\n");

	for(count=0;count<=12;count++)
	{
		printf ("\tr%d  = 0x%x\n",count, *(ptr+count));
	}
	printf ("\tsp	= 0x%x -> Stack Pointer\n", frame->register_sp);
	printf ("\tlr	= 0x%x -> Subroutine Call Return Address\n", frame->register_lr);
	printf ("\tpc	= 0x%x -> Program Counter\n", frame->register_pc);
	printf ("\tpsr	= 0x%x -> Program Status Register\n", frame->register_xpsr);
	printf ("\tfpexc	= 0x%x -> Floating-Point Exception Register\n", frame->register_xpsr);
	printf ("\tfpscr	= 0x%x -> Floating-Point Status and Control Register\n", frame->register_xpsr);

	ptr = &(frame->vfp_context->register_d0);

	printf ("\n Single-Precision FPU Registers:\n");

	for(count=0;count<16;count++)
	{
		fpu_reg = (ptr+(count*2));
		printf ("\ts%d	= %-20e\n",count, (uint64_t)fpu_reg->reg_value);
	}

	__asm__ volatile ("BKPT #01"); /* Break debugger here */
	while(1);
}

rtems_name       Extension_name;
rtems_id         Extension_id;

rtems_extensions_table Extensions = {
 NULL,                      /* task create user extension */
 NULL,                      /* task start user extension */
 NULL,                      /* task restart user extension */
 NULL,                      /* task delete user extension */
 NULL,               		/* task switch user extension */
 NULL,                      /* task begin user extension */
 NULL,                      /* task exitted user extension */
 stm32fxxxx_fatal_error_handler               /* fatal error user extension */
};

/**
 * Include #define CONFIGURE_MAXIMUM_USER_EXTENSIONS  8
 *		   #define CONFIGURE_RTEMS_INIT_TASKS_TABLE
 *	in application code
 *
 */
rtems_status_code stm32_initialize_extensions(void){

	rtems_status_code sc;
	Extension_name = rtems_build_name('E', 'X', 'U', '1');

	sc = rtems_extension_create(
			Extension_name,
			&Extensions,
			&Extension_id
	);
	return sc;
}
