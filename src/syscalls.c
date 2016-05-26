/***********************************************************************/
/*                                                                     */
/*  SYSCALLS.C:  System Calls                                          */
/*  most of this is from newlib-lpc and a Keil-demo                    */
/*                                                                     */
/*  These are "reentrant functions" as needed by                       */
/*  the WinARM-newlib-config, see newlib-manual.                       */
/*  Collected and modified by Martin Thomas                            */
/*                                                                     */
/***********************************************************************/

/* adapted for the SAM7 "serial.h" mthomas 10/2005 */

#include <stdlib.h>
#include <reent.h>
#include <sys/stat.h>

#include <libopencm3/stm32/usart.h>

// OLD : 	for (i = 0; i < len; i++) {c=usart_recv_blocking(USART1);*p++ = c;}

_ssize_t _read(struct _reent *r, int file, void *ptr, size_t len){
	char c;
	int  i;
	unsigned char *p;
	
	p = (unsigned char*)ptr;
	
	//for (i = 0; i < len; i++) {c=usart_recv_blocking(USART3);*p++ = c;} // Since we are using USART3
	return len - i;
}

// OLD : for (i = 0; i < len; i++) {usart_send_blocking(USART1, *p++);}

 _ssize_t _write ( struct _reent *r, int file, const void *ptr, size_t len){
	int i;
	const unsigned char *p;
	p = (const unsigned char*) ptr;
	
	for (i = 0; i < len; i++) {
                usart_send_blocking(USART3, *p++);
	}
	return len;
}

int _close_r(struct _reent *r,int file){
	// _close_r(struct _reent *, int);
	return 0;
}

_off_t _lseek_r(struct _reent *r, int file, _off_t ptr, int dir){
	/*  Always indicate we are at file beginning.  */
	return (_off_t)0;	
}

int _fstat_r(struct _reent *r, int file, struct stat *st){
	/*  Always set as character device.				*/
	st->st_mode = S_IFCHR;
	/* assigned to strong type with implicit 	*/
	/* signed/unsigned conversion.  Required by 	*/
	/* newlib.					*/

	return 0;
}


// inutile : jmfriedt
int _isatty(int file); // avoid warning 

int _isatty(int file) { return 1; }
int _kill(int file) { return 1; }
void _exit(int file) { while (1) {}}
int _getpid(int file) { return 1; }


#if 0
static void _exit (int n) {
label:  goto label; /* endless loop */
}
#endif 


/* "malloc clue function" from newlib-lpc/Keil-Demo/"generic" */

/**** Locally used variables. ****/
// mt: "cleaner": extern char* end;
extern char end[];              /*  end is set in the linker command 	*/
				/* file and is the end of statically 	*/
				/* allocated data (thus start of heap).	*/

static char *heap_ptr;		/* Points to current end of the heap.	*/

/************************** _sbrk_r *************************************
 * Support function. Adjusts end of heap to provide more memory to
 * memory allocator. Simple and dumb with no sanity checks.

 *  struct _reent *r -- re-entrancy structure, used by newlib to
 *                      support multiple threads of operation.
 *  ptrdiff_t nbytes -- number of bytes to add.
 *                      Returns pointer to start of new heap area.
 *
 *  Note:  This implementation is not thread safe (despite taking a
 *         _reent structure as a parameter).
 *         Since _s_r is not used in the current implementation, 
 *         the following messages must be suppressed.
 */
void * _sbrk_r(struct _reent *_s_r,ptrdiff_t nbytes){
	char  *base;		/*  errno should be set to  ENOMEM on error  */

	if (!heap_ptr) {	/*  Initialize if first time through.  */
		heap_ptr = end;
	}
	base = heap_ptr;	/*  Point to end of heap.  */
	heap_ptr += nbytes;	/*  Increase heap.  */
	
	return base;		/*  Return pointer to start of new heap area.*/
}

int usleep(int usec) {}
int sleep(int usec) {}
int __dso_handle() {}
int _open() {}


// CAUSE PROBLEMS when handeling MAVLink messages if it's not defined

// int _gettimeofday (){}