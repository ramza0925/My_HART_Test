/*
THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES INC. ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT, ARE
DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES INC. BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

YOU ASSUME ANY AND ALL RISK FROM THE USE OF THIS CODE OR SUPPORT FILE.

IT IS THE RESPONSIBILITY OF THE PERSON INTEGRATING THIS CODE INTO AN APPLICATION
TO ENSURE THAT THE RESULTING APPLICATION PERFORMS AS REQUIRED AND IS SAFE.

    Module       : retarget.c
    Description  :
    Date         : Dec 2012
    Version      : v1.01
    Changelog    : v1.00 (Feb 2010) Initial
                   v1.01 (Dec 2012) Modified for fputc and fgetc using UART
*/

#include <stdio.h>
#include <rt_misc.h>

#include "UART.h"

#pragma import(__use_no_semihosting_swi)

struct __FILE { int handle; };

FILE __stdout;  /* Use standard output */
FILE __stdin;   /* Use standard input  */

/*
 * writes the character specified by c (converted to an unsigned char) to
 * the output stream pointed to by stream, at the position indicated by the
 * asociated file position indicator (if defined), and advances the
 * indicator appropriately. If the file position indicator is not defined,
 * the character is appended to the output stream.
 * Returns: the character written. If a write error occurs, the error
 *          indicator is set and fputc returns EOF.
 */
int fputc(int ch, FILE * stream )
{
  if(ch == '\n')              /* any new-line (line feed) character */
    UART_putchar('\r');       /* precede by carrier return character */
  return(UART_putchar(ch));   /* pass character to UART output */
}

int __backspace(FILE *stream)
{
/*
 * Should be implemented if scanf function is used???
 */
  return 0x0;
}

/*
 * obtains the next character (if present) as an unsigned char converted to
 * an int, from the input stream pointed to by stream, and advances the
 * associated file position indicator (if defined).
 * Returns: the next character from the input stream pointed to by stream.
 *          If the stream is at end-of-file, the end-of-file indicator is
 *          set and fgetc returns EOF. If a read error occurs, the error
 *          indicator is set and fgetc returns EOF.
 */
int fgetc(FILE * stream)
{
  return (UART_getkey());   /* Use UART */
}


int ferror(FILE *f) 
{
  /* Your implementation of ferror */
  return EOF;
}


void _ttywrch(int ch)       
{ 
  fputc(ch,stdout); 
}


void _sys_exit(int return_code) {
label:  goto label;  /* endless loop */
}
