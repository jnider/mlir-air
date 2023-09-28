/******************************************************************************
* Copyright (c) 1995 - 2022 Xilinx, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
*******************************************************************************/
/*---------------------------------------------------*/
/* Modified from :                                   */
/* Public Domain version of printf                   */
/* Rud Merriam, Compsult, Inc. Houston, Tx.          */
/* For Embedded Systems Programming, 1991            */
/*                                                   */
/*---------------------------------------------------*/
#include <ctype.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "uart.h"

#define outbyte(c) uart_SendByte(c)

static void padding( const int32_t l_flag,const struct params_s *par);
static void outs(const char *lp, struct params_s *par);
static int32_t getnum(char **linep);

typedef struct params_s {
    int32_t len;
    int32_t num1;
    int32_t num2;
    int8_t pad_character;
    int32_t do_padding;
    int32_t left_flag;
    int32_t unsigned_flag;
} params_t;


/*---------------------------------------------------*/
/* The purpose of this routine is to output data the */
/* same as the standard printf function without the  */
/* overhead most run-time libraries involve. Usually */
/* the printf brings in many kilobytes of code and   */
/* that is unacceptable in most embedded systems.    */
/*---------------------------------------------------*/


/*---------------------------------------------------*/
/*                                                   */
/* This routine puts pad characters into the output  */
/* buffer.                                           */
/*                                                   */
static void padding( const int32_t l_flag, const struct params_s *par)
{
    int32_t i;

    if ((par->do_padding != 0) && (l_flag != 0) && (par->len < par->num1)) {
		i=(par->len);
        for (; i<(par->num1); i++) {
            outbyte( par->pad_character);
		}
    }
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine moves a string to the output buffer  */
/* as directed by the padding and positioning flags. */
/*                                                   */
static void outs(const char *lp, struct params_s *par)
{
    const char *LocalPtr;
	LocalPtr = lp;
    /* pad on left if needed                         */
	if(LocalPtr != NULL) {
		par->len = (int32_t)strlen( LocalPtr);
		padding( !(par->left_flag), par);
		/* Move string to the buffer                     */
		while (((*LocalPtr) != (int8_t)0) && ((par->num2) != 0)) {
			(par->num2)--;
			outbyte(*LocalPtr);
			LocalPtr += 1;
		}
}

    /* Pad on right if needed                        */
    /* CR 439175 - elided next stmt. Seemed bogus.   */
    padding( par->left_flag, par);
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine moves a number to the output buffer  */
/* as directed by the padding and positioning flags. */
/*                                                   */

static void outnum( const int32_t n, const int32_t base, struct params_s *par)
{
    int32_t negative;
	int32_t i;
    char outbuf[32];
    const int8_t digits[] = "0123456789ABCDEF";
    uint32_t num;
    for(i = 0; i<32; i++) {
	outbuf[i] = '0';
    }

    /* Check if number is negative                   */
    if ((par->unsigned_flag == 0) && (base == 10) && (n < 0L)) {
        negative = 1;
		num =(-(n));
    }
    else{
        num = n;
        negative = 0;
    }

    /* Build number (backwards) in outbuf            */
    i = 0;
    do {
		outbuf[i] = digits[(num % (uint32_t)base)];
		i++;
		num /= base;
    } while (num > 0U);

    if (negative != 0) {
		outbuf[i] = '-';
		i++;
	}

    outbuf[i] = '\0';
    i--;

    /* Move the converted number to the buffer and   */
    /* add in the padding where needed.              */
    par->len = (int32_t)strlen(outbuf);
    padding( !(par->left_flag), par);
    while (&outbuf[i] >= outbuf) {
      outbyte( outbuf[i] );
		i--;
}
    padding( par->left_flag, par);
}
/*---------------------------------------------------*/
/*                                                   */
/* This routine moves a 64-bit number to the output  */
/* buffer as directed by the padding and positioning */
/* flags. 											 */
/*                                                   */
#if defined (__aarch64__) || defined (__arch64__)
static void outnum1( const int64_t n, const int32_t base, params_t *par)
{
    int32_t negative;
	int32_t i;
    char outbuf[64];
    const int8_t digits[] = "0123456789ABCDEF";
    uint64_t num;
    for(i = 0; i<64; i++) {
	outbuf[i] = '0';
    }

    /* Check if number is negative                   */
    if ((par->unsigned_flag == 0) && (base == 10) && (n < 0L)) {
        negative = 1;
		num =(-(n));
    }
    else{
        num = (n);
        negative = 0;
    }

    /* Build number (backwards) in outbuf            */
    i = 0;
    do {
		outbuf[i] = digits[(num % base)];
		i++;
		num /= base;
    } while (num > 0);

    if (negative != 0) {
		outbuf[i] = '-';
		i++;
	}

    outbuf[i] = '\0';
    i--;

    /* Move the converted number to the buffer and   */
    /* add in the padding where needed.              */
    par->len = (int32_t)strlen(outbuf);
    padding( !(par->left_flag), par);
    while (&outbuf[i] >= outbuf) {
	outbyte( outbuf[i] );
		i--;
}
    padding( par->left_flag, par);
}
#endif
/*---------------------------------------------------*/
/*                                                   */
/* This routine gets a number from the format        */
/* string.                                           */
/*                                                   */
static int32_t getnum(char** linep)
{
	int32_t n = 0;
	int32_t ResultIsDigit = 0;
	char *cptr = *linep;

	while (cptr != NULL) {
		ResultIsDigit = isdigit(((uint8_t)*cptr));
		if (ResultIsDigit == 0) {
			break;
		}
		n = ((n*10) + (((int32_t)*cptr) - (int32_t)'0'));
		cptr += 1;
	}

	*linep = cptr;
	return n;
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine operates just like a printf/sprintf  */
/* routine. It outputs a set of data under the       */
/* control of a formatting string. Not all of the    */
/* standard C format control are supported. The ones */
/* provided are primarily those needed for embedded  */
/* systems work. Primarily the floating point        */
/* routines are omitted. Other formats could be      */
/* added easily by following the examples shown for  */
/* the supported formats.                            */
/*                                                   */

/* void esp_printf( const func_ptr f_ptr,
   const uint64_t ctrl1, ...) */
#if  defined (__aarch64__) && HYP_GUEST && EL1_NONSECURE && XEN_USE_PV_CONSOLE
void printf( const int8_t *ctrl1, ...){
	XPVXenConsole_Printf(ctrl1);
}
#else
int printf( const char *ctrl1, ...)
{
	va_list argp;

	va_start(argp, ctrl1);

	vprintf(ctrl1, argp);

	va_end(argp);

  return 0;
}
#endif

/* This routine is equivalent to vprintf routine */
int vprintf(const char *ctrl1, va_list argp)
{
	int32_t Check;
#if defined (__aarch64__) || defined (__arch64__)
    int32_t long_flag;
#endif
    int32_t dot_flag;

    params_t par;

    uint8_t ch;
    char *ctrl = (char *)ctrl1;

    while ((ctrl != NULL) && (*ctrl != (char)0)) {

        /* move format string chars to buffer until a  */
        /* format control is found.                    */
        if (*ctrl != '%') {
            outbyte(*ctrl);
			ctrl += 1;
            continue;
        }

        /* initialize all the flags for this format.   */
        dot_flag = 0;
#if defined (__aarch64__) || defined (__arch64__)
		long_flag = 0;
#endif
        par.unsigned_flag = 0;
		par.left_flag = 0;
		par.do_padding = 0;
        par.pad_character = ' ';
        par.num2=32767;
		par.num1=0;
		par.len=0;

 try_next:
		if(ctrl != NULL) {
			ctrl += 1;
		}
		if(ctrl != NULL) {
			ch = (uint8_t)*ctrl;
		} else {
			break;
		}

        if (isdigit(ch) != 0) {
            if (dot_flag != 0) {
                par.num2 = getnum(&ctrl);
			}
            else {
                if (ch == (uint8_t)'0') {
                    par.pad_character = '0';
				}
				if(ctrl != NULL) {
			par.num1 = getnum(&ctrl);
				}
                par.do_padding = 1;
            }
            if(ctrl != NULL) {
			ctrl -= 1;
			}
            goto try_next;
        }

        switch (tolower(ch)) {
            case '%':
                outbyte( '%');
                Check = 1;
                break;

            case '-':
                par.left_flag = 1;
                Check = 0;
                break;

            case '.':
                dot_flag = 1;
                Check = 0;
                break;

            case 'l':
            #if defined (__aarch64__) || defined (__arch64__)
                long_flag = 1;
            #endif
                Check = 0;
                break;

            case 'u':
                par.unsigned_flag = 1;
                /* fall through */
            case 'i':
            case 'd':
                #if defined (__aarch64__) || defined (__arch64__)
                if (long_flag != 0){
			        outnum1((int64_t)va_arg(argp, int64_t), 10L, &par);
                }
                else {
                    outnum( va_arg(argp, int32_t), 10L, &par);
                }
                #else
                    outnum( va_arg(argp, int32_t), 10L, &par);
                #endif
				Check = 1;
                break;
            case 'p':
                #if defined (__aarch64__) || defined (__arch64__)
                par.unsigned_flag = 1;
			    outnum1((int64_t)va_arg(argp, int64_t), 16L, &par);
			    Check = 1;
                break;
                #endif
            case 'X':
            case 'x':
                par.unsigned_flag = 1;
                #if defined (__aarch64__) || defined (__arch64__)
                if (long_flag != 0) {
				    outnum1((int64_t)va_arg(argp, int64_t), 16L, &par);
				}
				else {
				    outnum((int32_t)va_arg(argp, int32_t), 16L, &par);
                }
                #else
                outnum((int32_t)va_arg(argp, int32_t), 16L, &par);
                #endif
                Check = 1;
                break;

            case 's':
                outs( va_arg( argp, char *), &par);
                Check = 1;
                break;

            case 'c':
                outbyte( (int8_t)va_arg( argp, int32_t));
                Check = 1;
                break;

            case '\\':
                switch (*ctrl) {
                    case 'a':
                        outbyte( ((int8_t)0x07));
                        break;
                    case 'h':
                        outbyte( ((int8_t)0x08));
                        break;
                    case 'r':
                        outbyte( ((int8_t)0x0D));
                        break;
                    case 'n':
                        outbyte( ((int8_t)0x0D));
                        outbyte( ((int8_t)0x0A));
                        break;
                    default:
                        outbyte( *ctrl);
                        break;
                }
                ctrl += 1;
                Check = 0;
                break;

            default:
		Check = 1;
		break;
        }
        if(Check == 1) {
			if(ctrl != NULL) {
				ctrl += 1;
			}
                continue;
        }
        goto try_next;
    }

  return 0;
}
/*---------------------------------------------------*/
