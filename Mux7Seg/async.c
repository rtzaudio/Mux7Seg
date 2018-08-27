/****************************************************************************
 *
 * STC-1200 Search Timer Communications controller for Ampex MM-1200
 *
 * Copyright (C) 2016-2018, RTZ Professional Audio, LLC
 *
 * All Rights Reserved
 *
 * RTZ is registered trademark of RTZ Professional Audio, LLC
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  1. This source code may not be used, modified, adapted or integrated 
 *     into any commercial or business product without specific written 
 *     permission from the author.
 *
 *  2. This source code and all portions withing is designed for use with
 *     the Micro-RC repeater controller. Use of this code in source or binary
 *     form on repeater controller hardware other than the Micro-RC is
 *     strictly forbidden.
 *
 *  3. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  4. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *  5. The name of the author may not be used to endorse or promote
 *     products derived from this software without specific prior written
 *     permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT,  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/crc16.h>
#include "async.h"

/* Declare our static serial ring buffers and 
 * buffer management data items.
 */

static ASYNC async;

/****************************************************************************
 * USART INITIALIZATION
 ***************************************************************************/

void asi_init(void) 
{
    /* Set baud rate */
    UBRR0L = (uint8_t)((F_CPU / (16UL * BAUD_RATE)) - 1UL);
    UBRR0H = 0;
    /* Enable receiver and transmitter */
    UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<TXCIE0);
    /* Set frame format: 8 data bits, no parity, 1 stop bit */
    //UCSRC  = (1<<URSEL)|(3<<UCSZ0);
    UCSR0C = (3<<UCSZ00);
 
    /* Setup async ring buffer pointers */
    async.rx.head  = 0;     /* tx queue head index */
    async.rx.tail  = 0;     /* tx queue tail index */
    async.rx.count = 0;     /* tx queue size       */

    async.tx.head  = 0;     /* tx queue head index */
    async.tx.tail  = 0;     /* tx queue tail index */
    async.tx.count = 0;     /* tx queue size       */

    /* hook to stdlib printf() functions etc */
	//fdevopen(asi_putc, NULL, 0);
}

/****************************************************************************
 * READ functions
 ***************************************************************************/

/*
 * Name:        ASI_GETC - Get character from receive queue.
 *
 * Synopsis:    int asi_getc(port)
 *              int port;   - port number  (0-16)
 *
 * Description: Gets a character from the receive the queue.
 *
 * Return:      -1 if error occured.
 */

int asi_getc(void)
{
    int     c = -1;
    size_t  head;

    if (async.rx.head != async.rx.tail)
    {
        head = async.rx.head;                       /* get head pointer  */

        c = async.rx.buf[head++];                   /* get char in buff  */

        c &= 0xff;

        --(async.rx.count);                         /* dec buffer count  */

        async.rx.head = (head >= ASYNC_RX_BUFSIZE) ? 0 : head;
    }

    return c;
}

/*
 * Name:        ASI_READ - Read 'n' characters from comm port.
 *
 * Synopsis:    int asi_read(port, buf, buflen)
 *              int port;   - port number
 *              char *buf;  - pointer to data buffer
 *              int buflen; - data buffer length
 *
 * Description: Reads 'n' characters to the comm port and returns
 *              the number of bytes actually read. The number
 *              of bytes read may not equal the amount requested.
 *
 * Return:      Number of characters actually read.
 */

int asi_read(char *buf, size_t buflen)
{
    char *p = buf;
    int  i;
    int  ch;
    int  cnt = 0;

    for (i=0; i < buflen; i++, p++)
    {
        if ((ch = asi_getc()) == -1)
            break;

        *p = (char)ch;

        ++cnt;
    }

    return cnt;

}

/*
 * Name:        ASI_TGETC - Get character from receive queue.
 *
 * Synopsis:    int asi_tgetc(timeout)
 *              int timeout;    - timeout value in 10ms counts (0=none)
 *
 * Description: Gets a character from the receive the queue.
 *
 * Return:      0 if timeout occurred, otherwise non-zero.
 */

int asi_tgetc(uint8_t *p, int timeout)
{
    int c;
    int i;

    if (!timeout)                       /* timeout specified?   */
        return asi_getc();              /* no, just return it   */

    for (i=0; i < timeout; i++)
    {
        if ((c = asi_getc()) != -1)     /* any character found? */
		{
			*p = (uint8_t)c;
            return 1;                   /* yes, return it       */
		}

        _delay_ms(10);
    }

    return 0;
}

/****************************************************************************
 * WRITE functions
 ***************************************************************************/

/*
 * Name:        ASI_PUTC - Put character in transmit queue.
 *
 * Synopsis:    int asi_putc(c)
 *              int c;      - character to place in buffer
 *
 * Description: Places a character in the transmit queue.
 *
 * Return:      -1 if error occured.
 */

#if 1
int asi_putc(char c) 
{
    /* Wait for empty transmit buffer */
    loop_until_bit_is_set(UCSR0A, UDRE0);
    /* Sends a single char over the UART */
    UDR0 = (uint8_t)c;
    return 0;
}
#else
int asi_putc(char c) 
{
    size_t  tail;
    int     status = 0;

    tail = async.tx.tail;                           /* get tail index     */

    async.tx.buf[tail++] = (uint8_t)c;              /* put char in buff   */

    if (tail >= ASYNC_TX_BUFSIZE)                   /* tail wrap around?  */
        tail = 0;

    if (tail == async.tx.head)                      /* buffer overflow?   */
    {
        async.stat.flags |= A_TX_OVERFLOW;          /* yes, set flag      */
        status = -1;
    }
    else
    {
        async.tx.tail = tail;                       /* new buffer tail    */

        ++async.tx.count;                           /* inc buffer count   */

        UCSR0B |= (1<<UDRIE0);                      /* Enable UDRE int */
    }

    return status;
}
#endif



int asi_write(char *buf, size_t buflen)
{
    int  i;
    int  cnt = 0;

    for (i=0; i < buflen; i++)
    {
        if (asi_putc(*(buf+i)) == -1)
            break;
    }

    return cnt;

}

/****************************************************************************
 * USART TX/RX INTERRUPT HANDLERS
 ***************************************************************************/

//SIGNAL(USART_UDRE_vect)
//{
//}

/*** TRANSMIT INTERRUPT HANDLER ***/

ISR(USART_TX_vect)
{
    char    c;
    size_t  n;

    if (async.tx.head == async.tx.tail)             /* get head pointer   */
    {
		UCSR0B &= ~(1<<UDRIE0);                     /* disable UDRE int's */

        async.tx.count = 0;                         /* reset buf count    */
    }
    else
    {
        n = async.tx.head;

        c = (uint8_t)async.tx.buf[n++];             /* get char in buff   */

        --async.tx.count;                           /* dec buffer count   */

        async.tx.head = (n >= ASYNC_TX_BUFSIZE) ? 0 : n;
        
        UDR0 = c;                                   /* tx the character   */
    }
}

/*** RECEIVE INTERRUPT HANDLER ***/

ISR(USART_RX_vect)
{
    char    data;
    uint8_t stat;
    size_t  n;

    /* read line status and character data */

    stat = UCSR0A;                                  /* read status first! */
    data = UDR0;                                    /* read character     */

    if ((stat & (L_FRAMING_ERROR | L_PARITY_ERROR | L_DATA_OVERRUN)) != 0)
    {
        async.stat.line |= stat;                    /* line status reg    */
    }
    else
    {
        n = async.rx.tail;                          /* get tail index     */

        async.rx.buf[n++] = data;                   /* put char in buff   */

        if (n >= ASYNC_RX_BUFSIZE)                  /* tail wrap around?  */
            n = 0;

        if (n == async.rx.head)                     /* buffer overflow?   */
        {
            async.stat.flags |= A_RX_OVERFLOW;      /* yes, set flag      */
        }
        else
        {
            async.rx.tail = n;                      /* new buffer tail    */

            ++(async.rx.count);                     /* inc buffer count   */
        }
    }
}

/* End-Of-File */



