
/* ============================================================================
 *
 * STC-1200 Search Timer Communications controller for Ampex MM-1200
 *
 * Copyright (C) 2016-2018, RTZ Professional Audio, LLC
 *
 * All Rights Reserved
 *
 * RTZ is registered trademark of RTZ Professional Audio, LLC
 * ============================================================================
 */

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
#include "usart.h"
#include "Mux7Seg.h"

/****************************************************************************
 * USART INTERRUPT DRIVERN I/O FOR ATMEGA-88
 ***************************************************************************/

static void usart_delay_ms(uint16_t ms);

#define SLEEP_MS(n)     ( usart_delay_ms(n) )

/* Declare our static serial ring buffers and 
 * buffer management data items.
 */

//static int fdev_putc(char ch, FILE *fp);
//static FILE mystdout = FDEV_SETUP_STREAM(fdev_putc, NULL, _FDEV_SETUP_WRITE);

static USART async;

//extern struct segdata g_segdata;

/****************************************************************************
 * USART INITIALIZATION
 ***************************************************************************/

#define BAUD		250000		//38400
#define BAUDRATE	((F_CPU)/(BAUD*16UL)-1)

void usart_init(void) 
{
    /* Setup async ring buffer pointers */
    async.rx.head  = 0;     /* tx queue head index */
    async.rx.tail  = 0;     /* tx queue tail index */
    async.rx.count = 0;     /* tx queue size       */
    async.tx.head  = 0;     /* tx queue head index */
    async.tx.tail  = 0;     /* tx queue tail index */
    async.tx.count = 0;     /* tx queue size       */

	/* HARD CODED FOR 38.4 bps FOR NOW */
    //UBRR0L = 25;	//(uint8_t)(ubbr >> 8);
    //UBRR0H = 0;	// (uint8_t)(ubbr);

	/* Set the baud rate registers */    
    UBRR0H = (BAUDRATE >> 8);
    UBRR0L = BAUDRATE;
	
    /* Enable receiver and transmitter */
    UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);
    /* Set frame format: 8 data bits, no parity, 1 stop bit */
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);

    /* hook to stdlib printf() functions etc */
	//fdevopen(usart_putc, NULL, 0);
    //stdout = &mystdout;
}

/*
 * Name:        USART_DELAY_MS - General purpose ms delay function.
 *
 * Synopsis:    void usart_delay_ms(ms)
 *              int ms;		- millisecond delay value (1=1ms)
 *
 * Description: Gets a character from the receive the queue.
 *
 * Return:      void
 */

void usart_delay_ms(uint16_t ms)
{
    uint16_t d;

	//_delay_ms(10UL);
	
    for (d=0; d < ms; d++) {
        /* 16-bit count - 4 cycles/loop */
        _delay_loop_2((uint16_t)(F_CPU / 4000UL));
    }
}

/****************************************************************************
 * READ functions
 ***************************************************************************/

/*
 * Name:        USART_GETC - Get character from receive queue.
 *
 * Synopsis:    int usart_getc(port)
 *              int port;   - port number  (0-16)
 *
 * Description: Gets a character from the receive the queue.
 *
 * Return:      -1 if error occurred.
 */

int usart_getc(void)
{
    int     c = -1;
    size_t  head;

    if (async.rx.head != async.rx.tail)
    {
        head = async.rx.head;                       /* get head pointer  */

        c = async.rx.buf[head++];                   /* get char in buff  */

        c &= 0xff;

        --(async.rx.count);                         /* dec buffer count  */

        async.rx.head = (head >= USART_RX_BUFSIZE) ? 0 : head;
    }

    return c;
}

/*
 * Name:        USART_TGETC - Get character from receive queue.
 *
 * Synopsis:    int usart_tgetc(port, timeout)
 *              int port;       - port number  (0-16)
 *              int timeout;    - timeout value in 10ms counts (0=none)
 *
 * Description: Gets a character from the receive the queue.
 *
 * Return:      -1 if error occurred.
 */

int usart_tgetc(int timeout)
{
    int c;
    int i;

    if (!timeout)                       /* timeout specified?   */
        return usart_getc();              /* no, just return it   */

    for (i=0; i < timeout; i++)
    {
        if ((c = usart_getc()) != -1)     /* any character found? */
            return c;                   /* yes, return it       */

        SLEEP_MS(10);
    }

    return -1;
}

/*
 * Name:        USART_READ - Read 'n' characters from comm port.
 *
 * Synopsis:    int usart_read(port, buf, buflen)
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

int usart_read(char *buf, size_t buflen)
{
    char *p = buf;
    int  i;
    int  ch;
    int  cnt = 0;

    for (i=0; i < buflen; i++, p++)
    {
        if ((ch = usart_tgetc(10)) == -1)
            break;

        *p = (char)ch;

        ++cnt;
    }

    return cnt;
}

/****************************************************************************
 * WRITE functions
 ***************************************************************************/

/*
 * Name:        USART_PUTC - Put character in transmit queue.
 *
 * Synopsis:    int usart_putc(port, c)
 *              int port;   - port number
 *              int c;      - character to place in buffer
 *
 * Description: Places a character in the transmit queue.
 *
 * Return:      -1 if error occurred.
 */

#if 1
int usart_putc(char c) 
{
    /* Wait for empty transmit buffer */
    loop_until_bit_is_set(UCSR0A, UDRE0); 
    /* Sends a single char over the UART */
    UDR0 = (uint8_t)c;
    return 0;
}
#else
int usart_putc(char c) 
{
    size_t  tail;
    int     status = 0;

    tail = async.tx.tail;                           /* get tail index     */

    async.tx.buf[tail++] = (uint8_t)c;              /* put char in buff   */

    if (tail >= USART_TX_BUFSIZE)                   /* tail wrap around?  */
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



int usart_write(char *buf, size_t buflen)
{
    int  i;
    int  cnt = 0;

    for (i=0; i < buflen; i++)
    {
        if (usart_putc(*(buf+i)) == -1)
            break;
    }

    return cnt;

}

/****************************************************************************
 * USART TX/RX INTERRUPT HANDLERS
 ***************************************************************************/

ISR(USART_TX_vect)
{
}

/*
 * TRANSMIT INTERRUPT HANDLER
 */

ISR(USART_UDRE_vect)
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

        async.tx.head = (n >= USART_TX_BUFSIZE) ? 0 : n;
        
        UDR0 = c;                                   /* tx the character   */
    }
}

/*
 * RECEIVE INTERRUPT HANDLER
 */

ISR(USART_RX_vect)
{
    uint8_t rxd;
    uint8_t stat;
    size_t  n;

    /* read line status and character data */

    stat = UCSR0A;
    rxd  = UDR0;

    if ((stat & (L_FRAMING_ERROR | L_PARITY_ERROR | L_DATA_OVERRUN)) != 0)
    {
        async.stat.line |= stat;                    /* line status reg    */
    }
    else
    {
        n = async.rx.tail;                          /* get tail index     */

        async.rx.buf[n++] = rxd;                    /* put char in buff   */

        if (n >= USART_RX_BUFSIZE)                  /* tail wrap around?  */
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
