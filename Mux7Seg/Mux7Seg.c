/* ============================================================================
 *
 * STC-1200 Search Timer Communications controller for Ampex MM-1200
 *
 * Copyright (C) 2016-2024, RTZ Professional Audio, LLC
 * All Rights Reserved
 *
 * RTZ is registered trademark of RTZ Professional Audio, LLC
 *
 * This firmware drives the 7-segment display on the tape machine transport
 * using an ATMega88 to drive the multiplexed display segments. The main STC
 * processor communicates to this Mega88 via a standard UART interface.
 *
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/crc16.h>
#include "Mux7Seg.h"
#include "usart.h"
 
/* Fuse settings for ELF programmers. These fuse
 * settings are for 16Mhz external 18pF crystal.
 */
FUSES = {
    .extended = 0xF9,
    .high     = 0xDF,
    .low      = 0xF7
};

//LOCKBITS = (0xFF);

/****************************************************************************
 * Helper Macros
 ***************************************************************************/

/* Turn a port bit on or off */
#define MUX_ON(x)       (PORTD &= ~(_BV(x)))
#define MUX_OFF(x)      (PORTD |= _BV(x))

/* Given 1 Hz = 1000 ms, the equation for hertz to milliseconds is
 * Ms = 1 / Hz * 1000. We can calculate the rate time, in milliseconds,
 * for each timer tick interrupt. Then we can divide the time we want,
 * by the  interrupt rate time, to get the number of ticks needed to meet
 * the blink on/off times in milliseconds desired.
 */

#define TICK_RATE_MS        (1.0f / (float)MUX_RATE_HZ * 1000.0f)

#define BLINK_TIME_MS(ms)   ((uint16_t)((float)ms / TICK_RATE_MS))

#define BLINK_ON_TIME       800
#define BLINK_OFF_TIME      200

/****************************************************************************
 * Static Memory
 ***************************************************************************/

/* BCD 7-seg data buffer */
static SEGDATA g_segdata;

/* 7-Segment interrupt data buffers */
static volatile uint8_t g_seg0 = 0;
static volatile uint8_t g_seg1 = 0;
static volatile uint8_t g_seg2 = 0;
static volatile uint8_t g_seg3 = 0;

const char _copyright[] PROGMEM = {
    "STC-1200 7-SEG MUX, Copyright (C) 2024, RTZ Professional Audio"
};

/****************************************************************************
 * Helper Functions
 ***************************************************************************/

void delay_10ms(int count)
{
    int i;

    for (i=0; i < count; i++)
        _delay_ms(10UL);
}

/* Debug clock tick simulation */
#ifdef DEBUG_MUX
void debug_tick()
{
    if (g_segdata.secs >= 59)
    {
        g_segdata.secs = 0;
    
        ++g_segdata.mins;
    
        if (g_segdata.mins >= 59)
            g_segdata.secs = g_segdata.mins = 0;
    }
    else
    {
        ++g_segdata.secs;
    }
}
#endif

/****************************************************************************
 * Main Program Entry Point
 ***************************************************************************/

int main(void)
{
    int c;
    size_t  i;
    uint8_t buf[8];
    uint8_t csum;
    uint8_t check;	
    
    /* Default time to zero */
    g_segdata.flags = F_PLUS;
    g_segdata.hour  = 0;
    g_segdata.mins  = 0;
    g_segdata.secs  = 0;
    
    /* Initialize the I/O pins */
    io_init();
    
    /* Initialize the UART serial port */
    usart_init();
    
    /* Initialize and start the timer for the 7-seg mux interrupts */
    timer_init();

    /* Enable global interrupts */
    _SEI();  
    
    /* Enter the main loop forever reading display packet data
     * from the serial port.
     */
    
    while(1)
    {
        /* Read a display packet data from the UART. The 
         * display data packet is composed as follows:
         *
         *    Byte   Description
         *    ----   -------------------------------------
         *    [0]    Preamble 1, must be 0x89
         *    [1]    Preamble 2, must be 0xFC
         *    [2]    The SECONDS (0-59)
         *    [3]    The MINUTES (0-59)
         *    [4]    The HOUR digit (0 or 1)
         *    [5]    The SIGN (negative if zero)
         *    [6]    8-Bit Checksum
         */

        /* Read start of frame preamble 0x89 */

        do  {
            if ((c = usart_tgetc(100)) == -1)
                goto TimeOut;

        } while(c != 0x89);

        /* Read start of frame preamble 0xFC */
        
        do  {
            if ((c = usart_tgetc(100)) == -1)
                goto TimeOut;
        } while(c != 0xFC);

        /* Read sign, hour, mins, secs and calculate checksum */
        
        csum = 0;
        
        for (i=0; i < 4; i++)
        {
            buf[i] = 0;
            
            if ((c = usart_tgetc(100)) == -1)
                goto TimeOut;
            
            /* Buffer the rx packet data */
            buf[i]= (uint8_t)c;
            
            /* Update the checksum */
            csum += ((uint8_t)c & 0xFF);
        }

        /* Read the checksum MSB */
        if ((c = usart_tgetc(100)) == -1)
            goto TimeOut;

        check = (uint8_t)c & 0xFF;

        /* Validate the CRC values match */
        
        if (check == csum)
        {
            /* Valid packet received, update display data */
            g_segdata.secs  = buf[0] & 0x3F;
            g_segdata.mins  = buf[1] & 0x3F;
            g_segdata.hour  = buf[2] & 0x01;
            g_segdata.flags = buf[3];

            /* Calculate the 7-segment BCD values (division) to
             * avoid math and division in the timer ISR handler.
             */
            
            //_CLI();
            g_seg0 = g_segdata.secs % 10;
            g_seg1 = (g_segdata.secs - (g_segdata.secs % 10)) / 10;
            g_seg2 = g_segdata.mins % 10;
            g_seg3 = (g_segdata.mins - (g_segdata.mins % 10)) / 10;
            //_SEI();
        }

TimeOut:
        /* Start the loop over and attempt to synchronize again */
        csum = 0;
    }
}

/****************************************************************************
 * Timer Interrupt Handler for 7-Segment Display Multiplexing
 ***************************************************************************/
                        
ISR(TIMER0_COMPA_vect)
{
#if 0
    /* debug port bit toggle */
    PORTD = PORTD ^ _BV(PD2);
#else	
    static uint8_t state = 0;
    static uint16_t blink = 0;
    
    /* Handle blanked state logic */
    if (g_segdata.flags & F_BLANK)
    {
        /* Blank hour, plus is preserved */
        PORTD |= _BV(PD_HOUR);
        /* Turn off all segments */
        MUX_OFF(PD_TEN_MIN);
        MUX_OFF(PD_UNIT_MIN);
        MUX_OFF(PD_TEN_SEC);        
        MUX_OFF(PD_UNIT_SEC);
        goto exit;
    }

    /* Handle blink state logic */
    if (g_segdata.flags & F_BLINK)
    {
        blink++;
        
        /* Blink OFF time */
        if (blink <= BLINK_TIME_MS(BLINK_OFF_TIME))
        {
            /* Blank hour, plus is preserved */
            PORTD |= _BV(PD_HOUR);
            /* Turn off all segments */
            MUX_OFF(PD_TEN_MIN);
            MUX_OFF(PD_UNIT_MIN);
            MUX_OFF(PD_TEN_SEC);
            MUX_OFF(PD_UNIT_SEC);
            goto exit;
        }

        /* Blink ON time */
        if (blink >= BLINK_TIME_MS(BLINK_ON_TIME))
            blink = 0;
    }

    /*** Set digit/segment value for current multiplex state ***/
    
    switch(state)
    {
        /*** TENS OF SECONDS ******************/
        case 0:
            /* set tens of sec segment state */
            MUX_OFF(PD_TEN_MIN);            
            PORTC = g_seg0;
            MUX_ON(PD_UNIT_SEC);
            ++state;
            break;
            
        /*** SECONDS UNIT *********************/
        case 1:
            /* set unit sec segment state */
            MUX_OFF(PD_UNIT_SEC);
            PORTC = g_seg1;
            MUX_ON(PD_TEN_SEC);
            ++state;
            break;
            
        /*** TENS OF MINUTES ******************/
        case 2:
            /* set tens of min segment state */
            MUX_OFF(PD_TEN_SEC);
            PORTC = g_seg2;
            MUX_ON(PD_UNIT_MIN);
            ++state;
            break;
            
        /*** MINUTES UNIT *********************/
        default:
            /* set unit min segment state */
            MUX_OFF(PD_UNIT_MIN);
            PORTC = g_seg3;
            MUX_ON(PD_TEN_MIN);
            state = 0;
            break;
    }
  
    /* If the hour segment is enabled, toggle the 1-hour segment
     * so it's refreshed at the same rate as the other 7-segment
     * digits. We do this so it appears visually the same as the 
     * other segments (as opposed to just on or off).
     */
    
    if (g_segdata.hour)
        PORTD &= ~(_BV(PD_HOUR));
    else
        PORTD |= _BV(PD_HOUR);
        
    /* If the plus sign segment is enabled, set the plus sign
     * either on or off. We don't mux this segment as the 
     * negative segment is hard wired to always on. We want 
     * the sign segments to light at the same intensity so
     * we just set this segment either on or off according
     * sign segment flag.
     */
    
exit:

    if (g_segdata.flags & F_PLUS)
        PORTD &= ~(_BV(PD_PLUS));
    else
        PORTD |= _BV(PD_PLUS);
#endif
}

/****************************************************************************
 * Initialize and start the timer ISR
 ***************************************************************************/

void timer_init(void)
{
    /* On the Mega88 we use 8 bit TIMER0A in CTC mode for the seven
     * segment display multiplexer task.
     *
     * Timer control register TCCR0, setup clock source and clear on match.
     * (CS00|CS02)  - Prescale divide by 1024
     * (WGM01)      - Mode 2, clear timer counter (CTC) on OCR0A match
     */

    /* Set initial timer count value */
    OCR0A  = (uint8_t)(F_CPU / MUX_RATE_HZ);
    /* No clock prescale divide */
    TCCR0B = _BV(CS00);
    /* Normal CTC mode */
    TCCR0A = _BV(WGM01);
    /* Enable timer 0A interrupts */
    TIMSK0 |= _BV(OCIE0A);
}

/****************************************************************************
 * Default I/O Initialization
 ***************************************************************************/

void io_init(void)
{
    /* PORT-C
     *
     * NOTE: PC2 AND PC1 I/O's SWAPPED DUE TO HARDWARE MISTAKE!
     *
     * PC_7SEG_A    PC0  (out) gpio out to 7SEG_A
     * PC_7SEG_D    PC1  (out) gpio out to 7SEG_D
     * PC_7SEG_C    PC2  (out) gpio out to 7SEG_C
     * PC_7SEG_B    PC3  (out) gpio out to 7SEG_B
     */
    
    /* PC0-PC3 all pins low, no pullups */
    PORTC = 0x0;
    /* Set the output pins, all others are inputs */
    DDRC  = _BV(PC_7SEG_A) | _BV(PC_7SEG_B) | _BV(PC_7SEG_C) | _BV(PC_7SEG_D);
    _NOP();

    /* PORT-D
     *
     * PD_RXD       PD0   (in)  USART serial rx data
     * PD_TXD       PD1   (out) USART serial tx data
     * PD_TEN_SEC   PD2   (out) drive 7SEG_TEN_SEC
     * PD_UNIT_SEC  PD3   (out) drive 7SEG_UNIT_SEC
     * PD_TEN_MIN   PD4   (out) drive 7SEG_TEN_MIN
     * PD_UNIT_MIN  PD5   (out) drive 7SEG_UNIT_MIN
     * PD_HOUR      PD6   (out) drive 7SEG_HOUR
     * PD_PLUS      PD7   (out) drive 7SEG_PLUS
     */

    /* all pins low, no pullups */
    PORTD = 0x00;
    /* Set the output pins, all others are inputs */
    DDRD  = _BV(PD_TXD) | _BV(PD_TEN_SEC) | _BV(PD_UNIT_SEC) | _BV(PD_TEN_MIN) |
            _BV(PD_UNIT_MIN) | _BV(PD_HOUR) | _BV(PD_PLUS);
    _NOP();
    
    /* All high to disable */
    PORTD |= _BV(PD_TEN_SEC) | _BV(PD_UNIT_SEC) | _BV(PD_TEN_MIN) |
             _BV(PD_UNIT_MIN) | _BV(PD_HOUR) | _BV(PD_PLUS);

    /* PORT-B
     *
     * PB_SSEL      PB2  (in)  SPI slave select
     * PB_MOSI      PB3  (in)  SPI MOSI slave in
     * PB_MISO      PB4  (out) SPI MISO slave out
     * PB_SCK       PB5  (in)  SPI clock
     */

#ifdef SPI_INTERFACE
    /* all pins low, no pullups */
    PORTB = 0x00;
    /* All pins input for now */
    DDRB  = 0;
    /* Enable pullups on SSEL, MOSI, SCK */
    //PORTB = _BV(PB_SSEL) | _BV(PB_MOSI) | _BV(PB_SCK);
    _NOP();	
    /* Set MISO to output, all others are inputs */
    DDRB  = _BV(PB_MISO);
#endif
}
  
/****************************************************************************
 * SPI slave receive data interrupt handler
 ***************************************************************************/

#ifdef SPI_INTERFACE

void spi_init_slave(void)
{
    uint8_t d;
    
    /* Configure SPI and interrupt enable, 
     *
     * SPR0 = 0 : clock rate select - no effect for slave mode
     * SPR1 = 0 : clock rate select - no effect for slave mode
     * CPHA = 0 : clock phase
     * CPOL = 0 : clock polarity (SCK high when idle)
     * MSTR = 0 : clear for slave mode
     * DORD = 0 : MSB transmitted first
     * SPE  = 1 : SPI enable
     * SPIE = 1 : interrupt enable
     */    
    SPCR = _BV(SPE) | _BV(SPIE);
    
    /* Clear interrupt by reading SPSR & SPDR registers */
    d = SPSR;
    d = SPDR;
    d = 0;
    SPDR = d;
}

/* Function to send and receive a data byte */
unsigned char spi_transaction(unsigned char data)
{
    SPDR = data;                    /* Load data into the buffer */
    while((SPSR & (1<<SPIF) ));		/* Wait until transmission complete */
    return SPDR;                    /* Return received data */
}

/* SPI slave receive data interrupt handler */
ISR(SPI_STC_vect)
{
    uint16_t d;
    static uint16_t data = 0;
    static uint8_t state = 0;
    
    /* Read the data register */
    d = (uint16_t)SPDR;
            
    /* Read the data register */
    if (!state)
    {
        /* First byte, the MSB, read completed */
        state = 1;
        data = (d << 8);
    }
    else
    {
        /* Second byte, the LSB, read completed */
        state = 0;
        data |= d;
        
        /* Read two bytes from the UART for a 16-bit word. This
         * word specifies the sign, hour, minutes and seconds to
         * display on the 7-segment H:MM:SS display with sign. 
         *
         *    S (bits 1-6)     Specifies the SECONDS (0-59).
         *    M (bits 7-12)    Specifies the MINUTES (0-59).
         *    H (bit 13)       Specifies the HOUR digit (0 or 1).
         *    B (bit 14)       Sets BLANKING mode, all segments off.
         *    F (bit 15)       Sets FLASHING mode all segments.
         *    N (bit 16)       Specifies the SIGN as negative.
         * 
         *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
         *    |N|B|F|H|M|M|M|M|M|M|S|S|S|S|S|S|
         *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
         */
    
        /* set seconds segment value */
        g_segdata.secs = data & 0x3F;
                
        /* set minutes segment value */
        g_segdata.mins = (data >> 6) & 0x3F;
                
        /* set hour segment value from bit-13 */
        g_segdata.hour = (data & 0x1000) ? 1: 0;
    
        /* set the blanking bit-14 flag mask */
        if (data & 0x2000)
            g_segdata.flags |= F_BLANK;
        else
            g_segdata.flags &= ~(F_BLANK);
            
        /* set the blink bit-15 flag mask */
        if (data & 0x4000)
            g_segdata.flags |= F_BLINK;
        else
            g_segdata.flags &= ~(F_BLINK);

        /* set plus segment bit-16 flag mask */
        if (data & 0x8000)		
            g_segdata.flags |= F_PLUS;
        else
            g_segdata.flags &= ~(F_PLUS);

        /* Update display mux 7-seg data */
        g_seg0 = g_segdata.secs % 10;
        g_seg1 = (g_segdata.secs - (g_segdata.secs % 10)) / 10;
        g_seg2 = g_segdata.mins % 10;
        g_seg3 = (g_segdata.mins - (g_segdata.mins % 10)) / 10;
    }
}

#endif

/* EOF */