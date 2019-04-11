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

/* STC-1200 Hardware rev level, 1=REV-A, 2=REV-B, etc */
#define HARDWARE_REV        2

/* Non-zero to enable watchdog */
#define WATCHDOG_RESET      0

/* Non-zero to enable 500 KHz SPI */
#define SPI_500KHZ          0

/* 7-Segment Refresh Rate */
#define MUX_RATE_HZ         500UL

/*
 * Helper Macros
 */

#define _CLI()  __asm__ __volatile__ ("cli")
#define _SEI()  __asm__ __volatile__ ("sei")
#define _NOP()  __asm__ __volatile__ ("nop")

/* Binary Display Time Data */

struct segdata {
	uint8_t flags;   /* 0=minus/1=plus */
	uint8_t hour;    /* hour (1-12)    */
	uint8_t mins;    /* minutes (0-59) */
	uint8_t secs;    /* seconds (0-59) */
};

/* Bit flags for segdata.flags */
#define F_PLUS		0x01	/* 7-seg plus segment, negative if clear */
#define F_BLINK		0x02	/* blink all seven segment displays      */
#define F_BLANK		0x80	/* blank the entire display if set       */

/*****************************************************************************
 * ATMEGA88 STC-1200 Hardware Definitions
 *****************************************************************************/

/* PORTC (by bit number) */
#define PC_7SEG_A       PC0             /* (out) gpio out to 7SEG_A    */
#define PC_7SEG_D       PC1             /* (out) gpio out to 7SEG_D    */
#define PC_7SEG_C       PC2             /* (out) gpio out to 7SEG_C    */
#define PC_7SEG_B       PC3             /* (out) gpio out to 7SEG_B    */

/* PORTD (by bit number) */
#define PD_RXD          PD0             /* (in)  USART serial rx data  */
#define PD_TXD          PD1             /* (out) USART serial tx data  */
#define PD_TEN_SEC      PD2             /* (out) drive 7SEG_TEN_SEC    */
#define PD_UNIT_SEC     PD3             /* (out) drive 7SEG_UNIT_SEC   */
#define PD_TEN_MIN      PD4             /* (out) drive 7SEG_TEN_MIN    */
#define PD_UNIT_MIN     PD5             /* (out) drive 7SEG_UNIT_MIN   */
#define PD_HOUR         PD6             /* (out) drive 7SEG_HOUR       */
#define PD_PLUS         PD7             /* (out) drive 7SEG_PLUS       */

/* PORTB (by bit number) */
#define PB_SSEL         PB2             /* (in)  SPI slave select      */
#define PB_MOSI         PB3             /* (in)  SPI MOSI slave in     */
#define PB_MISO         PB4             /* (out) SPI MISO slave out    */
#define PB_SCK          PB5             /* (in)  SPI clock             */

/* End-Of-File */