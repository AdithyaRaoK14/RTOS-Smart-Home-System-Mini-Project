/*----------------------------------------------------------------------------
 *      RL-ARM - Library
 *----------------------------------------------------------------------------
 *      Name:    LCD.C
 *      Purpose: LCD module 2x16 driver for ST7066 controller
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <LPC23xx.H>                    /* LPC23xx definitions               */
#include "LCD.h"

#define MCB2300_V1                      /* First version of MCB2300          */

#define IO1DIR     IODIR1
#define IO1SET     IOSET1
#define IO1CLR     IOCLR1
#define IO1PIN     IOPIN1

/* LCD IO definitions */
#ifdef MCB2300_V1
 #define LCD_E     0xC0000000           /* Enable control pin                */
 #define LCD_RW    0x20000000           /* Read/Write control pin            */
 #define LCD_RS    0x10000000           /* Data/Instruction control          */
 #define LCD_CTRL  0xF0000000           /* Control lines mask                */
 #define LCD_DATA  0x0F000000           /* Data lines mask                   */
#else
 #define LCD_E     0x80000000           /* Enable control pin                */
 #define LCD_RW    0x20000000           /* Read/Write control pin            */
 #define LCD_RS    0x10000000           /* Data/Instruction control          */
 #define LCD_CTRL  0xB0000000           /* Control lines mask                */
 #define LCD_DATA  0x0F000000           /* Data lines mask                   */
#endif

/* Local variables */
static U32 lcd_ptr;

/* 8 user defined characters to be loaded into CGRAM (used for bargraph) */
static const U8 UserFont[8][8] = {
  { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
  { 0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10 },
  { 0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18 },
  { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C },
  { 0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E },
  { 0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F },
  { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
  { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 }
};

/* Local Function Prototypes */
static void delay (U32 cnt);
static void lcd_write (U32 c);
static void lcd_write_4bit (U32 c);
static U32  lcd_rd_stat (void);
static void lcd_wr_cmd (U32 c);
static void lcd_wr_data (U32 d);
static void lcd_wait_busy (void);

/*----------------------------------------------------------------------------
 * LCD Driver Interface Functions
 *---------------------------------------------------------------------------*/


/*--------------------------- delay -----------------------------------------*/

static void delay (U32 cnt) {
  /* Delay in while loop cycles. */

  while (cnt--);
}


/*--------------------------- lcd_write_4bit --------------------------------*/

static void lcd_write_4bit (U32 c) {
  /* Write a 4-bit command to LCD controller. */

  IO1DIR |= LCD_DATA | LCD_CTRL;
  IO1CLR  = LCD_RW   | LCD_DATA;
  IO1SET  = (c & 0xF) << 24;
  IO1SET  = LCD_E;
  delay (10);
  IO1CLR  = LCD_E;
  delay (10);
}


/*--------------------------- lcd_write -------------------------------------*/

static void lcd_write (U32 c) {
  /* Write data/command to LCD controller. */

  lcd_write_4bit (c >> 4);
  lcd_write_4bit (c);
}


/*--------------------------- lcd_rd_stat -----------------------------------*/

static U32 lcd_rd_stat (void) {
  /* Read status of LCD controller (ST7066) */
  U32 stat;

  IO1DIR &= ~LCD_DATA;
  IO1CLR  = LCD_RS;
  IO1SET  = LCD_RW;
  delay (10);
  IO1SET  = LCD_E;
  delay (10);
  stat    = (IO1PIN >> 20) & 0xF0;
  IO1CLR  = LCD_E;
  delay (10);
  IO1SET  = LCD_E;
  delay (10);
  stat   |= (IO1PIN >> 24) & 0xF;
  IO1CLR  = LCD_E;
  return (stat);
}


/*--------------------------- lcd_wait_busy ---------------------------------*/

static void lcd_wait_busy (void) {
  /* Wait until LCD controller (ST7066) is busy. */
  U32 stat;

  do {
    stat = lcd_rd_stat ();
  } while (stat & 0x80);                /* Wait for busy flag                */
}


/*--------------------------- lcd_wr_cmd ------------------------------------*/

static void lcd_wr_cmd (U32 c) {
  /* Write command to LCD controller. */

  lcd_wait_busy ();
  IO1CLR = LCD_RS;
  lcd_write (c);
}


/*--------------------------- lcd_wr_data -----------------------------------*/

static void lcd_wr_data (U32 d) {
  /* Write data to LCD controller. */

  lcd_wait_busy ();
  IO1SET = LCD_RS;
  lcd_write (d);
}


/*--------------------------- LCD_init --------------------------------------*/

void LCD_init (void) {
  /* Initialize the ST7066 LCD controller to 4-bit mode. */ 

  IO1DIR |= LCD_CTRL | LCD_DATA;
  IO1CLR  = LCD_RW   | LCD_RS   | LCD_DATA;

  lcd_write_4bit (0x3);                 /* Select 4-bit interface            */
  delay (100000);
  lcd_write_4bit (0x3);
  delay (10000);
  lcd_write_4bit (0x3);
  lcd_write_4bit (0x2);

  lcd_wr_cmd (0x28);                    /* 2 lines, 5x8 character matrix     */
  lcd_wr_cmd (0x0e);                    /* Display ctrl:Disp/Curs/Blnk=ON    */
  lcd_wr_cmd (0x06);                    /* Entry mode: Move right, no shift  */

  LCD_load ((U8 *)&UserFont, sizeof (UserFont));
  LCD_cls ();
}


/*--------------------------- LCD_load --------------------------------------*/

void LCD_load (U8 *fp, U32 cnt) {
  /* Load user-specific characters into CGRAM */
  U32 i;

  lcd_wr_cmd (0x40);                    /* Set CGRAM address counter to 0    */
  for (i = 0; i < cnt; i++, fp++)  {
    lcd_wr_data (*fp);
  }
}

/*--------------------------- LCD_gotoxy ------------------------------------*/

void LCD_gotoxy (U32 x, U32 y) {
  /* Set cursor position on LCD display. Left corner: 1,1, right: 16,2 */
  U32 c;

  c = --x;
  if (--y) {
    c |= 0x40;
  }
  lcd_wr_cmd (c | 0x80);
  lcd_ptr = y*16 + x;
}


/*--------------------------- LCD_cls ---------------------------------------*/

void LCD_cls (void) {
  /* Clear LCD display, move cursor to home position. */

  lcd_wr_cmd (0x01);
  LCD_gotoxy (1,1);
}


/*--------------------------- LCD_cur_off------------------------------------*/

void LCD_cur_off (void) {
  /* Switch off LCD cursor. */

  lcd_wr_cmd (0x0c);
}


/*--------------------------- LCD_on ------ ---------------------------------*/

void LCD_on (void) {
  /* Switch on LCD and enable cursor. */

  lcd_wr_cmd (0x0e);
}


/*--------------------------- LCD_putc --------------------------------------*/

void LCD_putc (U8 c) { 
  /* Print a character to LCD at current cursor position. */

  if (lcd_ptr == 16) {
    lcd_wr_cmd (0xc0);
  }
  lcd_wr_data (c);
  lcd_ptr++;
}


/*--------------------------- LCD_puts --------------------------------------*/

void LCD_puts (U8 *sp) {
  /* Print a string to LCD display. */

  while (*sp) {
    LCD_putc (*sp++);
  }
}


/*--------------------------- LCD_bargraph ----------------------------------*/

void LCD_bargraph (U32 val, U32 size) {
  /* Print a bargraph to LCD display.  */
  /* - val:  value 0..100 %            */
  /* - size: size of bargraph 1..16    */
  U32 i;

  val = val * size / 20;                /* Display matrix 5 x 8 pixels       */
  for (i = 0; i < size; i++) {
    if (val > 5) {
      LCD_putc (5);
      val -= 5;
    }
    else {
      LCD_putc (val);
      break;
    }
  }
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
