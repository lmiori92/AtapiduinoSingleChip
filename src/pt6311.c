/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    Lorenzo Miori (C) 2019 [ 3M4|L: memoryS60<at>gmail.com ]

    Version History
        * 1.0 initial

*/

/* Standard includes */

#include <stdint.h>
#include <stddef.h>

/* AVR includes */

#include <avr/io.h>
#include <util/delay.h>

/* Dependencies */
#include "pt6311.h"

#define D0_7_PORT       (PORTB)
#define D0_7_DDR        (DDRB )
#define D0_7_PIN        (PINB )

#define PIN_DOUT        (0)       /**< PB0 */
#define PIN_DIN         (1)       /**< PB1 */
#define PIN_CLK         (2)       /**< PB2 */
#define PIN_STB         (6)       /**< PC6 - Reset */
#define PORT_STB        (PORTC)
#define DDR_STB         (DDRC)

void pt6311_init(void)
{
    // Initialize D0-D7 as inputs
    // Initialize D8-D15 as inputs
    /* Initialize STB, CLK, and DIN as outputs */
    D0_7_DDR |= (1 << PIN_CLK) | (1 << PIN_DIN);
    DDR_STB  |= (1 << PIN_STB);
    /* Initialize DOUT as input with pull-up */
    D0_7_DDR &= ~(1 << PIN_DOUT);
    D0_7_PORT |= (1 << PIN_DOUT);
    /* Set STB and CLK high, to inhibit the chip, keep DIN low */
    D0_7_PORT |=  (1 << PIN_CLK);
    PORT_STB  |=  (1 << PIN_STB);
    D0_7_PORT &= ~(1 << PIN_DIN);
}

void pt6311_shift_out(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
      D0_7_PORT &= ~(1 << PIN_CLK);
      (data & 0x1) ? (D0_7_PORT |= (1 << PIN_DIN)) : (D0_7_PORT &= ~(1 << PIN_DIN));
      data >>= 1;
      _delay_us(1); /* 400 nanoseconds from datasheet; we do 1000 */
      D0_7_PORT |= (1 << PIN_CLK);
      _delay_us(1); /* 400 nanoseconds from datasheet; we do 1000 */
    }
}

void pt6311_shift_in(uint8_t *data)
{
    *data = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
      D0_7_PORT &= ~(1 << PIN_CLK);
      _delay_us(1); /* 400 nanoseconds from datasheet; we do 1000 */
      if ((D0_7_PIN >> PIN_DOUT) & 0x1) *data |= 0x1;
      *data <<= 1;
      D0_7_PORT |= (1 << PIN_CLK);
      _delay_us(1); /* 400 nanoseconds from datasheet; we do 1000 */

    }
}

void pt6311_write(uint8_t command, const uint8_t *data, uint8_t data_len)
{
    PORT_STB &= ~(1 << PIN_STB);   /* STB low  -> assert */
    _delay_us(1);                   /* 1 microsecond from datasheet STB assertion time */
    pt6311_shift_out(command);      /* shift out the command */
    if ((data != NULL) && (data_len > 0))
    {
      do
      {
        data_len--;
        pt6311_shift_out(*data);
        data++;
      } while(data_len > 0);
    }
    PORT_STB |= (1 << PIN_STB);    /* STB high -> deassert */
    _delay_us(1);                   /* 1 microsecond from datasheet STB assertion time */
}

void pt6311_read(uint8_t command, uint8_t *data, uint8_t data_len)
{
    PORT_STB &= ~(1 << PIN_STB);   /* STB low  -> assert */
    _delay_us(1);                   /* 1 microsecond from datasheet STB assertion time */
    pt6311_shift_out(command);      /* shift out the command */
        _delay_us(1);                   /* 1 microsecond from datasheet STB assertion time */
    if ((data != NULL) && (data_len > 0))
    {
      do
      {
        data_len--;
        pt6311_shift_in(data);
        data++;
      } while(data_len > 0);
    }
    PORT_STB |= (1 << PIN_STB);    /* STB high -> deassert */
    _delay_us(1);                   /* 1 microsecond from datasheet STB assertion time */
}

void pt6311_setup(void)
{
  uint8_t demo_buffer[3] = { 0x00, 0x00, 0x00 };
  pt6311_init();

  pt6311_read(0x46, demo_buffer, 3);
  if (KEY_PRESSED(demo_buffer, KEY_POWR_MASK, KEY_POWR_BYTE))
  {
      demo_buffer[0] = 0xFF;
            demo_buffer[1] = 0xFF;
                  demo_buffer[2] = 0xFF;
  }

  pt6311_write(0x00, NULL, 0);  /* COMMAND 1: DISPLAY MODE SETTING COMMANDS */
  pt6311_write(0x8C, NULL, 0);  /* COMMAND 4: display control commands */
  pt6311_write(0x40, NULL, 0);  /* COMMAND 2: DATA SETTING COMMANDS */
  pt6311_write(0xC0, demo_buffer, sizeof(demo_buffer));  /* COMMAND 3: ADDRESS SETTING COMMANDS */
  pt6311_write(0xC3, demo_buffer, sizeof(demo_buffer));  /* COMMAND 3: ADDRESS SETTING COMMANDS */
  pt6311_write(0xC6, demo_buffer, sizeof(demo_buffer));  /* COMMAND 3: ADDRESS SETTING COMMANDS */
  pt6311_write(0xC9, demo_buffer, sizeof(demo_buffer));  /* COMMAND 3: ADDRESS SETTING COMMANDS */
  pt6311_write(0xC0 + 0x0C, demo_buffer, sizeof(demo_buffer));  /* COMMAND 3: ADDRESS SETTING COMMANDS */
  pt6311_write(0xC0 + 0x0F, demo_buffer, sizeof(demo_buffer));  /* COMMAND 3: ADDRESS SETTING COMMANDS */
  pt6311_write(0xC0 + 0x12, demo_buffer, sizeof(demo_buffer));  /* COMMAND 3: ADDRESS SETTING COMMANDS */
  pt6311_write(0xC0 + 0x15, demo_buffer, sizeof(demo_buffer));  /* COMMAND 3: ADDRESS SETTING COMMANDS */
  while (KEY_PRESSED(demo_buffer, KEY_POWR_MASK, KEY_POWR_BYTE))
  {
        pt6311_read(0x46, demo_buffer, 3);
  }

  //loopDisplay();
}
