/* ########################################################################################

    Simple IDE ATAPI controller using the Arduino I2C interface and 
    3 PCF8574 I/O expanders. Release 3.1 (adapted for Arduino 1.0)
    using a 4th PCF8574 I/O expander interfacing to a 1602 LCD.

    Copyright (C) 2012  Carlos Durandal
    Copyright (C) 2019  Lorenzo Miori:
        - removed Arduino library dependency (standard C + gcc libraries + custom modules)
        - removed PCF8574 hardware (Single Chip solution, atmega328p only)
        - implemented pt6311 led-segment driver chip driver (but can work with others, too)

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


 ##########################################################################################
 
    See documentation for schematics and pinout.
    The modified program runs on a single atmega328(p) without requiring the external
    crystal nor the 3x i2c extender.
    For the extra missing Digital Output (nDIOw), AREF has been exploited plus an external
    buffer transistor since AREF can only source current.

    The atmega328p fuses shall be set so that
    the internal 8 MHz clock is selected in order to free the
    PB6 and PB7 i/o pins up. Additionally, RESET is inhibited and selected as GPIO.

 ##########################################################################################
*/

/* Standard includes */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

/* AVR includes */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* Dependencies */
#include "deasplay/deasplay.h"
#include "keypad/keypad.h"
#include "timers/timer.h"

// IDE Register addresses
const uint8_t DataReg = 0xF0;         // Addr. Data register of IDE device.
const uint8_t ErrFReg = 0xF1;         // Addr. Error/Feature (rd/wr) register of IDE device.
const uint8_t SecCReg = 0xF2;         // Addr. Sector Count register of IDE device.
const uint8_t SecNReg = 0xF3;         // Addr. Sector Number register of IDE device.
const uint8_t CylLReg = 0xF4;         // Addr. Cylinder Low register of IDE device.
const uint8_t CylHReg = 0xF5;         // Addr. Cylinder High register of IDE device.
const uint8_t HeadReg = 0xF6;         // Addr. Device/Head register of IDE device.
const uint8_t ComSReg = 0xF7;         // Addr. Command/Status (wr/rd) register of IDE device.
const uint8_t AStCReg = 0xEE;         // Addr. Alternate Status/Device Control (rd/wr) register of IDE device.

// Program Variables
uint8_t dataLval;                     // dataLval and dataHval hold data from/to
uint8_t dataHval;                     // D0-D15 of IDE
uint8_t regval;                       // regval holds addr. of reg. to be addressed on IDE
uint8_t reg;                          // Holds the addr. of the IDE register with adapted
                                   // nDIOR/nDIOW/nRST values to suit purpose.
uint8_t cnt;                          // packet uint8_t counter
uint8_t idx;                          // index used as pointer within packet array
uint8_t paclen = 12;                  // Default packet length
uint8_t s_trck;                       // Holds start track
uint8_t e_trck;                       // Holds end track
uint8_t c_trck;                       // Follows current track while reading TOC
uint8_t c_trck_m;                     // MSF values for current track
uint8_t c_trck_s;
uint8_t c_trck_f;
uint8_t a_trck = 1;                   // Holds actual track from reading subchannel data
uint8_t MFS_M;                        // Holds actual M value from reading subchannel data
uint8_t MFS_S;                        // Holds actual S value from reading subchannel data
uint8_t d_trck;                       // Destination track
uint8_t d_trck_m;                     // MSF values for destination track
uint8_t d_trck_s;
uint8_t d_trck_f;
uint8_t aud_stat = 0xFF;              // subchannel data: 0x11=play, 0x12=pause, 0x15=stop
uint8_t asc;
bool toc;

// Array containing sets of 16 uint8_t packets corresponding to part of the CD-ROM
// ATAPI function set. If the IDE device only supports packets with 12 uint8_t length
// the last 4 bytes are not sent. The great majority of tested devices use 12 byte.

uint8_t fnc[]= {
  0x1B,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=0 Open tray
  0x1B,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=16 Close tray
  0x1B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=32 Stop unit
  0x47,0x00,0x00,0x10,0x28,0x05,0x4C,0x1A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=48 Start PLAY
  0x4B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=64 PAUSE play
  0x4B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=80 RESUME play
  0x43,0x02,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=96 Read TOC
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=112 unit ready
  0x5A,0x00,0x01,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=128 mode sense
  0x42,0x02,0x40,0x01,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=144 rd subch.  
  0x03,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=160 req. sense
  0x4E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00  // idx=176 Stop disk
};

void    ide_initialization(void);
void    loop(void);
static void    highZ(void);
void    Disp_CD_data(void);
void    curr_MSF(void);
void    play(void);
void    stop(void);
void    eject(void);
void    load(void);
void    pause(void);
void    resume(void);
void    stop_disk(void);
void    reset_IDE(void);
void    readIDE(uint8_t reg);
void    writeIDE(uint8_t, uint8_t, uint8_t);
void    BSY_clear_wait();
void    DRQ_clear_wait();
void    DRY_set_wait();
void    SendPac(void);
void    get_TOC(void);
void    read_TOC(void);
void    read_subch_cmd(void);
uint8_t chck_disk(void);
void    unit_ready(void);
void    req_sense(void);
void    init_task_file(void);
void SendPac(void);

// End of Definitions ########################################################################

/**
 * Implementation of the timer interrupt.
 * This timer shall tick at 1 millisecond period (1000kHz)
 */
ISR(TIMER0_COMPA_vect)
{
    /* increment the timer variable */
    milliseconds_since_boot += 1;
}

#define D0_7_PORT       (PORTB)
#define D0_7_DDR        (DDRB )
#define D0_7_PIN        (PINB )

#define D8_15_PORT      (PORTD)
#define D8_15_DDR       (DDRD )
#define D8_15_PIN       (PIND )

#define CTRL_PORT       (PORTC)
#define CTRL_DDR        (DDRC )

#define CTRL_A0_PIN     (0)
#define CTRL_A1_PIN     (1)
#define CTRL_A2_PIN     (2)
#define CTRL_nCS0_PIN   (3)
#define CTRL_nCS1_PIN   (4)
#define CTRL_nDIOW_PIN  (6)  // This is a virtual pin, mapped to AREF ;-)
#define CTRL_nDIOR_PIN  (5)

static inline void io_8255_set_direction(bool read_or_write)
{
    if (read_or_write == true)
    {
        // Initialize D0-D7 as inputs
        // Initialize D8-D15 as inputs
        D0_7_DDR   = 0x00U;
        D8_15_DDR  = 0x00U;
        /* Enable pull-ups */
        D0_7_PORT  = 0xFFU;
        D8_15_PORT = 0xFFU;
    }
    else
    {
        // Initialize D0-D7 as outputs
        // Initialize D8-D15 as outputs
        D0_7_DDR = 0xFFU;
        D8_15_DDR = 0xFFU;
    }
}

static inline void ide_ctrl_port(uint8_t val)
{
    /* Write the value to the port (only 6 bits) */
    CTRL_PORT = val & 0x3FU;
    /* Write the additional virtual pin, mapped to AREF:
     *  the transistor is inverting!
     */
    ADMUX = ((val >> CTRL_nDIOW_PIN) & 0x01U) ? 0x00 : 0x40;
    _delay_us(100);
}

void ide_ctrl_init()
{
    // Initialize (= disable, inverting) the ADC to map the virtual pin to AREF
    ADMUX = 0x00;
    // Disable the TWI interface
    TWCR = 0x00;
    // Initialize control lines as outputs
    CTRL_DDR  = 0xFFU;
    // Initialize all as HIGH
    ide_ctrl_port(0xFFU);
    // Set all pins of all PCF8574 to high impedance inputs.
    highZ();
}

void ide_initialization()
{
// IDE Initialisation Part
// ########################

  uint8_t code1;
  uint8_t code2;

 // Serial.println("Atapiduino Single Chip");
  
//  Serial.setCursor (0,1);                      // cursor to beginning of LCD 2nd. line
 // Serial.println("Release 3.1");

  reset_IDE();                              // Do hard reset
  _delay_ms(3000);                              // This delay waits for the drive to initialise
 // Serial.println("Waiting BSY");
  BSY_clear_wait();                         // The ATAPI spec. allows drives to take up to 
 //   Serial.println("Waiting DRY");
  DRY_set_wait();                           // 31 sec. but all tested where alright within 3s.
  //lcd.clear();
  readIDE(CylLReg);                         // Check device signature for ATAPI capability
  code1 = dataLval;
  readIDE(CylHReg);
  code2 = dataLval;

  //dbg_display_number_and_wait_btn(code1);
  //dbg_display_number_and_wait_btn(code2);

  if ((code1 == 0x14) && (code2 == 0xEB))
  {
    // Serial.println("Found ATAPI Dev.");
  }
  else
  {
   //  Serial.println(code1, HEX);
  //   Serial.println(code2, HEX);
  //   Serial.println("No ATAPI Device!");
     while(1);                          // No need to go ahead.
  }

  writeIDE(HeadReg, 0x00, 0xFF);            // Set Device to Master (Device 0)
  
// Initialise task file
// ####################
   init_task_file();

// Run Self Diagnostic
// ###################
  _delay_ms(3000);
  //lcd.clear ();
 // Serial.println("Self Diag. ");
      readIDE(ErrFReg);
      //dbg_display_number_and_wait_btn(dataLval);
 //   Serial.println(dataLval, HEX);

  writeIDE(ComSReg, 0x90, 0xFF);            // Issue Run Self Diagnostic Command
  readIDE(ErrFReg);
  if(dataLval == 0x01){
//    Serial.println("OK");
  }else{
//        Serial.println("Fail");            // Units failing this may still work fine
  }
  _delay_ms(3000);
  //lcd.clear ();
  //Serial.println("ATAPI Device:");
  //lcd.setCursor (0,1);
// Identify Device
// ###############
  writeIDE (ComSReg, 0xA1, 0xFF);           // Issue Identify Device Command
  _delay_ms(500);                               // Instead of wait for IRQ. Needed by some dev.
//  readIDE(AStCReg); //

  do{
    readIDE(DataReg);
    if (cnt == 0){                                // Get supported packet lenght
      if(dataLval & (1<<0)){                      // contained in lower uint8_t of first word
        paclen = 16;                              // 1st bit set -> use 16 uint8_t packets
      }
    }
    if((cnt > 26) & (cnt < 47)){                      // Read Model
      //dbg_display_number_and_wait_btn(dataHval);
      //dbg_display_number_and_wait_btn(dataLval);
    }
    cnt++;
    readIDE(ComSReg);                             // Read Status Register and check DRQ,
  } while(dataLval & (1<<3));                     // skip rest of data until DRQ=0

  readIDE(AStCReg);
  DRQ_clear_wait();

// Check if unit ready
// ###################
  unit_ready();                                   // Send packet 'test unit ready'
  req_sense();                                    // Send packet 'Request Sense'
  if(asc == 0x29){                                // Req. Sense returns 'HW Reset' 
    unit_ready();                                 // (ASC=29h) at first since we had one.
    req_sense();                                  // New Req. Sense returns if media
  }                                               // is present or not.
  do{
     unit_ready();                                // Wait until drive is ready.
     req_sense();                                 // Some devices take some time
  }while(asc == 0x04);                            // ASC=04h -> LOGICAL DRIVE NOT READY
}

// ############
// End of setup
// ############


// This part reads the push buttons, checks device audio status, interprets operator commands
// and displays the corresponding data depending on the status and/or the commands resulting
// from pressing the push buttons.

void loop(void) {

  uint8_t btnbuf[5] = { 0 } ;

  pt6311_read(0x46, btnbuf, 3);

  /* Associate the raw keypresses to the keypad logic */
  keypad_set_input(BUTTON_EJECT,    KEY_PRESSED(btnbuf, KEY_EJCT_MASK, KEY_EJCT_BYTE));
  keypad_set_input(BUTTON_STOP,     KEY_PRESSED(btnbuf, KEY_STOP_MASK, KEY_STOP_BYTE));
  keypad_set_input(BUTTON_PLAY,     KEY_PRESSED(btnbuf, KEY_PLAY_MASK, KEY_PLAY_BYTE));
  keypad_set_input(BUTTON_FFWD,     KEY_PRESSED(btnbuf, KEY_FFWD_MASK, KEY_FFWD_BYTE));
  keypad_set_input(BUTTON_REW,      KEY_PRESSED(btnbuf, KEY_FREW_MASK, KEY_FREW_BYTE));
  keypad_set_input(BUTTON_POWER,    KEY_PRESSED(btnbuf, KEY_POWR_MASK, KEY_POWR_BYTE));

  /* this logic shall always be run periodically */
  keypad_periodic(timeout(10, SOFT_TIMER_0));

  display_set_cursor(0,0);
  display_clear();
  display_write_number(*(uint16_t*)btnbuf, 0);
  display_periodic();

  // Scan push buttons
  if(keypad_clicked(BUTTON_EJECT) == KEY_CLICK)
  {
    //lcd.clear();
    toc=false;                                    // Set toc invalid
    switch(chck_disk()) {
      case 0x00:                                  // If disk in tray case
      /*Serial.println("      OPEN");*/
      eject();
      break;
      case 0xFF:                                  // If tray closed but no disk in case
      eject();
      /*Serial.println("      OPEN");*/
      break;      
      case 0X71:                                 // If tray open -> close it
      /*Serial.println("      LOAD");*/
      load();
    }
    //lcd.clear();
    a_trck = s_trck;                             // Reset to start track
  }
  
  if(keypad_clicked(BUTTON_STOP) == KEY_CLICK){
    a_trck=s_trck;                               // Reset to start track
    stop_disk();                                 // Stop Disk
    stop();                                      // Stop unit    
    toc=false;
  }
  
  if(keypad_clicked(BUTTON_PLAY) == KEY_CLICK){                  // Play has been pressed
      switch(aud_stat){
         case 0x15:                              // If stopped
         play();                                 // start play
         break;
         case 0x12:                              // if paused
         resume();                               // resume play
         break;
         case 0x11:                              // if playing
         pause();                                // pause playback
      }
    toc=false;                                   // mark TOC unknown in case disk
    //lcd.clear();                                 // is removed using device eject buton
  }                                              // while play in progress

  if(keypad_clicked(BUTTON_FFWD) == KEY_CLICK){
    a_trck = a_trck + 1;                         // a_track becomes next track
    if(a_trck > e_trck){(a_trck = s_trck);}      // over last track? -> point to start track
    get_TOC();                                   // Get MSF for a_trck
    fnc[51] = d_trck_m;                          // Store new play start position 
    fnc[52] = d_trck_s;                          // in play packet and start play
    fnc[53] = d_trck_f;
    play();
    if(aud_stat == 0x12 ||                       // If paused or stopped -> pause
       aud_stat == 0x15)
     {
     pause();
    }
    //lcd.clear();
  }
  
  if(keypad_clicked(BUTTON_REW) == KEY_CLICK){                 // Basically like the NEXT function above
    a_trck = a_trck - 1;                        // only backwards
    if(a_trck < s_trck){(a_trck = e_trck);}
    get_TOC();
    fnc[51] = d_trck_m;
    fnc[52] = d_trck_s;
    fnc[53] = d_trck_f;
    play();
    if(aud_stat == 0x12 ||
       aud_stat == 0x15)
     {
     pause();
    }
    //lcd.clear();
  }
  
  if(timeout(250, SOFT_TIMER_1)){          // This part will periodically check the
    read_subch_cmd();                           // current audio status and update the display
    if(aud_stat==0x11){                         // accordingly.
      //lcd.home();
        /*Serial.println("      PLAY    ");*/
      curr_MSF();                               // Display pickup position
    }
    if(aud_stat==0x12){
      //lcd.home();
        /*Serial.println("     PAUSE   ");*/
      curr_MSF();
    }
    if(aud_stat==0x15 && !toc){                  // If stopped and TOC invalid
      get_TOC();                                // try to read TOC 
      Disp_CD_data();                           // display TOC data and set TOC valid
      toc=true;                                 // to prevent reading over and over
    }
    if(aud_stat==0x00){                         // Audio status 0 covers all other posible
      //lcd.clear();                              // states not decoded by this sketch and
      /*Serial.println("    NO DISC");*/                 // handles them as NO DISC.
    }
  }
}

// #######################################
// Auxiliary functions for displaying data
// #######################################

 void Disp_CD_data(){                          // Used to display track range and
     //lcd.clear();                              // Total playing time as recovered
     /*Serial.println("Tracks  ");                    // from reading the TOC
     Serial.println(s_trck, DEC);
     Serial.println("-");
     Serial.println(e_trck, DEC);*/
     //lcd.setCursor (0,1);
     /*Serial.println("Time   ");
     Serial.println(fnc[54], DEC);
     Serial.println(":");*/
      if(fnc[55] < 10)
      {
        //Serial.println("0");                         // Print a leading 0 for seconds when below 10
      }
      //Serial.println(fnc[55], DEC);
 }

void curr_MSF(){                               // During PLAY or PAUSE operation show the pickup
      //lcd.setCursor (2,1);                     // position as absolute playing time
      /*Serial.println(a_trck, DEC);*/
      //lcd.setCursor (11,1);
      /*Serial.println(MFS_M,DEC);
      Serial.println(":");*/
    /*if(MFS_S < 10)
      {
        Serial.println("0");                         // Print a leading 0 for seconds when below 10
      }
      Serial.println(MFS_S,DEC);*/
}
// ##################################
// Auxiliary functions User Interface
// ##################################

void play(void){
    idx = 48;                                     // pointer to play function and Play
    SendPac();                                    // from MSF location stored at idx=(51-56)
}                                                 // See also doc. sff8020i table 76
void stop(void){
    idx = 32;                                     // pointer to stop unit function
    SendPac();
}
void eject(void){
    idx = 0;                                      // pointer to eject function
    SendPac();
}
void load(void){
    idx = 16;                                     // pointer to load
    SendPac();
}
void pause(void){
     idx = 64;                                    // pointer to hold
     SendPac();
}
void resume(void){
     idx = 80;                                    // pointer to resume
     SendPac();
}
void stop_disk(void){
    idx = 176;                                    // pointer to stop disk function
    SendPac();
}

// Set to high impedance all ports of PCF8475 interfacing to IDE.
static inline void highZ(void){
    /* All outputs high: deassert IDE bus */
    ide_ctrl_port(0xFFU);
    /* All data ports as input */
    io_8255_set_direction(true);
    /* high-z for IDE means we can re-start display operation */
    pt6311_setup_io();
}

// Reset Device
void reset_IDE(void){

  /* STUB ONLY: hard reset is only performed when hard-resetting
   * the microcontroller. nRST is connected to PC6 (Reset).
   */
}

/*
/CS0122/CS1 A2 A1 A0  Addr.   Read Function       Write Function
(CS1FX) (CS3FX)
0: asserted
1: negated
----------------------------------------------------------------------
  1     1    x  x  x   N/A    Data Bus High impedance with nDIOR asserted (low)
  0     1    0  0  0   1F0    Data Register       Data Register
  0     1    0  0  1   1F1    Error Register      (Write Precomp Reg.)
  0     1    0  1  0   1F2    Sector Count        Sector Count
  0     1    0  1  1   1F3    Sector Number       Sector Number
  0     1    1  0  0   1F4    Cylinder Low        Cylinder Low
  0     1    1  0  1   1F5    Cylinder High       Cylinder High
  0     1    1  1  0   1F6    SDH Register        SDH Register
  0     1    1  1  1   1F7    Status Register     Command Register
  1     0    1  1  0   3F6    Alternate Status    Digital Output
  1     0    1  1  1   3F7    Drive Address       Not Used


// IDE Register addresses
const uint8_t DataReg = 0xF0;         // Addr. Data register of IDE device.
const uint8_t ErrFReg = 0xF1;         // Addr. Error/Feature (rd/wr) register of IDE device.
const uint8_t SecCReg = 0xF2;         // Addr. Sector Count register of IDE device.
const uint8_t SecNReg = 0xF3;         // Addr. Sector Number register of IDE device.
const uint8_t CylLReg = 0xF4;         // Addr. Cylinder Low register of IDE device.
const uint8_t CylHReg = 0xF5;         // Addr. Cylinder High register of IDE device.
const uint8_t HeadReg = 0xF6;         // Addr. Device/Head register of IDE device.
const uint8_t ComSReg = 0xF7;         // Addr. Command/Status (wr/rd) register of IDE device.
const uint8_t AStCReg = 0xEE;         // Addr. Alternate Status/Device Control (rd/wr) register of IDE device.

*/

// Read one word from IDE register
void readIDE (uint8_t regval)
{
    reg = regval & ~(1 << CTRL_nDIOR_PIN);    // set nDIOR bit LOW preserving register address

    /* set the direction */
    io_8255_set_direction(true);

    /* Write the control port */
    ide_ctrl_port(reg);

    /* Read IDE data ports */
    dataLval = D0_7_PIN;
    dataHval = D8_15_PIN;

    reg |= (1 << CTRL_nDIOR_PIN);             // set nDIOR bit HIGH preserving register address
    /* Write the control port */
    //ide_ctrl_port(reg);

    highZ();                                  // set all I/O pins to HIGH -> impl. nDIOR release
}

// Write one word to IDE register
void writeIDE (uint8_t regval, uint8_t dataLval, uint8_t dataHval){

  reg = regval | (1 << CTRL_nDIOW_PIN);             // set nDIOW bit HIGH preserving register address

  /* set the direction */
  io_8255_set_direction(false);

  /* Write the control port */
  ide_ctrl_port(reg);
  //_delay_ms(1);
  /* Write IDE data ports */
  D0_7_PORT = dataLval;                   // send data for IDE D8-D15
  D8_15_PORT = dataHval;                  // send data for IDE D0-D7

  reg &= ~(1 << CTRL_nDIOW_PIN);  // set nDIOW LOW preserving register address

  /* Write the control port */
  ide_ctrl_port(reg);

  reg |= (1 << CTRL_nDIOW_PIN);  // set nDIOW HIGH

  /* Write the control port */
  ide_ctrl_port(reg);

  //_delay_ms(1);

  highZ();                              // All I/O pins to high impedance -> impl. nDIOW release
}

// #################################################
// Auxiliary functions ATAPI Status Register related
// #################################################

// Wait for BSY clear
void BSY_clear_wait(void){
  do{
    readIDE(ComSReg);
  } while(dataLval & (1<<7));
}

// Wait for DRQ clear
void DRQ_clear_wait(void){
  do{
    readIDE(ComSReg);
  } while(dataLval & (1<<3));
}

// Wait for DRQ set
void DRQ_set_wait(void){
     do{
        readIDE(ComSReg);
     }while((dataLval & ~(1<<3)) == true);
}

// Wait for DRY set
void DRY_set_wait(void){
     do{
        readIDE(ComSReg);
     }while((dataLval & ~(1<<6)) == true);
}

// ##################################
// Auxiliary functions Packet related
// ##################################

// Send a packet starting at fnc array position idx
void SendPac(void){
     writeIDE (AStCReg, 0b00001010, 0xFF);     // Set nIEN before you send the PACKET command!
     writeIDE(ComSReg, 0xA0, 0xFF);           // Write Packet Command Opcode
     _delay_ms(400);
     for (cnt=0;cnt<paclen;cnt=cnt+2){        // Send packet with length of 'paclen' 
     dataLval = fnc[(idx + cnt)];             // to IDE Data Registeraccording to idx value
     dataHval = fnc[(idx + cnt + 1)];
     writeIDE(DataReg, dataLval, dataHval);
     readIDE(AStCReg);                         // Read alternate stat reg.     
     readIDE(AStCReg);                         // Read alternate stat reg.          
     }
     BSY_clear_wait();
}

void get_TOC(void){
       idx =  96;                             // Pointer to Read TOC Packet
       SendPac();                             // Send read TOC command packet
       _delay_ms(10);
       DRQ_set_wait();
       read_TOC();                            // Fetch result
}

void read_TOC(void){
        readIDE(DataReg);                      // TOC Data Length not needed, don't care
        readIDE(DataReg);                      // Read first and last session
        s_trck = dataLval;
        e_trck = dataHval;        
        do{
           readIDE(DataReg);                   // Skip Session no. ADR and control fields
           readIDE(DataReg);                   // Read curent track number
           c_trck = dataLval;
           readIDE(DataReg);                   // Read M
           c_trck_m = dataHval;                // Store M of curent track
           readIDE(DataReg);                   // Read S and F
           c_trck_s = dataLval;                // Store S of current track
           c_trck_f = dataHval;                // Store F of current track
           
           if (c_trck == s_trck){              // Store MSF of first track
               fnc[51] = c_trck_m;             // 
               fnc[52] = c_trck_s;
               fnc[53] = c_trck_f;            
           }           
           if (c_trck == a_trck){              // Store MSF of actual track
               d_trck_m = c_trck_m;            // 
               d_trck_s = c_trck_s;
               d_trck_f = c_trck_f;            
           }                      
           if (c_trck == 0xAA){                // Store MSF of end position
               fnc[54] = c_trck_m;
               fnc[55] = c_trck_s;
               fnc[56] = c_trck_f;
           }
           readIDE(ComSReg);
        } while(dataLval & (1<<3));            // Read data from DataRegister until DRQ=0
}

void read_subch_cmd(void){
        idx=144;                             // Pointer to read Subchannel Packet
        SendPac();                           // Send read Subchannel command packet
        readIDE(DataReg);                    // Get Audio Status
        if(dataHval==0x13){                  // Play operation successfully completed
          dataHval=0x15;                     // means drive is neither paused nor in play
        }                                    // so treat as stopped
        if(dataHval==0x11||                  // playing
           dataHval==0x12||                  // paused
           dataHval==0x15)                   // stopped
           {aud_stat=dataHval;               // 
        }else{
            aud_stat=0;                      // all other values will report "NO DISC"
        }
        readIDE(DataReg);                    // Get (ignore) Subchannel Data Length
        readIDE(DataReg);                    // Get (ignore) Format Code, ADR and Control
        readIDE(DataReg);                    // Get actual track
        a_trck = dataLval;
        readIDE(DataReg);                    // Get M field of actual MFS data and
        MFS_M = dataHval;                    // store M it
        readIDE(DataReg);                    // get S and F fields
        MFS_S = dataLval;                    // Store S value
        do{
          readIDE(DataReg);
          readIDE(ComSReg);
        } while(dataLval & (1<<3));          // Read rest of data from Data Reg. until DRQ=0
}

uint8_t chck_disk(void){
     uint8_t disk_ok = 0xFF;                        // assume no valid disk present.
     idx = 128;                                  // Send mode sense packet
     SendPac();                                  // 
     _delay_ms(10);
     DRQ_set_wait();                             // Wait for data ready to read.
     readIDE(DataReg);                           // Read and discard Mode Sense data length
     readIDE(DataReg);                           // Get Medium Type byte
                                                 // If valid audio disk present disk_ok=0x00
     if (dataLval == 0x02 ||
         dataLval == 0x06 ||
         dataLval == 0x12 ||
         dataLval == 0x16 ||
         dataLval == 0x22 ||
         dataLval == 0x26)
         {disk_ok = 0x00;
     }
     if (dataLval == 0x71){                      // Note if door open
        disk_ok = 0x71;
     }
     do{                                         // Skip rest of packet
       readIDE(DataReg);
       readIDE(ComSReg);
     } while(dataLval & (1<<3));
     return(disk_ok);
}

void unit_ready(void){                                // Reuests unit to report status
        idx=112;                                  // used to check_unit_ready
        SendPac();     
}

void req_sense(void){                                 // Request Sense Command is used to check
  idx=160;                                        // the result of the Unit Ready command.
  SendPac();                                      // The Additional Sense Code is used,
  _delay_ms(10);                                      // see table 71 in sff8020i documentation
  DRQ_set_wait();
  cnt=0;
  do{
       readIDE(DataReg);
       if (cnt == 6){
           asc=dataLval;                          // Store Additional Sense Code
       }
       cnt++;
       readIDE(AStCReg);       
       readIDE(ComSReg);
     } while(dataLval & (1<<3));                  // Skip rest of packet
}

void init_task_file(void){
  writeIDE(ErrFReg, 0x00, 0xFF);            // Set Feature register = 0 (no overlapping and no DMA)
  writeIDE(CylHReg, 0x02, 0xFF);            // Set PIO buffer to max. transfer length (= 200h)
  writeIDE(CylLReg, 0x00, 0xFF);
  writeIDE(AStCReg, 0x02, 0xFF);            // Set nIEN, we don't care about the INTRQ signal
  BSY_clear_wait();                         // When conditions are met then IDE bus is idle,
  DRQ_clear_wait();                         // this check may not be necessary (???)
}

// END ####################################################################################


/*
In the final project the STB pin will be always "HIGH" if the IDE port is accessed.
Then, when not accessed, the STB pin will follow the STB output from the microcontroller.

AND gate made by diodes?

*/

uint8_t seven_segment_mapping[] =
{
0xee ,
0x24 ,
0xd6 ,
0xb6 ,
0x3c ,
0xba ,
0xfa ,
0x26 ,
0xfe ,
0xbe
};
/*
 * 0 - some labels
 * 1 - 1st digit
 * 2 - nothing
 * 3 - 3rd digit
 * 4 - 2nd digit
 * 5 - nothing
 * 6 - 5th digit
 * 7 - 4th digit
 * 8 - nothing
 * 9 - 7th
 * 10 - 6th
 * 11 - nothing
 * 12 - 9th
 * 13 - 8th
 * 14 - nothing
 * 15 - disk image
 * 16 - labels like play pause cd ..
 * 17 - l/r
 *
 *
 * */
uint8_t display_buffer[64] = { 0 };

void pt6311_update(void)
{
    /* Write the buffer in chunks of 48 bytes: the maximum
     * chunk in a single transfer that is accepted by the controller.
     */
    for (uint8_t i = 0; i < 64; i+=6)
    {
        pt6311_write(0x40,      NULL, 0);                /* COMMAND 2: DATA SETTING COMMANDS */
        pt6311_write(0xC0 + i,  &display_buffer[i], 6U);     /* COMMAND 3: ADDRESS SETTING COMMANDS */
    }
    pt6311_write(0x40, NULL, 0);                /* COMMAND 2: DATA SETTING COMMANDS */
    pt6311_write(0xC0 + 60,   &display_buffer[60], 4U);     /* COMMAND 3: ADDRESS SETTING COMMANDS */
}
#include <string.h>
void dbg_display_char(const char* str, uint8_t len)
{
    uint32_t bf = 0;

    if (len > 9) {
        len = 9;
    }
//
//    display_buffer[1] = SevenSegmentASCII[str[0] - 32];
//    display_buffer[4] = SevenSegmentASCII[str[1] - 32];
//    display_buffer[3] = SevenSegmentASCII[str[2] - 32];
//    display_buffer[7] = SevenSegmentASCII[str[3] - 32];
//    display_buffer[6] = SevenSegmentASCII[str[4] - 32];
//    display_buffer[10] = SevenSegmentASCII[str[5] - 32];
//    display_buffer[9] = SevenSegmentASCII[str[6] - 32];
//    display_buffer[13] = SevenSegmentASCII[str[7] - 32];
//    display_buffer[12] = SevenSegmentASCII[str[8] - 32];
    //pt6311_update();

    display_clear();
    display_set_cursor(0,0);
    display_write_string(str);
    display_periodic();

//    while(1)
//    {
//        memset(display_buffer, 0, 64);
//        display_buffer[u] = SevenSegmentASCII['8' - 32];
//        pt6311_update();
//        do
//        {
//          pt6311_read(0x46, (uint8_t*)&bf, 3);
//        } while(bf == 0);
//        // wait release
//        do
//        {
//          pt6311_read(0x46, (uint8_t*)&bf, 3);
//        } while(bf != 0);
//
//        u+=1;
//
//
//        if (u > 63) u=0;
//    }


    do
    {
      pt6311_read(0x46, (uint8_t*)&bf, 3);
    } while(bf == 0);
    // wait release
    do
    {
      pt6311_read(0x46, (uint8_t*)&bf, 3);
    } while(bf != 0);
}

void dbg_display_number_and_wait_btn(uint32_t number)
{
  uint32_t bf = 0;
  //display_number(number);

  //snprintf(buf, 16, "%ld", number);
  dbg_display_char("Lorenzo", 9);

  // wait press
  do
  {
    pt6311_read(0x46, (uint8_t*)&bf, 3);
  } while(bf == 0);
  // wait release
  do
  {
    pt6311_read(0x46, (uint8_t*)&bf, 3);
  } while(bf != 0);
}

void display_number(uint32_t number)
{
    uint8_t buffer[3] = { 0, 0, 0 };

    buffer[1] = seven_segment_mapping[(number % 1000UL) % 10];
    number /= 10UL;
    pt6311_write(0x40, NULL, 0);  /* COMMAND 2: DATA SETTING COMMANDS */
    pt6311_write(0xC0, buffer, 3);  /* COMMAND 3: ADDRESS SETTING COMMANDS */

    buffer[1] = seven_segment_mapping[(number % 100UL) % 10];
    buffer[0] = seven_segment_mapping[(number % 100UL) % 10];
    number /= 10UL;
    pt6311_write(0x40, NULL, 0);  /* COMMAND 2: DATA SETTING COMMANDS */
    pt6311_write(0xC3, buffer, 3);  /* COMMAND 3: ADDRESS SETTING COMMANDS */

    buffer[0] = buffer[1] = seven_segment_mapping[(number % 10UL) % 10];
    number /= 10UL;
    pt6311_write(0x40, NULL, 0);  /* COMMAND 2: DATA SETTING COMMANDS */
    pt6311_write(0xC6, buffer, 3);  /* COMMAND 3: ADDRESS SETTING COMMANDS */

    buffer[0] = buffer[1] = seven_segment_mapping[number % 10];
    pt6311_write(0x40, NULL, 0);  /* COMMAND 2: DATA SETTING COMMANDS */
    pt6311_write(0xC9, buffer, 3);  /* COMMAND 3: ADDRESS SETTING COMMANDS */

}

int main(void)
{

    /* Start 1000Hz system timer:
     * - Frequency / Prescaler / Target Frequency - 1 */
    OCR0A = F_CPU / 64 / 1000 - 1;
    TCCR0A = _BV(WGM01);
    TCCR0B = _BV(CS00) | _BV(CS01);     /* 64x prescaler */
    TIMSK0 = _BV(OCIE0A);               /* enable interrupt routine */

    // Initialize the IDE control port
    ide_ctrl_init();
    _delay_ms(100);

    /* Initialize the display library */
    display_init();


    /* Call the "Arduino" setup() */
    ide_initialization();

    /* Init done, enable interrupts and start dancing */
    sei();

    do
    {
        /* Call the periodc logic */
        loop();
    } while(1);
}
