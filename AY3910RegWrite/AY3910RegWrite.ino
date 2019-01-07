/*******************************************************************
 *               AY-3-3910 Register writer for Arduino
 *                 (c) 2014 Manoel "Godzil" Trapier
 *
 * All the code is made by me apart from the the timer code that 
 * was inspired from code found on the internet. I'm sorry, I can't
 * remmember where.
 **************************** Licence ******************************
 * This file is licenced under the licence:
 *                    WTFPL v2 Postal Card Edition:
 *
 *             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
 *                    Version 2, December 2004
 *
 * Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>
 *
 * Everyone is permitted to copy and distribute verbatim or modified
 * copies of this license document, and changing it is allowed as long
 * as the name is changed.
 *
 *            DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
 *   TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION
 *
 *  0. You just DO WHAT THE FUCK YOU WANT TO.
 *  1. If you like this software you can send me a (virtual) postals
 *     card. Details bellow:
 *
 *             < godzil-nospambot at godzil dot net >
 *
 * If you want to send a real postal card, send me an email, I'll
 * give you my address. Of course remove the -nospambot from my
 * e-mail address.
 *
 ******************************************************************/

const int freqOutputPin = 11;   // OC2A output pin for ATmega328 boards
// Constants are computed at compile time
// If you change the prescale value, it affects CS22, CS21, and CS20
// For a given prescale value, the eight-bit number that you
// load into OCR2A determines the frequency according to the
// following formulas:
//
// With no prescaling, an ocr2val of 3 causes the output pin to
// toggle the value every four CPU clock cycles. That is, the
// period is equal to eight slock cycles.
//
// With F_CPU = 16 MHz, the result is 2 MHz.
//
// Note that the prescale value is just for printing; changing it here
// does not change the clock division ratio for the timer!  To change
// the timer prescale division, use different bits for CS22:0 below
const int ocr2aval  = 3; 
// The following are scaled for convenient printing
//
void setup_clock()
{
    pinMode(freqOutputPin, OUTPUT); 
    // Set Timer 2 CTC mode with no prescaling.  OC2A toggles on compare match
    //
    // WGM22:0 = 010: CTC Mode, toggle OC 
    // WGM2 bits 1 and 0 are in TCCR2A,
    // WGM2 bit 2 is in TCCR2B
    // COM2A0 sets OC2A (arduino pin 11 on Uno or Duemilanove) to toggle on compare match
    //
    TCCR2A = ((1 << WGM21) | (1 << COM2A0));
    // Set Timer 2  No prescaling  (i.e. prescale division = 1)
    //
    // CS22:0 = 001: Use CPU clock with no prescaling
    // CS2 bits 2:0 are all in TCCR2B
    TCCR2B = (1 << CS20);
    // Make sure Compare-match register A interrupt for timer2 is disabled
    TIMSK2 = 0;
    // This value determines the output frequency
    OCR2A = ocr2aval;
}

enum { INACTIVE = B00, READ = B01, WRITE = B10, ADDRESS = B11};
void setup_data(int mode)
{
  switch(mode)
  {
    default:
    case READ:
    case INACTIVE:
      DDRD = B00000000; // Set all D port as input
      DDRB &= ~0x03;
      break;
    case ADDRESS:
    case WRITE:
      DDRD = B11111111; // Set all D port as output
      DDRB |= 0x03;
      break;
  }
}

void setup_control()
{
  DDRC = DDRC | B00000011;
  PORTC &= ~B00000011;
}

void set_control(int mode)
{
  PORTC = (PORTC & 111111100) | (mode);
}

void SetData(unsigned char data)
{
  PORTD = data & 0xFC;
  PORTB = data & 0x03;
} 
 
unsigned char GetData(void)
{
  return (PORTD & 0xFC) | (PORTB & 0x03); 
}
  

/* Registers */
enum
{
  REG_FREQ_A_LO = 0,
  REG_FREQ_A_HI,
  REG_FREQ_B_LO,
  REG_FREQ_B_HI,
  REG_FREQ_C_LO,
  REG_FREQ_C_HI,
  
  REG_FREQ_NOISE,
  REG_IO_MIXER,
  
  REG_LVL_A,
  REG_LVL_B,
  REG_LVL_C,
  
  REG_FREQ_ENV_LO,
  REG_FREQ_ENV_HI,
  REG_ENV_SHAPE,
  
  REG_IOA,
  REG_IOB
};

void write_2149_reg(uint8_t reg, uint8_t value)
{
  setup_data(ADDRESS);
  SetData(reg & 0x0F);
  set_control(ADDRESS);
  delayMicroseconds(3);
  set_control(INACTIVE);

  delayMicroseconds(1);
  setup_data(WRITE);
  SetData(value);
  delayMicroseconds(1);
    
  set_control(WRITE);
  delayMicroseconds(5);
  
  set_control(INACTIVE);
  PORTD = 0;
  //setup_data(INACTIVE);
}

uint8_t read_2149_reg(uint8_t reg)
{
  uint8_t ret = 0;
  return ret;
}

const char STX = 0x02;
const char ETX = 0x03;

// Enough buffer to hold 14 registers, plus 2 frame markers all twice
// e.g. LEN = (STX + registers + ETX) * 2
// This buffer is written to in a circular fashion, but written to
// twice per cycle, with an offset of LEN/2.
// This is done to ensure that there is always a contiguous copy
// of the data frame in the buffer  - regardless of whether
// junk is received between frames - and which makes it easier
// to detect a complete frame and read the data out.
char buf[32];
int pw; // buf write pointer
int pr; // buf read pointer

// Example layout of a data frame within the buffer:
//
// S == STX, E == ETX, d == data,
// R == read pointer, W = write pointer
//
// #00000000001111111111222222222233
// #01234567890123456789012345678901
// [......SddddddddddddddE..........]
// *                     R
// ?     W               W
//
// W may have been 21, or 05.
// (check: 05 == (21 + 16) % 32)
//
// R will always be > 15 and < 32;
// 
// if    R   == &ETX == 21,
// then &STX == 06   == 21 - 15
// and  &d   == 07   == 21 - 14
// 
// Assuming no junk is recieved, then the data shown
// as '.' will be a copy of the framed data except that
// it is always wrapped around the end/start of buf.
// It could also of course be junk or an incomplete frame.
// A complete data frame ending at R will never be wrapped.


void setup()
{
  setup_clock();
  setup_control();
  setup_data(INACTIVE);
 
  Serial.begin(115200);
  // Be sure to kill all possible sound by setting volume to zero
  write_2149_reg(REG_LVL_A, B00000000);
  write_2149_reg(REG_LVL_B, B00000000);
  write_2149_reg(REG_LVL_C, B00000000);

  pw = 0;
  pr = 0;
}

void loop()
{
  // For each available byte
  while (Serial.available()) {
    // Read it
    buf[pw] = Serial.read();

    // Ensure the buffer contains duplicate
    // data and update the read pointer
    if (pw < 16) {
      // If writing to the first half of buf,
      // advance the read pointer,
      pr = pw + 16;
      // and copy the current byte forwards
      buf[pr] = buf[pw];
    } else {
      // If writing to the second half of buf,
      // read pointer is the same as write pointer,
      pr = pw;
      // and copy the current byte backwards
      buf[pw - 16] = buf[pw];
    }

    // If there is a framed message ending at pr;
    // NOTE: that this is not entirely watertight,
    // because the register data can take any values,
    // including (0x02, 0x03), then there is a chance
    // that the buf may have STX...ETX 16 bytes apart
    // but these bytes are not the frame markers.
    // TODO needs testing in practice to see how often
    // this occurs.
    // In any case, this framing should be more reliable
    // than assuming that the last 14 bytes recieved are
    // intended for the registers in the order given above.
    if (buf[pr] == ETX && buf[pr - 15] == STX) {
      // Calculate the start of data within the frame
      // within the buffer; this will always end in the
      // second half of buf.
      char* data = buf + pr - 14;
      for (int i = 0; i < 14; ++i)
      {
        write_2149_reg(i, data + i);
      }

      // TODO: we could send an ACK back to the other side
      // for each processed frame?
    }

    // Advance the write pointer
    pw = (pw + 1) % 32;
  }
}
