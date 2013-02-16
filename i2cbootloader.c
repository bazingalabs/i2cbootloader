/***************************************************** 
   RequiredText.c 3/10/08 Joe Pardue 
   Minimum AVR109 bootloader commands 
   ATmega32 Verson 
******************************************************/ 

// I HATE LICENSES LIKE THIS BUT I've been told that without 
// the license then the work is automatically copyrighted in my name 
// since my purpose is to educate, I want the code to used by whoever 
// wants to use it to learn something. If you like it, then visit 
// my website www.smileymicros.com and buy something. 

/* 
 *  BSD License 
 *  ----------- 
 * 
 *  Copyright (c) 2008, Smiley Micros, All rights reserved. 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions are met: 
 * 
 *  - Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *    
 *  - Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution. 
 *    
 *  - Neither the name of the Smiley Micros nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission. 
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 *  ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 *  POSSIBILITY OF SUCH DAMAGE. 
 */ 

// And to further cover my ass, let me add that if you use this software 
// it will destroy whatever machine you use it on and kill anyone in a one 
// kilometer radius. So don't even consider using it for any reason whatsoever! 

#include <inttypes.h>
#include <avr/io.h> 
#include <avr/pgmspace.h> 
#include <avr/boot.h> 
#include <avr/interrupt.h> 
#include <avr/wdt.h> // watchdog timer 
#include <compat/twi.h>
#include "defines.h"

#ifndef cbi
#define cbi(sfr, bit) (sfr &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (sfr |= _BV(bit))
#endif

// USART declarations 
static void USARTInit(); 
static void sendByte( uint8_t ); 
static uint8_t receiveByte( void ); 

// AVR109 Command Parser declaration 
static void AVR109CommandParser(void); 

// AVR109 Command declarations 
static void enterProgramMode(void); 
static void autoIncrementAddress(void); 
static void setAddress(void); 
static void chipErase(void); 
static void selectDeviceType(void); 
static void readSignatureBytes(void); 
static void returnSupportedDeviceCodes(void); 
static void returnSoftwareIdentifier(void); 
static void returnSoftwareVersion(void);
static void returnProgrammerType(void); 
static void checkBlockSupport(void); 
static void blockFlashLoad(uint16_t size); 
static void blockFlashRead(uint16_t size); 

static void twi_init(uint8_t);
static void put_twi_byte(uint8_t);
static uint8_t get_twi_byte();

// Pin definitions for boot or application reset state 
#define   BOOT_STATE_PORT         PORTC 
#define   BOOT_STATE_PIN         PINC 
#define   BOOT_STATE_PIN_NUMBER   PD0  

#define SUPPORTED_DEVICE_CODE    0x73 

// Signature bytes for ATmega32 0x1e950f
#define   SIGNATURE_BYTE_1      0x1E 
#define   SIGNATURE_BYTE_2      0x95 
#define   SIGNATURE_BYTE_3      0x0F 

// Function pointer to jump to the applicaion memory section 
void (*funcptr)( void ) = 0x0000; 

// Used by chipErase function 
#define BOOTSIZE 4096

#define DEBUG_INIT  DDRB = (1<<PB0);

#define DEBUG_ON PORTB = (1<<PB0);
#define DEBUG_OFF PORTB = (0<<PB0);

// SRAM Page buffer for flash page pre-load 
uint8_t pageBuffer[SPM_PAGESIZE]; 
#define UART_RX_BUFFER_SIZE SPM_PAGESIZE 

// AVR109 uses a global address 
uint16_t address; 

// From Peter Fluery AVRFreaks Aug 10 2005 - to remove interrupt Vector table 
// put -nostartfiles in LDFlags, add the following function 
// saves wasted space 
void __jumpMain     (void) __attribute__ ((naked)) __attribute__ ((section (".init9"))); 

void __jumpMain(void) 
{    
    asm volatile ( ".set __stack, %0" :: "i" (RAMEND) ); 
    asm volatile ( "clr __zero_reg__" );        // r1 set to 0 
    asm volatile ( "rjmp main");                   // jump to main() 
} 

int main(void) 
{ 
  //DEBUG_INIT
  // USARTInit(); 
    twi_init(0x10);
//sendByte('?');

   // Use bootloader or application code? 
    if( !(BOOT_STATE_PIN & (1<<BOOT_STATE_PIN_NUMBER)) ) /* If BOOT_STATE_PIN_NUMBER is low, use bootloader. */ 
    { 
      while(1) 
      { 
         // wait for esc character (0x1B) and respond with AVRBOOT 
         if(receiveByte() == 0x1B) 
         { 
            // wait for software identifier request 
            while( receiveByte() != 'S'); 
             
            // answer the request 
            returnSoftwareIdentifier(); 
             
            // begin servicing the commands 
            AVR109CommandParser(); 
         } 
         else sendByte('?'); 
      } 
   } 
   else 
   { 
      // If BOOT_STATE_PIN_NUMBER is high, don't use bootloader. 
      // So... jump to the application 
      funcptr(); 
   } 

   return 0; 
} 


void AVR109CommandParser() 
{ 
   uint8_t cmd,tempRec; 
   uint16_t tempSize; 

   while(1) 
   { 
      cmd = receiveByte(); 

      switch (cmd) 
      { 
         case 'P': 
            enterProgramMode(); 
            break; 
         case 'a': 
            autoIncrementAddress(); 
            break; 
         case 'A': 
            setAddress(); 
            break; 
         case 'e': 
            chipErase();    
            break; 
         case 'L': 
            enterProgramMode(); 
            break; 
         case 'T': 
            selectDeviceType(); 
            break;                
         case 's': 
            readSignatureBytes();    
            break;          
         case 't': 
            returnSupportedDeviceCodes();    
            break; 
         case 'S': 
            returnSoftwareIdentifier();    
            break; 
         case 'V': 
            returnSoftwareVersion();    
            break;          
         case 'p': 
            returnProgrammerType();    
            break;          
         case 'b': 
            checkBlockSupport();    
            break; 
         case 'B': 
         case 'g': 
            tempSize = (receiveByte() << 8) | receiveByte(); 
            // dummy read for type - we only do flash NOT EEPROM 
            tempRec = receiveByte(); 
            if(cmd == 'B') blockFlashLoad( tempSize ); 
            else blockFlashRead( tempSize  ); 
            break;                            
         default: 
            if(cmd != 0x1B) sendByte('?'); 
      } 
   } 
} 


void twi_init(uint8_t address) {
  // enable twi module, acks, and twi interrupt
 // sbi(TWCR,TWEA);
 // sbi(TWCR,TWEN);
  // Enable pullup
 //sbi(PORTC,PINC4);
 //sbi(PORTC,PINC5);
 TWAR = address << 1; 
}

void put_twi_byte(uint8_t data) {
  
  // Check for START + slave address 
  while (!(TWCR & _BV(TWINT)));
  
  if (TW_STATUS == TW_ST_SLA_ACK) {
    TWDR = data;
    cbi(TWCR,TWEA);
  }
  
  // Check for slave data  
  while (!(TWCR & _BV(TWINT)));
  //Serial.println(TW_STATUS, BIN);
  if (TW_STATUS == TW_ST_DATA_NACK ) {
    
    sbi(TWCR,TWEA);
    // Handle STOP
    while (!(TWCR & _BV(TWINT)));
  }

}

uint8_t get_twi_byte() {
  uint8_t ret = 0;
  // Check for START + slave address 
  while (!(TWCR & _BV(TWINT)));
  if (TW_STATUS == TW_SR_SLA_ACK) {
    sbi(TWCR,TWEA);
  }
  
  // Check for slave data  
  while (!(TWCR & _BV(TWINT)));
  if (TW_STATUS == TW_SR_DATA_ACK) {
    ret = TWDR;
    sbi(TWCR,TWEA);
    // Handle STOP
    while (!(TWCR & _BV(TWINT)));
    if (TW_STATUS == TW_SR_STOP) {
      sbi(TWCR,TWEA);
    }
  }
  return ret;
}
/***************************************************** 
   AVR109 Self Programming Commands 
******************************************************/ 

void enterProgramMode() // 'P' 
{ 
   // what else is it going to do? 
   sendByte('\r'); 
} 

void autoIncrementAddress(void) // 'a' 
{ 
   // Yes, this code autoincrements 
   sendByte('Y'); 
} 

void setAddress(void) // 'A' 
{ 
   // Note that flash addresses are in words, not bytes                
    address = receiveByte();    
   address = (address<<8) | receiveByte(); 
   address = address << 1; // convert word address to byte address 

    sendByte('\r');  // respond okay 
} 

void chipErase(void) // 'e' 
{ 
    int i; 
    for(i = 0 ; i < (FLASHEND - (BOOTSIZE * 2)); i += SPM_PAGESIZE) 
   { 
       boot_page_erase_safe(i);   // Erase the page 
       boot_spm_busy_wait();      // Wait until finished. 
   } 
    
   sendByte('\r');  // respond okay 
} 

void selectDeviceType() // 'T' 
{ 
   //dummy read since we only have one device type 
   uint8_t dummy; 
   dummy = receiveByte(); 
   sendByte('\r'); 
} 

void readSignatureBytes(void) // 'S' 
{ 
    sendByte( SIGNATURE_BYTE_3 ); 
    sendByte( SIGNATURE_BYTE_2 ); 
    sendByte( SIGNATURE_BYTE_1 ); 
} 

void returnSupportedDeviceCodes(void) // 't' 
{ 
   sendByte(SUPPORTED_DEVICE_CODE); // Support only this device 
   sendByte(0); // list terminator 
} 

void returnSoftwareIdentifier(void) // 'S' 
{ 
    // Software identifier is 'AVRBOOT' 
   sendByte('A'); 
    sendByte('V'); 
    sendByte('R'); 
    sendByte('B'); 
    sendByte('O'); 
    sendByte('O'); 
    sendByte('T'); 
} 
void returnSoftwareVersion(void) // 'V' 
{ 
    // Software version 
   sendByte('1'); 
    sendByte('7'); 
} 

void returnProgrammerType(void) // 'p' 
{      
   // Get programmer type - serial. 
    sendByte('S'); 
} 

void checkBlockSupport(void) // 'b' 
{ 
    sendByte('Y'); // yes, block load is supported. 
    sendByte((SPM_PAGESIZE>>8) & 0xFF); // send MSB first. 
    sendByte(SPM_PAGESIZE & 0xFF); // send LSB second. 
} 

void blockFlashLoad(uint16_t size) 
{ 
   uint16_t tempAddress = address; 

   uint16_t i,tempWord; 

   // store values to be programmed in temporary buffer 
   for (i=0; i<UART_RX_BUFFER_SIZE; i++) { 
      if (i<size) pageBuffer[i]=receiveByte(); 
      else pageBuffer[i]=0xFF; 
   } 
   //receiveByte();
   DEBUG_ON
   //cli(); 
   i=0; 
    boot_page_erase(address);   // Perform page erase 
    boot_spm_busy_wait();      // Wait until the memory is erased. 

    for(i = 0; i < size; i+=2) 
   { 
      tempWord = pageBuffer[i]; // load the little end then increment i 
      tempWord += (pageBuffer[i+1] << 8); // load the big end 
        boot_page_fill(address,tempWord); 
                          
        address = address + 2;     // word increment 
    } 
    
    boot_page_write(tempAddress); 

    boot_spm_busy_wait(); 

    boot_rww_enable();            // Re-enable the RWW section    
    DEBUG_OFF  
    //sei();
   sendByte('\r'); 
   //sendByte('\r'); 
} 

void blockFlashRead(uint16_t size) 
{ 
   uint8_t data; 
   do { 
      data = pgm_read_byte_near(address++);   // read_program_memory(address,0x00); 
      sendByte(data);                        // send byte 
      size--;                     // reduce number of bytes to read by one 
   } while (size);                  // loop through size 
} 

/***************************************************** 
   Functions from UART_Test 
******************************************************/ 

void USARTInit() 
{ 
   // Set baud rate hard coded to 19200 for 12MHz 
  // UBRRL = 38; 
   // Enable receiver and transmitter 
 //  UCSRB = (1<<RXEN)|(1<<TXEN); 
   // Set frame format: n,8,1 
  // UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);

//  #if F_CPU < 2000000UL && defined(U2X)
//  UCSR0A = _BV(U2X);             /* improve baud rate error by using 2x clk */
//  UBRR0L = (F_CPU / (8UL * UART_BAUD)) - 1;
//#else
//  UBRR0L = (F_CPU / (16UL * UART_BAUD)) - 1;
//#endif
//  UCSR0B = _BV(TXEN0) | _BV(RXEN0); /* tx/rx enable */  
//
unsigned int ubrr = 16000000UL/16/38400;

 /*Set baud rate */
UBRR0H = (unsigned char)(ubrr>>8);
UBRR0L = (unsigned char)ubrr;
/*Enable receiver and transmitter */
UCSR0B = (1<<RXEN0)|(1<<TXEN0);
/* Set frame format: 8data, 2stop bit */
UCSR0C = (1<<USBS0)|(3<<UCSZ00);
} 

void sendByte( uint8_t data ) 
{ 
   wdt_reset(); // reset the watchdog timer, if it is set 
   // Wait for empty transmit buffer 
   //loop_until_bit_is_set(UCSR0A, UDRE0);
   // Put data into buffer, sends the data 
  //UDR0 = data;
   //loop_until_bit_is_set(UCSR0A, UDRE0);
   // Put data into buffer, sends the data 
 // UDR0 = data;
   put_twi_byte(data);
} 

uint8_t receiveByte( void ) 
{ 
   wdt_reset(); // reset the watchdog timer, if it is set 
   // Wait for data to be received    
   // Get and return received data from buffer 
   //loop_until_bit_is_set(UCSR0A, RXC0);
  //return UDR0;
   uint8_t b = get_twi_byte();
//loop_until_bit_is_set(UCSR0A, UDRE0);
   // Put data into buffer, sends the data 
 // UDR0 = b;
   return b;
} 