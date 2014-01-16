// FHT/ADC/Serial Print Test App
// GPLv3
// Copyright 2014 Ross Melville
#define ATmega328              // hack/fix for IR interrupts making pops on ATmega328
#define LIN_OUT            1   // use linear fht output function
#define FHT_N             64   // set to 32 point fht
#define PrintInterval  33334   // serial port print interval in uS
#define ADCReset        0xf4   // reset the adc, freq = 1/32, 500 kHz/ 13.5 =~ 36 kHz sampling rate
#define ADCFreeRun      0xe4   // set the adc to free running mode, freq = 1/32, 500 kHz/ 13.5 =~ 36 kHz sampling rate
#define SerialBuad   2666667 // PuTTY works at odd bitrates like 2666667, 1843200 (both Win7 x64 and Ubuntu 12.04 32bit work fine)
#define ANSIMax         10*2 // max char length of charts, doubled because of half blocks

#include <FHT.h>               // http://wiki.openmusiclabs.com/wiki/ArduinoFHT

unsigned int PrintingArray[FHT_N/2];
unsigned int PrintingPeakArray[FHT_N/2];
unsigned int PeakRaw = 0;

void setup()
{
  ADCSRA = ADCFreeRun; // set the adc to free running mode
  ADMUX = 0x40; // use adc0
  DIDR0 = 0x01; // turn off the digital input for adc0
  Serial.begin(SerialBuad); // in debug use the serial port
}

void loop()
{
  unsigned long TimeExitPrint = micros();     // need to load a vaule in the first time
  while(1)
  {
    for (byte i = 0 ; i < FHT_N ; i++) // save FHT_N samples... get out of this loop ASAP
    {
      int k = ReadADC();
      fht_input[i] = k;         // put real data into bins
    }          
    
    fht_window();  // window the data for better frequency response
    fht_reorder(); // reorder the data before doing the fht
    fht_run();     // process the data in the fht
    fht_mag_lin(); // take the linear output of the fht
    
    for (byte Index = 0; Index < (FHT_N/2); Index++)
    {
      if (fht_lin_out[Index] > PrintingArray[Index]){PrintingArray[Index] = fht_lin_out[Index];}
      if (fht_lin_out[Index] > PrintingPeakArray[Index]){PrintingPeakArray[Index] = fht_lin_out[Index];}
    }
    if ((micros() - TimeExitPrint) > PrintInterval - 1500)
    {
      //while ((micros() - TimeExitPrint) > PrintInterval + 2){}
      SerialClearScreen();
      SerialCursorHome();
      for (int Index = ((FHT_N/2) - 1); Index > -1; Index--) // print array backwards so low freq is at bottom
      {
        Serial.print(PrintingArray[Index]);
        PrintBlocks(map(PrintingArray[Index], 0, PrintingPeakArray[Index], 0, ANSIMax));
      }
      for (byte Index = 0; Index < (FHT_N/2); Index++){PrintingArray[Index] = 0;} // Clean the main array
      TimeExitPrint = micros();
    } 
  }
}

void SerialCursorHome()
{
  Serial.write(27);       // ESC command
  Serial.print("[H");     // cursor to home command
}

void SerialClearScreen()
{
  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
}

byte PrintBlocks(byte blocks)
{
  byte blockcount = blocks;
  Serial.print("▌");
  for (blocks; blocks > 0; blocks--)
  {
    if (blocks == 1 && blockcount % 2 != 0)
    {
      Serial.print("▌");
    }
    else if (blocks % 2 == 0)
    {
      Serial.print("█");
    }
  }
  Serial.println();
}

int ReadADC()
{
  #ifdef ATmega328 // hack/fix for IR interrupts making pops on ATmega328 ADC
  byte j = 1; // setup j and m to catch while loop
  byte m = 0;
  int k; //Debug Testing
  while ((m <= 1 && j == 1) || (m >= 254 && j == 2) || (m == 245 && j == 1) || (m == 244 && j == 1)) // Pops only _seem_ to be these values
  {
  #endif
    while(!(ADCSRA & 0x10));  // wait for adc to be ready
    ADCSRA = ADCReset;        // reset the adc
    m = ADCL;            // fetch adc data low
    delayMicroseconds(300);   // for testing, bug ***inducing*** 
    j = ADCH;            // fetch adc data high
     //Debug Testing
    k = (j << 8) | m;     // form into an int
    k -= 0x01FF;              // form into a signed int at the midrange point of mic input (511 = 0x01FF, 512 = 0x0200;)
    k <<= 6;                  // form into a 16b signed int
    
    #ifdef ATmega328          // hack/fix for IR interrupts making pops on ATmega328
  }
  #endif
   //Debug Testing
  if (abs(k) >= 600)
  {
    Serial.print(m);
    Serial.print("m,  j:");
    Serial.println(j);
    while(1){}
  }
  
  //int k = (j << 8) | m;     // form into an int
  //k -= 0x01FF;              // form into a signed int at the midrange point of mic input (511 = 0x01FF, 512 = 0x0200;)
  //k <<= 6;                  // form into a 16b signed int
  return k;
}
