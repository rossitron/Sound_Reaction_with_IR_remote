// FHT/ADC/Serial Print Test App
// GPLv3
// Copyright 2014 Ross Melville
#define ATmega328              // hack/fix for IR interrupts making pops on ATmega328
#define LIN_OUT            1   // use linear fht output function
#define FHT_N             32   // set to 32 point fht
#define PrintInterval  33334   // serial port print interval in uS
#define ADCReset        0xf5   // reset the adc, freq = 1/32, 500 kHz/ 13.5 =~ 36 kHz sampling rate
#define ADCFreeRun      0xe5   // set the adc to free running mode, freq = 1/32, 500 kHz/ 13.5 =~ 36 kHz sampling rate
#define SerialBuad   2666667 // PuTTY works at odd bitrates like 2666667, 1843200 (both Win7 x64 and Ubuntu 12.04 32bit work fine)
#define ANSIMax         8*2 // max char length of charts, doubled because of half blocks

#include <FHT.h>               // http://wiki.openmusiclabs.com/wiki/ArduinoFHT

unsigned int PrintingArray[FHT_N/2];
unsigned int PrintingPeakArray[FHT_N/2];
unsigned int PeakRaw = 0;
unsigned long ADCTimeLast;
byte BadSample = 0;

void setup()
{
  ADCSRA = ADCFreeRun; // set the adc to free running mode
  ADMUX = 0x40; // use adc0
  DIDR0 = 0x01; // turn off the digital input for adc0
}

void loop()
{
  unsigned long TimeExitPrint = micros();     // need to load a vaule in the first time
  while(1)
  {
    for (byte i = 0 ; i < FHT_N ; i++) // save FHT_N samples... get out of this loop ASAP
    {
      int k = ReadADC();
      int r = ReadADC();
      //int o = ReadADC();
      //int n = ReadADC();
      //int p = ReadADC();
      //int q = ReadADC();
      //int t = ReadADC();
      int kr;
      kr = (k + r)/2;
      //kr = (kr + o)/2;
      //kr = (kr + n)/2;
      //kr = (kr + p)/2;
      //kr = (kr + q)/2;
      //kr = (kr + t)/2;
      fht_input[i] = kr;         // put real data into bins
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
      Serial.begin(SerialBuad); // in debug use the serial port
      SerialClearScreen();
      SerialCursorHome();
      for (int Index = ((FHT_N/2) - 1); Index > -1; Index--) // print array backwards so low freq is at bottom
      {
        Serial.print(PrintingArray[Index]);
        if (PrintingArray[Index] < 10000) // yeah... this nesting is ugly, it's a test prog...
        {
          Serial.print(" ");
          if (PrintingArray[Index] < 1000)
          {
            Serial.print(" ");
            if (PrintingArray[Index] < 100)
            {
              Serial.print(" ");
              if (PrintingArray[Index] < 10)
              {
                Serial.print(" ");
              }
            }
          }
        }
        PrintBlocks(map(PrintingArray[Index], 0, PrintingPeakArray[Index], 0, ANSIMax));
      }
      Serial.end();
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
  byte HighByte;
  byte LowByte;
  unsigned long ADCTimeBegin;
  BadSample = 1; // setup while loop to retry on a bad sample
  while (BadSample == 1)
  {
    noInterrupts();           // get lots more nasty pops/error on the measurements without doing this
    ADCTimeBegin = micros();
    while(!(ADCSRA & 0x10));  // wait for adc to be ready
    ADCSRA = ADCReset;        // reset the adc
    LowByte = ADCL;      // fetch adc data low
    HighByte = ADCH;     // fetch adc data high
    unsigned long ADCTime = micros() - ADCTimeBegin;
    interrupts();
    if (ADCTime <= 16) // took under 16uS? good sample
    {
      BadSample = 0;
      ADCTimeLast = ADCTime;
    }
    else
    {
      Serial.begin(SerialBuad);
      Serial.print("Bad Sample! Low Byte: ");
      Serial.print(LowByte);
      Serial.print(", High Byte: ");
      Serial.print(HighByte);
      Serial.print(", ADCTime: ");
      Serial.println(ADCTime);
      Serial.end();
      delay(3000);
    }
  }
  int Output = (HighByte << 8) | LowByte;  // form into an int
  Output -= 0x01FF;                        // form into a signed int at the midrange point of mic input (511 = 0x01FF, 512 = 0x0200;)
  Output <<= 6;                            // form into a 16b signed int
  return Output;
}
