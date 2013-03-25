#include "Rainbow.h"

// Rainbowduino 2.0 is a Duemilanove (ATMega 328)

/******************************************************************
    User editable variables
******************************************************************/

//9600 14400 19200 28800 38400 56000 57600 115200
// Serial communications speed. If faster speeds don't work, try 9600!
// Remember to match this with the Serial speed in your screen sampling software, e.g. Boblight, atmowin, VLC, etc
#define serialSpeed 9600
// How many LED strips we're plugging in
#define ledStrips 4

/******************************************************************

******************************************************************/


// http://www.seeedstudio.com/blog/2009/07/14/rainbowduino-drive-the-12v-led-strip/
// http://www.videolan.org/developers/vlc/modules/video_filter/atmo/README.txt

/*
Color Resolution				Payload					Brightness Levels
12 bit (4bit per color), 4096 Colors		96 bytes (12bit*64=768bit)		16
15 bit (5bit per color), 32768 Colors		120 bytes (15bit*64=960bit)		32
24 bit (8bit per color), 16777216 Colors	24 bytes (24bit*8=192bit)		256
*/

/* Memory Use
   colourBuffer - 2x3x8x2 = 192bytes
   gamma          -           256bytes
*/

/* Protocol Handshakes, etc

LTBL (boblight) - http://blogger.xs4all.nl/loosen/articles/420470.aspx

Comms open:
0x55 x 256
0xFF x 3

Handshake:
0x55
0xAA

with:
<= 0x80   - Start channel
<= 0x80   - Number of channels
0xXX 0xXX - Channel colour in 16 bit, MSB first. Repeated for each channel.

or:
0x81      - Request current colours being displayed
0x02      - Number of bytes to be recieved (will always be 2 for this command)
<= 0x80   - Start channel
0xXX      - Number of channels

0x82      - Write channel info to device
0x02      - Number of bytes to be recieved (will always be 2 for this command)
<= 0x80   - Start channel
0xXX      - Number of channels

0x83      - Open light - lock device for boblight use
0x00      - Number of bytes to be received (will always be 0 for this command)

0x84      - Close light - free device for connections
0x00      - Number of bytes to be received (will always be 0 for this command)

AtmoLight - http://fun3md.blogspot.com/2009/07/atmolight-technical-post.html

0xFF           - Start byte
0x00 0x00      - Start channel (normally 0x00 0x00), low byte first!
0x0F           - Number of channels (Normally 0x0F, or 16)
0xXX 0xXX 0xXX - Channel RGB values, in the following order: Sum (Center), Left, Right, Top, Bottom

or:

0xFF           - Start byte
0x00 0x00      - Start channel (normally 0x00 0x00), low byte first!
0xXX           - 101: Triggerbyte / commandbyte for WB adjust
0xXX           - brightness
0xXX 0xXX 0xXX - Contrast (RGB)
0xXX 0xXX 0xXX - Gamma (RGB)
0xXX           - Contrast (global)
0xXX           - Write command (write to EEPROM)

fnortlicht - https://github.com/fd0/fnordlicht/blob/master/doc/PROTOCOL



Internal sequence:

0xC0 - pulse colours    À
0xC1 - cycle colours    Á
0xCE - block of colour  Í
0xCF - stop lights      Î

So, we can expect one of the following:

0x55 0x55 0x55 ... 0x55 0x55 0x55 0xFF 0xFF 0xFF
0x55 0xAA 0x00 0x0C 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX
0x55 0xAA 0x81 0x02 0x00 0x0C
0x55 0xAA 0x82 0x02 0x00 0x0C
0x55 0xAA 0x83 0x00
0x55 0xAA 0x84 0x00
0xFF 0x00 0x00 0x0F 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX
0xFF 0x00 0x00 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX
0xC0 0xCE 0xCE 0xCE
0xC1 0xCE 0xCE 0xCE
0xCF 0xCE 0xCE 0xCE
*/

/* Rainbowduino Hardware Variables */
// Available channels for plugging LED strips on the board
#define ledChannels 8
// Number of colours on our LED strips
#define ledColours 3
const byte channelCount = ledStrips * ledColours;

/* Onboard LED used for diagnostics */
#define diagLED 13
byte diagLEDState = 0;

/* Colour depths */
#define inputBits 16
//#define outputBits 4
//#define outputBits 7
// The following has to be 2 to the power of our output Bits, either 2 (1 bit), 4 (2 bit), 8 (3 bit), 16 (4 bit), 32 (5 bit), 64 (6 bit), 128 (7 bit), 256 (8 bit), 512 (9 bit), 1024 (10 bit), 2048 (11 bit), 4096 (12 bit), 8192 (13 bit), 16384 (14 bit), 32768 (15 bit) or 65536 (16 bit)
#define outputColours 128
// log2 n = (1/logk 2)logk n
const byte outputBits = log(outputColours) / log(2);
//#define outputColours 16
//#define outputColours 128
//const word outputColours = pow(2, outputBits) - 1;
const byte colourOffset = inputBits - outputBits;
#define pi 3.141592

/* Gamma */
// Static gamma value to use for all LED flashing levels
#define staticGamma 0xE7
// Array of gamma values, which allows us to vary the brightness curve as we light the LEDs
//byte gamma[outputColours];

//const byte serialDataSize = ledChannels * ledColours * 2;
//byte serialData[serialDataSize];

byte serialBuffer[256];
byte serialOutBuffer[256];
byte serialStart = 0x00; // Points to the first filled byte of our serial buffer
byte serialEnd = 0x01; // Points to the first free byte of our serial buffer

byte lightMode;                                          // The lighting mode we're currently using
/* Available Light Modes:
0x55 - Boblight
0xFF - Atmo
0xC0 - Colour pulse
0xC1 - Colour cycle */
/* Protocol handshake bytes */

/*
Hex		Dec		Char	Windows
C0		192		À		183
C1		193		Á		181
C2		194		Â		182
C3		195		Ã		199
C4		196		Ä		142
C5		197		Å		143
C6		198		Æ		146
C7		199		Ç		128
C8		200		È		212
C9		201		É		144
CA		202		Ê		210
CB		203		Ë		211
CC		204		Ì		222
CD		205		Í		214
CE		206		Í		215
CF		207		Î		216
*/

#define bobByte1       0x55
#define bobByte2       0xAA
#define bobMaxChannels 0x80
#define bobRead        0x81
#define bobOpen        0x83
#define bobClose       0x84

#define atmoByte1      0xFF
#define atmoByte2      0x00
#define atmoByte3      0x0F

#define pulseByte      0xC0
#define cycleByte      0xC1
#define fillerByte     0xCE
#define stopByte       0xCF

word level;
//byte level;                                              // An index of the level of colour (gamma) we're currently showing
byte cycleIndex;                                         // An index of the point we are up to in a colour cycling function
#define buffers 2                                        // Number of buffers holding our colour values
byte frontBuffer, backBuffer, currentBuffer;             // used for handling the buffers
word colourBuffer[buffers][ledColours][ledChannels];   // Holds sets of color values, up to 16bit per colour

/******************************************************************
    Initialisation
******************************************************************/

void setup()
{
 initPorts();
 initGfx();       // init Graphics
 setupTimer2();    // init the timer for flashing the LED matrix
 initSerial();      // init Serial UART communication protocol
}

void initPorts(void) { // Configure ports and data register (see http://www.arduino.cc/en/Reference/PortManipulation)
 /*
 DDRD=0xff;        // Port D (digital pins 0-7)  as OUTPUT
 DDRC=0xff;        // Port C (analog  pins 0-5)  as OUTPUT
 DDRB=0xff;        // Port B (digital pins 8-13) as OUTPUT
 PORTD=0;          // Port D (digital pins 0-7)  as READ (LOW)
 PORTB=0;          // Port B (digital pins 8-13) as READ (LOW)
 */
 pinMode(diagLED, OUTPUT);
 switchDisplay(0);
}

void initGfx() { // Initialise the graphics
 level = 0;
 cycleIndex = 0;
 frontBuffer = 0;
 backBuffer = 1;
 currentBuffer = 0;
 //initGamma();
 clearColours();
 switchDisplay(0);
 lightMode = cycleByte;
}

/*
void initGamma()
{
 for (int i = 0; i < outputColours; i++)
 {
  gamma[i] = 0xE7; // Default linear gamma
  // http://electronics.stackexchange.com/questions/1983/correcting-for-non-linear-brightness-in-leds-when-using-pwm
  // 1/(1+EXP(((A1/21)-6)*-1))*255
  // gamma[i] = 255 - (255 / (1 + exp(((gamma[i - 1] / 21) - 6) * -1)))
  //gamma[i] = 0xFF - (pow(1.1, i)); // Progressive gamma, good for 256 gamma levels?
  //flashLED(255 - (int)gamma[i], 50, 500);
  //Serial.println(gamma[i]);
  //int dutyCycle = int (float (1023.0 * pow(((ii + 1) / 1023.0), 2.5)));
  //{0xE7,0xE7,0xE7,0xE7,0xE7,0xE7,0xE7,0xE7,0xE7,0xE7,0xE7,0xE7,0xE7,0xE7,0xE7,0xE7}; // Default linear gamma
  //{0xFF,0xFE,0xFD,0xFC,0xFB,0xF9,0xF7,0xF5,0xF3,0xF0,0xED,0xEA,0xE7,0xE4,0xE1,0xDD}; // Progressive gamma
  //{0xFE,0xFE,0xFD,0xFC,0xFB,0xF9,0xF7,0xF5,0xF3,0xF0,0xED,0xEA,0xE7,0xE4,0xE1,0xDD}; // Another Progressive gamma

 }
}
*/

void loop()
{
 fillSerialBuffer();
 if (serialEnd - serialStart - 0x01 >= 4)
 {
  //flashLED(serialEnd - serialStart - 0x01, 100);
  //delay(1000);
  processSerialData();
 }
 internalSequence();
}

void switchDisplay(byte displayOn)
{
 if (displayOn)
 {
  initTimer2(staticGamma);
  open_lines(ledStrips);
  //lightDiag(0);
  return;
 }
 initTimer2(0x00);
 open_lines(0);
 lightDiag(1);
}

void lightDiag(byte ledOn)
{
 digitalWrite(diagLED, ledOn);
 diagLEDState = ledOn;
}



/********************************************************
   Serial/Buffer Functions
********************************************************/

void initSerial()
{
 //delay(10);
 Serial.begin(serialSpeed);
 //delay(10);
}

byte fillSerialBuffer()
{
 byte bufferAvailable = serialStart - serialEnd;
 if (bufferAvailable) // If we have room in the buffer
 {
  byte serialAvailable = Serial.available(); // Get the number of bytes in the Serial buffer
  if (serialAvailable) // If we have 1 or more byte(s) waiting in the Serial buffer
  {
   if (serialAvailable > bufferAvailable) // If there are more bytes in the serial buffer than we can handle
   {
    serialAvailable = bufferAvailable; // Only fill the buffer to the space we have available
   }
   byte serialReadTo = serialEnd + serialAvailable; // Work out the last byte we should read to
   for (byte i = serialEnd; i != serialReadTo; i++) // For each buffer space we should be filling
   {
    serialBuffer[i] = Serial.read(); // Put the Serial buffer contents in out buffer
   }
   /*
   byte serialPointer;
   for (byte i = 0; i < serialAvailable; i++) // For each buffer space we should be filling
   {
    serialPointer = serialEnd + i;
    serialBuffer[serialPointer] = Serial.read(); // Put the Serial buffer contents in out buffer
   }
   */
   serialEnd = serialReadTo; // Set the new pointer to the end of our data
   return bufferAvailable - serialAvailable; // Return the number of bytes we just wrote to the buffer
  }
 }
 return 0;
}

void processSerialData()
{
 serialStart++;
 lightDiag(1);
 switch (serialBuffer[serialStart - 1])
 {
  case bobByte1:  // Is this the first boblight handshake byte
   if (serialBuffer[serialStart] == bobByte2) // Is this the second handshake byte
   {
    if (serialBuffer[serialStart + 1] <= bobMaxChannels) // If this is a command to update the LEDs (number of starting channel is less than or equal to the maximum number of channels)
    {
     if (serialBuffer[serialStart + 2] > 0x00) // If the number of channels to update is greater than 0
     {
      if (serialBuffer[serialStart + 2] <= bobMaxChannels - serialBuffer[serialStart + 1] + 0x01) // If the number of channels to update, starting at the starting channel, doesn't exceed the maximum number of channels
      {
       serialStart++; // Increment our serial data starting pointer
       byte requiredBuffer = (serialBuffer[serialStart + 1] * 2) + 0x02;
       if (serialEnd - serialStart - 0x01 < requiredBuffer) // If we don't have enough bytes for all the LED strips (2 bytes for each strip), plus our 2 remaining header bytes
       {
        delay(1);
        fillSerialBuffer(); // Grab any new data in the serial buffer
       }
       if (serialEnd - serialStart - 0x01 >= requiredBuffer) // If we have enough bytes for all the LED strips
       {
        bobWriteColours(); // Write our colours to the LEDs
       }
       else // Oops, there aren't enough bytes to light our LEDs
       {
        serialStart = serialEnd - 0x01; // Wipe the rest of the buffer and carry on
       }
      }
     }
    }
    else if (serialBuffer[serialStart + 1] <= bobRead)
    {
     serialStart += 2;
     if (serialBuffer[serialStart] == 0x02)
     {
      lightDiag(0);
      serialStart++;
      if (serialEnd - serialStart - 0x01 < 2) // If we don't have enough bytes for all the LED strips (2 bytes for each strip), plus our 2 remaining header bytes
      {
       delay(1);
       fillSerialBuffer(); // Grab any new data in the serial buffer
      }
      if (serialEnd - serialStart - 0x01 >= 2) // If we have enough bytes for all the LED strips
      {
       bobReadColours(); // Write our current colours from the buffer to the serial port
      }
      else // Oops, there aren't enough bytes to know our start channel and number of channels
      {
       serialStart = serialEnd - 0x01; // Wipe the rest of the buffer and carry on
      }
     }
     serialStart += 2;
    }
    else if (serialBuffer[serialStart + 1] <= bobOpen)
    {
     serialStart += 2;
     if (serialBuffer[serialStart] == 0x00)
     {
      serialStart++;
      lightMode = bobByte1;
      switchDisplay(1);
     }
    }
    else if (serialBuffer[serialStart + 1] <= bobClose)
    {
     serialStart += 2;
     if (serialBuffer[serialStart] == 0x00)
     {
      serialStart++;
      lightMode = stopByte;
      switchDisplay(0);
      clearDisplay();
     }
    }
   }
   break;
  case atmoByte1:
/*
    byte data = Serial.read();
    if (data == 0x00)
    {
     word startChannel = word(data, Serial.read());
     byte numChannels = Serial.read();
     if (numChannels <= 64)
     {
      byte quot, rem;
      for (byte i = 0; i < numChannels; i++)
      {
       quot = (startChannel + i) / 3;
       rem  = (startChannel + i) % 3;
       colourBuffer[backBuffer][rem][quot] = word(Serial.read(), 0x00);
       //colourBuffer[backBuffer][rem][quot] = Serial.read();
      }
     }
     else //Gamma control, etc
     {
      
     }
    }
    break;
 */
   case pulseByte:
    //if ()
    lightMode = pulseByte;
    switchDisplay(1);
    break;
   case cycleByte:
    lightMode = cycleByte;
    switchDisplay(1);
    break;
   case stopByte:
    lightMode = stopByte;
    switchDisplay(0);
 }
}

void internalSequence()
{
 switch (lightMode)
 {
  case cycleByte:
   copyBuffer(frontBuffer); // Copy front buffer to back buffer
   cycleColours(backBuffer); // Increment the buffer, to make it pretty!
   //printBuffer(backBuffer); // Print the buffer to the serial port
   swapBuffers();
   break;
  case pulseByte:
   copyBuffer(frontBuffer); // Copy front buffer to back buffer
   fadeColours(backBuffer);
   swapBuffers();
   break;
 }
}

/********************************************************
   BobLight Functions
********************************************************/

void bobWriteColours()
{
 byte quot, rem;
 byte firstChannel = serialBuffer[serialStart];
 serialStart++;
 byte lastChannel = firstChannel + serialBuffer[serialStart] - 0x01;
 serialStart++;
 for (byte i = serialBuffer[serialStart]; i <= lastChannel; i++)
 {
  quot = i / 3;
  rem  = i % 3;
  colourBuffer[backBuffer][rem][quot] = word(serialBuffer[serialStart], serialBuffer[serialStart + 1]);
  serialStart += 2;
 }
 swapBuffers();
}

void bobReadColoursOld()
{
 byte startChannel = serialBuffer[serialStart];
 byte numChannels =  serialBuffer[serialStart + 1];
 byte j = 4;
 serialOutBuffer[0] = bobByte1;
 serialOutBuffer[1] = bobByte2;
 if (startChannel >= channelCount || numChannels <= 0)
 {
  serialOutBuffer[2] = 0x00;
  serialOutBuffer[3] = 0x00;
 }
 else
 {
  serialOutBuffer[2] = startChannel;
  serialOutBuffer[3] = numChannels;
  byte quot, rem;
  for (byte i = startChannel; i < startChannel + numChannels; i++)
  {
   quot = i / 3;
   rem  = i % 3;
   serialOutBuffer[j] = colourBuffer[frontBuffer][rem][quot] >> 8;
   j++;
   serialOutBuffer[j] = colourBuffer[frontBuffer][rem][quot] & 0xFF;
   j++;
  }
 }
 //delay(5);
 Serial.write(serialOutBuffer, j);
 //delay(5);
}

void bobReadColours()
{
 byte startChannel = serialBuffer[serialStart];
 byte numChannels =  serialBuffer[serialStart + 1];
 Serial.write(bobByte1);
 Serial.write(bobByte2);
 if (startChannel >= channelCount || numChannels <= 0)
 {
  Serial.write(0);
  Serial.write(0);
 }
 else
 {
  numChannels = min(channelCount - startChannel, numChannels);
  Serial.write(startChannel);
  Serial.write(numChannels);
  byte quot, rem;
  for (byte i = startChannel; i < startChannel + numChannels; i++)
  {
   quot = (startChannel + i) / 3;
   rem  = (startChannel + i) % 3;
   Serial.write(colourBuffer[frontBuffer][rem][quot] >> 8);
   Serial.write(colourBuffer[frontBuffer][rem][quot] & 0xFF);
  }
 }
}

/********************************************************
    LED cycling Functions (via manipulation of buffers)
********************************************************/

/* Byte and word manipulation */

byte sineByte(byte fromByte)
{
 return sin(((fromByte * pi * 2 / 256) + 1)) * (256 / 2);
}

byte wordToByte(word fromWord)
{
 return fromWord >> 8;
}

word byteToWord(byte fromByte)
{
 return word(fromByte, 0x00);
}

void incDecCycleIndex()
{
 cycleIndex = incDecByte(cycleIndex);
}

byte sineCycleIndex()
{
 return sineByte(cycleIndex++);
}

byte incDecByte(byte fromByte)
{
 if (fromByte == 0xFE)
 {
  return 0xFF;
 }
 else if (fromByte == 0x01)
 {
  return 0x00;
 }
 int increment = 2;
 if (fromByte % 2)
 {
  increment = - increment;
 }
 return fromByte + increment;
}

word incDecWord(word fromWord)
{
 return word(incDecByte(fromWord >> 8), 0x00);
}

byte shiftByte(byte fromByte, byte offset, byte multiplier)
{
 return ((fromByte + offset) * multiplier) % 256;
}

word shiftWord(word fromWord, byte offset, byte multiplier)
{
 return word(((((fromWord >> 8) + offset) * multiplier) % 256), 0x00);
}

/* Colour array manipulation */

void cycleColours(byte buffer)
{
 byte colour, channel, multiplier;
 byte incColour = 0;
 for (channel = 0; channel < ledStrips; channel++)
 {
  for (colour = 0; colour < ledColours; colour++)
  {
   incColour++;
   for (multiplier = 0; multiplier < incColour; multiplier++)
   {
    colourBuffer[buffer][colour][channel] = incDecWord(colourBuffer[buffer][colour][channel]);
   }
  }
 }
}

void cycleColours2(byte buffer)
{
 byte colour, channel, multiplier;
 for (channel = 0; channel < ledStrips; channel++)
 {
  for (colour = 0; colour < ledColours; colour++)
  {
   colourBuffer[buffer][colour][channel] = incDecWord(colourBuffer[buffer][colour][channel]);
  }
 }
}

void fadeColours(byte buffer)
{
 byte colour, channel;
 int increment;
 for (colour = 0; colour < ledColours; colour++)
 {
  for (channel = 0; channel < ledStrips; channel++)
  {
   if (colour == (channel % 3))
   {
    colourBuffer[buffer][colour][channel] = incDecWord(colourBuffer[buffer][colour][channel]);
   }
  }
 }
}

void clearDisplay()
{
 blockColour(backBuffer, 0x0000, 0x0000, 0x0000);
 swapBuffers();
 blockColour(backBuffer, 0x0000, 0x0000, 0x0000);
}

void clearColours()
{
 blockColour(backBuffer,  0x0000, 0x0000, 0x0000);
 blockColour(frontBuffer, 0x0000, 0x0000, 0x0000);
}

void flashError()
{
 flashColour(0xFF00, 0x0000, 0x0000);
}

void flashWarning()
{
 flashColour(0xFF00, 0xFF00, 0x0000);
}

void flashOK()
{
 flashColour(0x0000, 0xFF00, 0x0000);
}

void flashColour(word newRed, word newGreen, word newBlue)
{
 blockColour(backBuffer, newRed, newGreen, newBlue);
 swapBuffers();
 clearDisplay();
}

void blockColour(byte buffer, word newRed, word newGreen, word newBlue)
{
 byte colour, channel;
 for (channel = 0; channel < ledChannels; channel++)
 {
  colourBuffer[buffer][RedIndex][channel]   = newRed;
  colourBuffer[buffer][GreenIndex][channel] = newGreen;
  colourBuffer[buffer][BlueIndex][channel]  = newBlue;
 }
}

/********************************************************
    LED Functions
********************************************************/

/* Timer */

void setupTimer2(void)                    // Initialize Timer2
{
 TCCR2A |=  (1 << WGM21)  | (1 << WGM20);
 TCCR2B |=  (1 << CS22);                   // by clk/64
 TCCR2B &=~ ((1 << CS21)  | (1 << CS20));  // by clk/64
 TCCR2B &=~ ((1 << WGM21) | (1 << WGM20)); // Use normal mode
 ASSR   |=  (0 << AS2);                    // Use internal clock - external clock not used in Arduino
 TIMSK2 |=  (1 << TOIE2)  | (0 << OCIE2B); // Timer2 Overflow Interrupt Enable
 //TCNT2   = staticGamma;
 sei(); // Start the time
}

void initTimer2(byte currentGamma)
{
 TCNT2 = currentGamma; // Set the timing to the gamma value for the first PWM level
 sei(); // Start the timer
}

ISR(TIMER2_OVF_vect)  // Timer2 Service
{
 //TCNT2 = gamma[level];        // Set the flashing time using gamma value table
 TCNT2 = staticGamma;                  // Set the flashing time to a static value
 flash_next_level(level);       // flash the LEDs for the current level.
 if (level < outputColours)
 {
  level++;
 }
 else // After flashing all levels, go back to level 0 and swap the buffers
 {
  level = 0;
  currentBuffer = frontBuffer;  // do the actual swapping, synced with display refresh.
 }
}

/* Buffer Handling */

void swapBuffers() // Swap Front with Back buffer
{
 frontBuffer = !frontBuffer;
 backBuffer = !backBuffer;
 while(currentBuffer != frontBuffer)  // Wait for display to change.
 {
  delayMicroseconds(10);
 }
}

void copyBuffer(byte buffer)
{
 byte colour, channel;
 for (colour = 0; colour < ledColours; colour++)
 {
  for (channel = 0; channel < ledStrips; channel++)
  {
   colourBuffer[!buffer][colour][channel] = colourBuffer[buffer][colour][channel];
  }
 }
}

void printBuffer(byte buffer)
{
 byte colour, channel;
 for (colour = 0; colour < ledColours; colour++)
 {
  //for (channel = 0; channel < ledStrips; channel++)
  for (channel = 0; channel < ledChannels; channel++)
  {
   Serial.print(colourBuffer[buffer][colour][channel], HEX);
   Serial.write(" ");
  }
  Serial.println("");
  delay(5);
 }
 Serial.println("");
}

/* LED Lighting */

void flash_next_level(byte lvl)  // Scan one line
{
 //open_lines(ledStrips);
 disable_oe;
 shift_24_bit(lvl);
 enable_oe;
 //open_lines(0);
}

/*
void shift_24_bit(byte lvl) // Translate from the stored RGB values to our required order for the rainbowduino hardware of RBG
{
 le_high; // Set the latch to high, so that we can send our serial data to be parallelised
 byte colour, channel;
 for(channel = 0; channel < ledChannels; channel++) // Loop for each channel
 {
  shift_1_bit((colourBuffer[currentBuffer][RedIndex][channel]   >> colourOffset) > lvl); // Flash the LED on if it needs flashing for this gamma level
  shift_1_bit((colourBuffer[currentBuffer][GreenIndex][channel] >> colourOffset) > lvl); // Flash the LED on if it needs flashing for this gamma level
  shift_1_bit((colourBuffer[currentBuffer][BlueIndex][channel]  >> colourOffset) > lvl); // Flash the LED on if it needs flashing for this gamma level
 }
 le_low;
}
*/

void shift_24_bit(byte lvl)
{
 le_high; // Set the latch to high, so that we can send our serial data to be parallelised
 byte colour, channel;
 for(colour = 0; colour < ledColours; colour++) // Loop for each colour
 {
  for(channel = 0; channel < ledStrips; channel++) // Loop for each channel that has has a LED strip attached
  {
   shift_1_bit((colourBuffer[currentBuffer][colour][channel] >> colourOffset) > lvl); // Flash the LED on if it needs flashing for this gamma level
  }
  for(channel; channel < ledChannels; channel++) //For the channels without a LED strip...
  {
   shift_data_0; // ...don't send anything (Quicker than having to do the calculation for the bit shift in the for loop above)
  }
 }
 le_low;
}

static void shift_1_bit(byte ls)
{
 if (ls) { shift_data_1; }
 else    { shift_data_0; }
 clk_rising;
}

static void open_lines(byte maxline)
{
 switch (maxline)
 {
  case 0: { PORTB = 0x00; PORTD = 0x00; break; }
  case 1: { PORTB = 0x04; PORTD = 0x00; break; }
  case 2: { PORTB = 0x06; PORTD = 0x00; break; }
  case 3: { PORTB = 0x07; PORTD = 0x00; break; }
  case 4: { PORTB = 0x07; PORTD = 0x80; break; }
  case 5: { PORTB = 0x07; PORTD = 0xC0; break; }
  case 6: { PORTB = 0x07; PORTD = 0xE0; break; }
  case 7: { PORTB = 0x07; PORTD = 0xF0; break; }
  case 8: { PORTB = 0x07; PORTD = 0xF8; break; }
 }
}

static void open_line(byte openline)
{
 switch (openline)
 {
  case 0: { PORTB = 0x07; PORTD = 0x00; break; }
  case 1: { PORTB = 0x04; PORTD = 0x00; break; }
  case 2: { PORTB = 0x02; PORTD = 0x00; break; }
  case 3: { PORTB = 0x01; PORTD = 0xf8; break; }
  case 4: { PORTB = 0x00; PORTD = 0x80; break; }
  case 5: { PORTB = 0x00; PORTD = 0xC0; break; }
  case 6: { PORTB = 0x00; PORTD = 0xE0; break; }
  case 7: { PORTB = 0x00; PORTD = 0xF0; break; }
  case 8: { PORTB = 0x00; PORTD = 0xF8; break; }
 }
}

/******************************************************************
    Unused Functions
******************************************************************/

void flashLED(byte flashes, int period)
{
 for (byte i = 0; i < flashes; i++)
 {
  digitalWrite(diagLED, HIGH);
  delay(period);
  digitalWrite(diagLED, LOW);
  delay(period);
 }
}

/*
int toInt(byte b)
{
 return map(b, 0, 255, -128, 127);
}
*/

/*
static void open_line(byte line)
{
 switch (line)
 {
  case 0: { PORTB=0x04; break; }
  case 1: { PORTB=0x02; break; }
  case 2: { PORTB=0x01; break; }
  case 3: { PORTD=0x80; break; }
  case 4: { PORTD=0x40; break; }
  case 5: { PORTD=0x20; break; }
  case 6: { PORTD=0x10; break; }
  case 7: { PORTD=0x08; break; }
 }
}
*/

/*
static void open_line(byte ln) // Open the line and close others
{
 if(ln < 3)
 {
  PORTB  = (PINB & ~0x07) | 0x04 >> ln;
  PORTD  = (PIND & ~0xF8);
 }
 else
 {
  PORTB  = (PINB & ~0x07);
  PORTD  = (PIND & ~0xF8) | 0x80 >> (ln - 3);
 }
}
*/
