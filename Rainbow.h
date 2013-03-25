#ifndef Rainbow_h
#define Rainbow_h

//=============================================
//MBI5168
#define SH_DIR_SDI         DDRC
#define SH_DIR_CLK         DDRC
#define SH_DIR_LE          DDRC
#define SH_DIR_OE          DDRC

//pin 23 of the arduino maps to first MBI5169 (blue) SDI input
#define SH_BIT_SDI         0x01
//pin 24 of the arduino maps to the MBI5169 CLK input
#define SH_BIT_CLK         0x02
//pin 25 of the arduino maps to the MBI5169 LE input
#define SH_BIT_LE          0x04
//pin 26 of the arduino maps to the MBI5169 OE input
#define SH_BIT_OE          0x08

#define SH_PORT_SDI        PORTC
#define SH_PORT_CLK        PORTC
#define SH_PORT_LE         PORTC
#define SH_PORT_OE         PORTC

//============================================
//Setting a bit: byte |= 1 << bit;
//Clearing a bit: byte &= ~(1 << bit);
//Toggling a bit: byte ^= 1 << bit;
//Checking if a bit is set: if (byte & (1 << bit))
//Checking if a bit is cleared: if (~byte & (1 << bit)) OR if (!(byte & (1 << bit)))

//Clock input terminal for data shift on rising edge
#define clk_rising         {SH_PORT_CLK &=~ SH_BIT_CLK; SH_PORT_CLK |= SH_BIT_CLK;}
//Data strobe input terminal, Serial data is transfered to the respective latch when LE is high.
#define le_high            {SH_PORT_LE |= SH_BIT_LE;}
//The data is latched when LE goes low.
#define le_low             {SH_PORT_LE &=~ SH_BIT_LE;}
//Output Enabled, when (active) low, the output drivers are enabled;
#define enable_oe          {SH_PORT_OE &=~ SH_BIT_OE;}
//when high, all output drivers are turned OFF (blanked).
#define disable_oe         {SH_PORT_OE |= SH_BIT_OE;}

#define shift_data_1       {SH_PORT_SDI |=  SH_BIT_SDI;}
#define shift_data_0       {SH_PORT_SDI &=~ SH_BIT_SDI;}

//============================================
#define close_all_lines	   {PORTD &=~ 0xf8; PORTB &=~ 0x07;}
#define open_all_lines	   {PORTD |=  0xf8; PORTB |=  0x07;}

#define RedIndex   0
#define BlueIndex  1
#define GreenIndex 2

#endif

