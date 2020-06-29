/*****************************************************************/
/*                VFO for QRP SSB Transceiver 20m                */
/*             "Micro26" (Rebuild of Micro 20  -1)               */
/*                 w. ATMega328p and Si5351                      */
/*                    Display OLED SSD1306                       */
/*  ************************************************************ */
/*  MUC:              ATMEL AVR ATmega328p, 16 MHz               */
/*                                                               */
/*  Compiler:         GCC (GNU AVR C-Compiler)                   */
/*  Author:           Peter Rachow (DK7IH)                       */
/*  Last change:      2020-06-18                                 */
/*****************************************************************/

//PORTS
//OUTPUT
//PB1: AGC
//PB2: Tone control

//INPUT
//PC0: ADC0: Pull-up for key switches with various resistors against GND 
//PC1: ADC1: AGC voltage for S-Meter
//PC2: ADC2: Voltage measurement
//PC3: ADC3: TX PWR meter
//     ADC6: PA Temp.
//PD0: TX/RX indicator
//PD5, PD6: Rotary encoder 


//OUTPUT
//PB1 AGC fast/slow
//PB2 Tone hi/lo
//TWI
//PC4=SDA (white), PC5=SCL (violet): I²C-Bus lines: 

//  EEPROM  //////////////////////////////////////////////////
// Bytes
// 0:7: QRG VFOA and VFOB
// 8: Last VFO in use
//9: TONE
//10 AGC
//11 Last memory used
//12 Scan threshold
//16:272: 16 MEM frequencies for 16 VFOs

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/pgmspace.h>

  //////////////////////////////////////////////
 // Radio general Defines and Declarations   //
//////////////////////////////////////////////
#define MAXVFO 1
#define MENUITEMS {4, 1, 3, 2, 1, 1}

//Bands, VFOs etc.
int cur_band = 0;
int cur_vfo = 0;
int sideband = 0;  //0=USB, 1=LSB

//Tuning
int tuningcount = 0;
int tuningknob = 0;
int laststate = 0; //Last state of rotary encoder

//Seconds counting
long runseconds10 =  0;

//METER
int sv_old = 0;
int smax = 0;
long runseconds10s = 0;

//Interfrequency options
#define IFOPTION 0

#if (IFOPTION == 0) //9MHz Filter 9XMF24D (box73.de)
    #define INTERFREQUENCY 9000000
    #define F_LO_LSB 8998600
    #define F_LO_USB 9001700
#endif  
  
#if (IFOPTION == 1)  //10.695MHz Filter 10M04DS (ex CB TRX "President Jackson")
    #define INTERFREQUENCY 10695000 //fLSB 10692100, fUSB 10697700
    #define F_LO_LSB 10691050
    #define F_LO_USB 10697770
#endif  

#if (IFOPTION == 2) //10.7MHz Filter 10MXF24D (box73.de)
    #define INTERFREQUENCY 10700000 
    #define F_LO_LSB 10697800
    #define F_LO_USB 10702400
#endif  

#if (IFOPTION == 3) //Ladderfilter 9.830 MHz low profile xtals
    #define INTERFREQUENCY 9830000
    #define F_LO_LSB 9828320
    #define F_LO_USB 9831250
#endif  

#if (IFOPTION == 4) //Ladderfilter 9.832 MHz high profile xtals "NARVA"
    #define INTERFREQUENCY 9830000
    #define F_LO_LSB 9831000
    #define F_LO_USB 9835300
#endif  

#if (IFOPTION == 5)     //Ladderfilter 10 MHz high profile xtals
    #define INTERFREQUENCY 10000000
    #define F_LO_LSB 9994720
    #define F_LO_USB 9999840
#endif  

//Frequencies
long f_lo[] = {F_LO_USB, F_LO_LSB};  //Stores frequencies for USB and LSB
long f_vfo[MAXVFO + 1]; //Stores freqencies of 2 VFOs

  //////////////////////////////////////////////
 //   L   C   D   Defines and Declarations   //
//////////////////////////////////////////////
#define OLEDCMD 0x00   //Command follows
#define OLEDDATA 0x40  //Data follows
#define OLEDADDR 0x78  //address for the chip - usually 0x7C or 0x78. 

#define FONTW 6
#define FONTH 8

#define S_SETLOWCOLUMN           0x00
#define S_SETHIGHCOLUMN          0x10
#define S_PAGEADDR               0xB0
#define S_SEGREMAP               0xA0

#define S_LCDWIDTH               128
#define S_LCDHEIGHT              64 

// Font 6x8 for OLED
#define FONTWIDTH 6
const char font[][6] PROGMEM={
{0x00,0x00,0x00,0x00,0x00,0x00},	// 0x20
{0x00,0x00,0x06,0x5F,0x06,0x00},	// 0x21
{0x00,0x07,0x03,0x00,0x07,0x03},	// 0x22
{0x00,0x24,0x7E,0x24,0x7E,0x24},	// 0x23
{0x00,0x24,0x2B,0x6A,0x12,0x00},	// 0x24
{0x00,0x63,0x13,0x08,0x64,0x63},	// 0x25
{0x00,0x36,0x49,0x56,0x20,0x50},	// 0x26
{0x00,0x00,0x07,0x03,0x00,0x00},	// 0x27
{0x00,0x00,0x3E,0x41,0x00,0x00},	// 0x28
{0x00,0x00,0x41,0x3E,0x00,0x00},	// 0x29
{0x00,0x08,0x3E,0x1C,0x3E,0x08},	// 0x2A
{0x00,0x08,0x08,0x3E,0x08,0x08},	// 0x2B
{0x00,0x00,0xE0,0x60,0x00,0x00},	// 0x2C
{0x00,0x08,0x08,0x08,0x08,0x08},	// 0x2D
{0x00,0x00,0x60,0x60,0x00,0x00},	// 0x2E
{0x00,0x20,0x10,0x08,0x04,0x02},	// 0x2F
{0x00,0x3E,0x51,0x49,0x45,0x3E},	// 0x30
{0x00,0x00,0x42,0x7F,0x40,0x00},	// 0x31
{0x00,0x62,0x51,0x49,0x49,0x46},	// 0x32
{0x00,0x22,0x49,0x49,0x49,0x36},	// 0x33
{0x00,0x18,0x14,0x12,0x7F,0x10},	// 0x34
{0x00,0x2F,0x49,0x49,0x49,0x31},	// 0x35
{0x00,0x3C,0x4A,0x49,0x49,0x30},	// 0x36
{0x00,0x01,0x71,0x09,0x05,0x03},	// 0x37
{0x00,0x36,0x49,0x49,0x49,0x36},	// 0x38
{0x00,0x06,0x49,0x49,0x29,0x1E},	// 0x39
{0x00,0x00,0x6C,0x6C,0x00,0x00},	// 0x3A
{0x00,0x00,0xEC,0x6C,0x00,0x00},	// 0x3B
{0x00,0x08,0x14,0x22,0x41,0x00},	// 0x3C
{0x00,0x24,0x24,0x24,0x24,0x24},	// 0x3D
{0x00,0x00,0x41,0x22,0x14,0x08},	// 0x3E
{0x00,0x02,0x01,0x59,0x09,0x06},	// 0x3F
{0x00,0x3E,0x41,0x5D,0x55,0x1E},	// 0x40
{0x00,0x7E,0x11,0x11,0x11,0x7E},	// 0x41
{0x00,0x7F,0x49,0x49,0x49,0x36},	// 0x42
{0x00,0x3E,0x41,0x41,0x41,0x22},	// 0x43
{0x00,0x7F,0x41,0x41,0x41,0x3E},	// 0x44
{0x00,0x7F,0x49,0x49,0x49,0x41},	// 0x45
{0x00,0x7F,0x09,0x09,0x09,0x01},	// 0x46
{0x00,0x3E,0x41,0x49,0x49,0x7A},	// 0x47
{0x00,0x7F,0x08,0x08,0x08,0x7F},	// 0x48
{0x00,0x00,0x41,0x7F,0x41,0x00},	// 0x49
{0x00,0x30,0x40,0x40,0x40,0x3F},	// 0x4A
{0x00,0x7F,0x08,0x14,0x22,0x41},	// 0x4B
{0x00,0x7F,0x40,0x40,0x40,0x40},	// 0x4C
{0x00,0x7F,0x02,0x04,0x02,0x7F},	// 0x4D
{0x00,0x7F,0x02,0x04,0x08,0x7F},	// 0x4E
{0x00,0x3E,0x41,0x41,0x41,0x3E},	// 0x4F
{0x00,0x7F,0x09,0x09,0x09,0x06},	// 0x50
{0x00,0x3E,0x41,0x51,0x21,0x5E},	// 0x51
{0x00,0x7F,0x09,0x09,0x19,0x66},	// 0x52
{0x00,0x26,0x49,0x49,0x49,0x32},	// 0x53
{0x00,0x01,0x01,0x7F,0x01,0x01},	// 0x54
{0x00,0x3F,0x40,0x40,0x40,0x3F},	// 0x55
{0x00,0x1F,0x20,0x40,0x20,0x1F},	// 0x56
{0x00,0x3F,0x40,0x3C,0x40,0x3F},	// 0x57
{0x00,0x63,0x14,0x08,0x14,0x63},	// 0x58
{0x00,0x07,0x08,0x70,0x08,0x07},	// 0x59
{0x00,0x71,0x49,0x45,0x43,0x00},	// 0x5A
{0x00,0x00,0x7F,0x41,0x41,0x00},	// 0x5B
{0x00,0x02,0x04,0x08,0x10,0x20},	// 0x5C
{0x00,0x00,0x41,0x41,0x7F,0x00},	// 0x5D
{0x00,0x04,0x02,0x01,0x02,0x04},	// 0x5E
{0x80,0x80,0x80,0x80,0x80,0x80},	// 0x5F
{0x00,0x00,0x03,0x07,0x00,0x00},	// 0x60
{0x00,0x20,0x54,0x54,0x54,0x78},	// 0x61
{0x00,0x7F,0x44,0x44,0x44,0x38},	// 0x62
{0x00,0x38,0x44,0x44,0x44,0x28},	// 0x63
{0x00,0x38,0x44,0x44,0x44,0x7F},	// 0x64
{0x00,0x38,0x54,0x54,0x54,0x08},	// 0x65
{0x00,0x08,0x7E,0x09,0x09,0x00},	// 0x66
{0x00,0x18,0xA4,0xA4,0xA4,0x7C},	// 0x67
{0x00,0x7F,0x04,0x04,0x78,0x00},	// 0x68
{0x00,0x00,0x00,0x7D,0x40,0x00},	// 0x69
{0x00,0x40,0x80,0x84,0x7D,0x00},	// 0x6A
{0x00,0x7F,0x10,0x28,0x44,0x00},	// 0x6B
{0x00,0x00,0x00,0x7F,0x40,0x00},	// 0x6C
{0x00,0x7C,0x04,0x18,0x04,0x78},	// 0x6D
{0x00,0x7C,0x04,0x04,0x78,0x00},	// 0x6E
{0x00,0x38,0x44,0x44,0x44,0x38},	// 0x6F
{0x00,0xFC,0x44,0x44,0x44,0x38},	// 0x70
{0x00,0x38,0x44,0x44,0x44,0xFC},	// 0x71
{0x00,0x44,0x78,0x44,0x04,0x08},	// 0x72
{0x00,0x08,0x54,0x54,0x54,0x20},	// 0x73
{0x00,0x04,0x3E,0x44,0x24,0x00},	// 0x74
{0x00,0x3C,0x40,0x20,0x7C,0x00},	// 0x75
{0x00,0x1C,0x20,0x40,0x20,0x1C},	// 0x76
{0x00,0x3C,0x60,0x30,0x60,0x3C},	// 0x77
{0x00,0x6C,0x10,0x10,0x6C,0x00},	// 0x78
{0x00,0x9C,0xA0,0x60,0x3C,0x00},	// 0x79
{0x00,0x64,0x54,0x54,0x4C,0x00},	// 0x7A
{0x00,0x08,0x3E,0x41,0x41,0x00},	// 0x7B
{0x00,0x00,0x00,0x77,0x00,0x00},	// 0x7C
{0x00,0x00,0x41,0x41,0x3E,0x08},	// 0x7D
{0x00,0x02,0x01,0x02,0x01,0x00},	// 0x7E
{0x00,0x3C,0x26,0x23,0x26,0x3C},	// 0x7F
{0x00,0x1E,0xA1,0xE1,0x21,0x12},    // 0x80
{0x00,0x00,0xF8,0x08,0x08,0x00},    // 81 left top
{0x08,0x08,0xF8,0x00,0x00,0x00},	// 82 right top
{0x00,0x00,0x0F,0x08,0x08,0x00},	// 83 left bottom
{0x08,0x08,0x0F,0x00,0x00,0x00},	// 84 right bottom
{0x00,0x00,0xFF,0x00,0x00,0x00},   // 85 |
{0x08,0x08,0x08,0x08,0x08,0x00},   // 86 -
{0x00,0x06,0x09,0x09,0x06,0x00}    //87 °
};	

  ///////////////////////////
 //  OLED DECLARATIONS    // 
///////////////////////////
//I²C
void twi_init(void);
void twi_sart(void);
void twi_stop(void);

//OLED
void oled_command(int value);
void oled_data(unsigned int*, unsigned int);
void oled_gotoxy(unsigned int, unsigned int);
void oled_cls(int);
void oled_init(void);
void oled_byte(unsigned char);
void oled_putchar1(unsigned int x, unsigned int y, unsigned char ch, int);
void oled_putchar2(unsigned int x, unsigned int y, unsigned char ch, int);
void oled_putnumber(int, int, long, int, int, int);
void oled_putstring(int, int, char*, char, int);
void oled_write_section(int, int, int, int);
void oled_drawbox(int, int, int, int);
void draw_meter_scale(int);

//String
int int2asc(long num, int dec, char *buf, int buflen);
int strlen(char *s);

//Data display functions
void show_frequency(long, int);
void show_vfo(int, int);
void show_sideband(int, int);
void show_txrx(int);
void show_voltage(int);
void show_temp(int);
void show_tone(int, int);
void show_agc(int, int);
void show_split(int);
void show_mem_num(int, int);
void show_meter(int);
void reset_smax(void);

//MISC
int main(void);

//Keys, TX & ADC
int get_adc(int);
int get_keys(void);
int get_s_value(void);
int get_temp(void);
int get_tx_pwr_value(void);
int get_txrx(void);

//Menu
long menux(long, int);
void print_menu_head(char*, int);
void print_menu_item(char*, int, int);
void print_menu_item_list(int, int, int);
int navigate_thru_item_list(int, int);

//Tone
void set_tone(int);

//AGC
void set_agc(int);

//EEPROM
long load_frequency(int, int);
int load_last_vfo(void);
void store_last_mem(int);
int recall_last_mem(void);
void store_frequency(long, int, int);
void store_last_vfo(int);
int is_mem_freq_ok(long);
void store_tone(int);
void store_agc(int);
int recall_tone(void);
int recall_agc(void);
void store_scan_thresh(int);
int recall_scan_thresh(void);

//Memories
void show_mem_menu_item(int, int);
int mem_select(int, int);

//SCAN
int scan_memories(int);
long scan_vfo(int);
int set_scan_threshold(int);

//Setting frequencies
int calc_tuningfactor(void);
void set_vfo_frequency(long);
void set_lo_frequency(long);

void adj_lo_frequency(int);

  ////////////////////////////////////////////
 //   Si5351   Defines and Declarations    //
////////////////////////////////////////////
#define SI5351_ADDRESS 0xC0 // 0b11000000 for my module. Others may vary! The 0x60 did NOT work with my module!
#define PLLRATIO 36
#define CFACTOR 1048575

//Set of Si5351A register addresses
#define CLK_ENABLE_CONTROL       3
#define PLLX_SRC				15
#define CLK0_CONTROL            16 
#define CLK1_CONTROL            17
#define CLK2_CONTROL            18
#define SYNTH_PLL_A             26
#define SYNTH_PLL_B             34
#define SYNTH_MS_0              42
#define SYNTH_MS_1              50
#define SYNTH_MS_2              58
#define PLL_RESET              177
#define XTAL_LOAD_CAP          183

#undef F_CPU
#define F_CPU 8000000

void si5351_write(int, int);
void si5351_start(void);
void si5351_set_freq(int, unsigned long);

  /////////////////////////////
 //   Misc. Declarations    //
/////////////////////////////
char *oldbuf = "         ";


  ///////////////////////////
 //         ISRs          // 
///////////////////////////
ISR(TIMER1_COMPA_vect)
{
    runseconds10++; 
    tuningcount = 0;
}

//Rotary encoder
ISR(PCINT2_vect)
{ 
	int gray = (PIND & 0x60) >> 5;           // Read PD5 and PD6
    	
    int state = (gray >> 1) ^ gray;         // Convert from Gray code to binary

    if (state != laststate)                //Compare states
    {        
        tuningknob += ((laststate - state) & 0x03) - 2; // Results in -1 or +1
        laststate = state;
        tuningcount += 2;
    } 
	PCIFR |=  (1 << PCIF0); // Clear pin change interrupt flag.
}


///////////////////////////
//
//         TWI
//
///////////////////////////

void twi_init(void)
{
    //set SCL to 400kHz
    TWSR = 0x00;
    TWBR = 0x0C;
	
    //enable TWI
    TWCR = (1<<TWEN);
}

//Send start signal
void twi_start(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

//send stop signal
void twi_stop(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void twi_write(uint8_t u8data)
{
    TWDR = u8data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

////////////////////////////////
//
// OLED routines
//
///////////////////////////////

//Send comand to OLED
void oled_command(int value)
{
   twi_start();
   twi_write(OLEDADDR); //Device address
   twi_write(OLEDCMD);  //Command follows
   twi_write(value);    //
   twi_stop();
} 

//Send a 'number' bytes of data to display - from RAM
void oled_data(unsigned int *data, unsigned int number)
{
   int t1;
   twi_start();
   twi_write(OLEDADDR); //Device address
   twi_write(OLEDDATA); //Data follows
   
   for(t1 = 0; t1 < number; t1++)
   {
      twi_write(data[t1]); //send the byte(s)
   }   
   twi_stop ();   
} 

//Set "cursor" to current position to screen
void oled_gotoxy(unsigned int x, unsigned int y)
{
   int x2 = x; // + 2;
   twi_start();
   twi_write(OLEDADDR); //Select display  I2C address
   twi_write(OLEDCMD);  //Be ready for command
   twi_write(S_PAGEADDR + y); //Select display row
   twi_write(S_SETLOWCOLUMN + (x2 & 0x0F)); //Col addr lo byte
   twi_write(S_SETHIGHCOLUMN + ((x2 >> 4) & 0x0F)); //Col addr hi byte
   twi_stop();
}

void oled_cls(int invert)
{
    unsigned int row, col;

    //Just fill the memory with zeros
    for(row = 0; row < S_LCDHEIGHT / 8; row++)
    {
        oled_gotoxy(0, row); //Set OLED address
        twi_start();
        twi_write(OLEDADDR); //Select OLED
        twi_write(OLEDDATA); //Data follows
        for(col = 0; col < S_LCDWIDTH; col++)
        {
            if(!invert)
            {
                twi_write (0); //normal
            }   
            else
            {
                twi_write(255); //inverse
            } 
        }
        twi_stop();
    }
    oled_gotoxy(0, 0); //Return to 0, 0
}

//Write number of bitmaps to one row of screen
void oled_write_section(int x1, int x2, int row, int number)
{
    int t1;
    oled_gotoxy(x1, row);
    	
    twi_start();
    twi_write(OLEDADDR); //Device address
    twi_write(OLEDDATA); //Data follows
   
    for(t1 = x1; t1 < x2; t1++)
    {
       twi_write(number); //send the byte(s)
    }    
    twi_stop ();   
}


//Initialize OLED
void oled_init(void)
{
    oled_command(0xAE); // Display OFF
	oled_command(0x20); // Set Memory Addressing Mode
    oled_command(0x00); // HOR
    
    oled_command(0xB0);    // Set Page Start Address for Page Addressing Mode, 0-7
    oled_command(0xC8);    // Set COM Output Scan Direction
    oled_command(0x00);    // --set low column address
    oled_command(0x10);    // --set high column address
    oled_command(0x40);    // --set start line address
    oled_command(0x81);
    oled_command(0xFF);    // Set contrast control register
    oled_command(0xA1);    // Set Segment Re-map. A0=address mapped; A1=address 127 mapped.
    oled_command(0xA6);    // Set display mode. A6=Normal; A7=Inverse
    oled_command(0xA8);
    oled_command(0x3F);    // Set multiplex ratio(1 to 64)
    oled_command(0xA4);    // Output RAM to Display
					       // 0xA4=Output follows RAM content; 0xA5,Output ignores RAM content
    oled_command(0xD3);
    oled_command(0x00);    // Set display offset. 00 = no offset
    oled_command(0xD5);    // --set display clock divide ratio/oscillator frequency
    oled_command(0xF0);    // --set divide ratio
    oled_command(0xD9); 
    oled_command(0x22);    // Set pre-charge period
    oled_command(0xDA);
    oled_command(0x12);    // Set com pins hardware configuration
    oled_command(0xDB);    // --set vcomh
    oled_command(0x20);    // 0x20,0.77xVcc
    oled_command(0x8D);
    oled_command(0x14);    // Set DC-DC enabl
    oled_command(0xAF);    //Display ON
   
} 

//Write 1 byte pattern to screen using vertical orientation 
void oled_byte(unsigned char value)
{
   twi_start();
   twi_write(OLEDADDR); //Device address
   twi_write(OLEDDATA); //Data follows
   twi_write(value);
   twi_stop ();   
}

//Write character to screen (normal size);
void oled_putchar1(unsigned int x, unsigned int y, unsigned char ch, int invert)
{
	int t0;
		
	oled_gotoxy(x, y);
	for(t0 = 0; t0 < FONTW; t0++)
	{
		if(!invert)
		{
            oled_byte(pgm_read_byte(&font[ch - 32][t0]));
        }
        else    
        {
            oled_byte(~pgm_read_byte(&font[ch - 32][t0]));
        }
        
	}
}		

//Write character to screen (DOUBLE size);
void oled_putchar2(unsigned int x, unsigned int y, unsigned char ch, int invert)
{
	int t0, t1;
	char c;
	int i[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
	for(t0 = 0; t0 < FONTW; t0++)
	{
		for(t1 = 0; t1 < 8; t1++)
		{
			if(!invert)
			{
				c = pgm_read_byte(&font[ch - 32][t0]);
			}
			else
			{
				c = ~pgm_read_byte(&font[ch - 32][t0]);
			}
				
		    if(c & (1 << t1))
		    {
			    i[t0] += (1 << (t1 * 2));
			    i[t0] += (1 << (t1 * 2 + 1));
		    }	
	    }
	}
	
	oled_gotoxy(x, y);
	for(t0 = 0; t0 < FONTW; t0++)
	{		
	    oled_byte(i[t0] & 0xFF);
	    oled_byte(i[t0] & 0xFF);
	}
	
	oled_gotoxy(x, y + 1);
	for(t0 = 0; t0 < FONTW; t0++)
	{		
	    oled_byte((i[t0] & 0xFF00) >> 8);
	    oled_byte((i[t0] & 0xFF00) >> 8);
	}
}		

//Print string in given size
//lsize=0 => normal height, lsize=1 => double height
void oled_putstring(int col, int row, char *s, char lsize, int inv)
{
    int c = col;
	
	while(*s)
	{
	    if(!lsize)
		{
	        oled_putchar1(c, row, *s++, inv);
		}
        else
        {
            oled_putchar2(c, row, *s++, inv);
		}	
		c += (lsize + 1) * FONTW;
	}
}

//Print an integer/long to OLED
void oled_putnumber(int col, int row, long num, int dec, int lsize, int inv)
{
    char *s = malloc(16);
	if(s != NULL)
	{
	    int2asc(num, dec, s, 16);
	    oled_putstring(col, row, s, lsize, inv);
	    free(s);
	}	
}


void oled_drawbox(int x0, int y0, int x1, int y1)
{
	int t1;
	
	for(t1 = x0 + 5; t1 < x1; t1 += 5)
	{
		oled_putchar1(t1, y0, 0x86, 0);
		oled_putchar1(t1, y1, 0x86, 0);
	}
	
	for(t1 = y0 + 1; t1 < y1; t1++)
	{
		oled_putchar1(x0, t1, 0x85, 0);
		oled_putchar1(x1, t1, 0x85, 0);
	}
	
	oled_putchar1(x0, y0, 0x81, 0); //Left top corner
	oled_putchar1(x1, y0, 0x82, 0); //Right top corner
	oled_putchar1(x0, y1, 0x83, 0); //Left bottom corner
	oled_putchar1(x1, y1, 0x84, 0); //Right bottom corner
}


void draw_meter_scale(int meter_type)
{
    if(!meter_type)
    {
        oled_putstring(0, 7, "S1 3 5 7 9 +10 +20dB", 0, 0);
    }
    else
    {
        oled_putstring(0, 7, "0 1W  2W  3W  4W  5W", 0, 0);
    }
}

 
/////////////////////////////////
//
// STRING FUNCTIONS
//
////////////////////////////////
//INT 2 ASC
int int2asc(long num, int dec, char *buf, int buflen)
{
    int i, c, xp = 0, neg = 0;
    long n, dd = 1E09;

    if(!num)
	{
	    *buf++ = '0';
		*buf = 0;
		return 1;
	}	
		
    if(num < 0)
    {
     	neg = 1;
	    n = num * -1;
    }
    else
    {
	    n = num;
    }

    //Fill buffer with \0
    for(i = 0; i < 12; i++)
    {
	    *(buf + i) = 0;
    }

    c = 9; //Max. number of displayable digits
    while(dd)
    {
	    i = n / dd;
	    n = n - i * dd;
	
	    *(buf + 9 - c + xp) = i + 48;
	    dd /= 10;
	    if(c == dec && dec)
	    {
	        *(buf + 9 - c + ++xp) = '.';
	    }
	    c--;
    }

    //Search for 1st char different from '0'
    i = 0;
    while(*(buf + i) == 48)
    {
	    *(buf + i++) = 32;
    }

    //Add minus-sign if neccessary
    if(neg)
    {
	    *(buf + --i) = '-';
    }

    //Eleminate leading spaces
    c = 0;
    while(*(buf + i))
    {
	    *(buf + c++) = *(buf + i++);
    }
    *(buf + c) = 0;
	
	return c;
}

//STRLEN
int strlen(char *s) 
{
   int t1 = 0;

   while(*(s + t1++));

   return (t1 - 1);
}

    ////////////////////////////
   //                        // 
  // Data display functions //      
 //                        //
////////////////////////////
//Current frequency (double letter height)
void show_frequency(long f, int refresh)
{
	char *buf;
	int t1 = 0;
	int ypos = 4;
	
	buf = malloc(10);
	
	//Init buffer string
	for(t1 = 0; t1 < 10; t1++)
	{
	    *(buf + t1) = 0;
	}
	
	if(f)
	{
	    int2asc(f / 10, 2, buf, 9);
	}
	else
	{
		for(t1 = 0; t1 < 8; t1++)
		{
			buf[t1] = 32;
		}
	}		
	
	//Display buffer (but only the letters that have changed)
	for(t1 = 0; *(buf + t1); t1++)
	{
		if((*(oldbuf + t1) != *(buf + t1)) || refresh)
		{
		    oled_putchar2(15 + t1 * 12, ypos, *(buf + t1), 0);
		}   
	}	
	
	//Copy current buffer to storing variable
	t1 = 0;
	while(*(buf + t1))
	{
		*(oldbuf + t1) = *(buf + t1);
		t1++;
	}	
	
	free(buf);

}

//VFO
void show_vfo(int nvfo, int invert)
{
	int xpos = 0, ypos = 0;
	
	oled_putstring(xpos * FONTWIDTH, ypos, "VFO:", 0, invert);			
	oled_putchar1((xpos + 4) * 6, ypos, nvfo + 65, invert);  
	
}

//Sideband
void show_sideband(int sb, int invert)
{
	int xpos = 6, ypos = 0;
	char *sidebandstr[] = {"USB", "LSB"};
	
	//Write string to position
	oled_putstring(xpos * FONTWIDTH, ypos, sidebandstr[sb], 0, invert);
}

//Sideband
void show_txrx(int tr)
{
	int xpos = 10, ypos = 0;
	char *trstr[] = {"RX", "TX"};
	
	//Write string to position
	oled_putstring(xpos * FONTWIDTH, ypos, trstr[tr], 0, tr);
}

void show_voltage(int v1)
{
    char *buffer;
	int t1, p;
	int xpos = 15, ypos = 0;
		
	buffer= malloc(0x10);
	//Init buffer string
	for(t1 = 0; t1 < 0x10; t1++)
	{
	    *(buffer + t1) = 0;
	}
    p = int2asc(v1, 1, buffer, 6) * 6;
    oled_putstring(xpos * 6, ypos, buffer, 0, 0);
	oled_putstring(p + xpos * 6, ypos, "V ", 0, 0);
	free(buffer);
}

void show_temp(int temperature)
{
    char *buffer;
	int t1, p;
	int xpos = 0, ypos = 1;
		
	buffer= malloc(0x10);
	//Init buffer string
	for(t1 = 0; t1 < 0x10; t1++)
	{
	    *(buffer + t1) = 0;
	}
    p = int2asc(temperature, -1, buffer, 6) * 6;
    oled_putstring(xpos * 6, ypos, buffer, 0, 0);
	oled_putchar1(p + xpos * 6, ypos, 0x87, 0);
	oled_putchar1(p + (xpos + 1) * 6, ypos, 'C', 0);
	free(buffer);
}

void show_agc(int a, int invert)
{
    int xpos = 10, ypos = 1;
		
	if(!a)
	{	
        oled_putstring(xpos * FONTWIDTH, ypos, "AGC-S", 0, invert);
    }
    else    
    {	
        oled_putstring(xpos * FONTWIDTH, ypos, "AGC-F", 0, invert);
    }
}

void show_tone(int t, int invert)
{
    int xpos = 5, ypos = 1;
		
	if(!t)
	{	
        oled_putstring(xpos * FONTWIDTH, ypos, "LOW ", 0, invert);
    }
    else    
    {	
        oled_putstring(xpos * FONTWIDTH, ypos, "HIGH ", 0, invert);
    }
}

void show_mem_num(int n, int invert)
{
    int xpos = 16, ypos = 1;
		
	oled_putstring(xpos++ * FONTWIDTH, ypos, "M", 0, invert);
	if(n < 10)
	{
		oled_putstring(xpos++ * FONTWIDTH, ypos, "0", 0, invert);
	}	
	oled_putnumber(xpos * FONTWIDTH, ypos, n, -1, 0, invert);
    
}

void show_split(int sp)
{
    int xpos = 0, ypos = 2;
		
	switch(sp)
	{
		case 0: oled_putstring(xpos * FONTWIDTH, ypos, "SPLT OFF", 0, 0);
		        break;
		case 1: oled_putstring(xpos * FONTWIDTH, ypos, "SPLT ON ", 0, 0);
		        break;        
    }
}
    
//S-Meter bargraph (Page 6)
void show_meter(int sv0)
{
    int sv;
	sv = sv0;
	
    if(sv > 120)
	{
	    sv = 120;
	}
	
	if(sv < 0)
	{
	    sv = 0;
	}
		
    //Draw bar graph
    oled_write_section(0, sv, 6, 0x1E);
    oled_write_section(sv + 2, 128, 6, 0);
	
    sv_old = sv;
    
    if(sv > smax) //Get max value
	{
		smax = sv;
		runseconds10s = runseconds10;
	}	
}

//Reset max value of s meter
void reset_smax(void)
{
	int s = get_s_value();
	
	s = (s / 3) * 3;
	
	//Clear bar graph
	oled_write_section(0, 128, 6, 0);
	smax = 0;	
	sv_old = 0;
	show_meter(s);
}	


    ////////////////////////
   //                    // 
  // Si5351A commands   //      
 //                    //
////////////////////////
void si5351_write(int reg_addr, int reg_value)
{
   	 
   twi_start();
   twi_write(SI5351_ADDRESS);
   twi_write(reg_addr);
   twi_write(reg_value);
   twi_stop();
} 

// Set PLLs (VCOs) to internal clock rate of 900 MHz
// Equation fVCO = fXTAL * (a+b/c) (=> AN619 p. 3
void si5351_start(void)
{
  unsigned long a, b, c;
  unsigned long p1, p2;//, p3;
    
  // Init clock chip
  si5351_write(XTAL_LOAD_CAP, 0xD2);      // Set crystal load capacitor to 10pF (default), 
                                          // for bits 5:0 see also AN619 p. 60
  si5351_write(CLK_ENABLE_CONTROL, 0x00); // Enable all outputs
  si5351_write(CLK0_CONTROL, 0x0F);       // Set PLLA to CLK0, 8 mA output
  si5351_write(CLK1_CONTROL, 0x2F);       // Set PLLB to CLK1, 8 mA output
  si5351_write(CLK2_CONTROL, 0x2F);       // Set PLLB to CLK2, 8 mA output
  si5351_write(PLL_RESET, 0xA0);          // Reset PLLA and PLLB

  // Set VCOs of PLLA and PLLB to 650 MHz
  a = PLLRATIO;     // Division factor 650/25 MHz !!!!
  b = 0;            // Numerator, sets b/c=0
  c = CFACTOR;      //Max. resolution, but irrelevant in this case (b=0)

  //Formula for splitting up the numbers to register data, see AN619
  p1 = 128 * a + (unsigned long) (128 * b / c) - 512;
  p2 = 128 * b - c * (unsigned long) (128 * b / c);
  //p3  = c;
  
  //Write data to registers PLLA and PLLB so that both VCOs are set to 900MHz intermal freq
  si5351_write(SYNTH_PLL_A, 0xFF);
  si5351_write(SYNTH_PLL_A + 1, 0xFF);
  si5351_write(SYNTH_PLL_A + 2, (p1 & 0x00030000) >> 16);
  si5351_write(SYNTH_PLL_A + 3, (p1 & 0x0000FF00) >> 8);
  si5351_write(SYNTH_PLL_A + 4, (p1 & 0x000000FF));
  si5351_write(SYNTH_PLL_A + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  si5351_write(SYNTH_PLL_A + 6, (p2 & 0x0000FF00) >> 8);
  si5351_write(SYNTH_PLL_A + 7, (p2 & 0x000000FF));

  si5351_write(SYNTH_PLL_B, 0xFF);
  si5351_write(SYNTH_PLL_B + 1, 0xFF);
  si5351_write(SYNTH_PLL_B + 2, (p1 & 0x00030000) >> 16);
  si5351_write(SYNTH_PLL_B + 3, (p1 & 0x0000FF00) >> 8);
  si5351_write(SYNTH_PLL_B + 4, (p1 & 0x000000FF));
  si5351_write(SYNTH_PLL_B + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  si5351_write(SYNTH_PLL_B + 6, (p2 & 0x0000FF00) >> 8);
  si5351_write(SYNTH_PLL_B + 7, (p2 & 0x000000FF));

}

void si5351_set_freq(int synth, unsigned long freq)
{
  unsigned long  a, b, c = CFACTOR; 
  unsigned long f_xtal = 25000000;
  double fdiv = (double) (f_xtal * PLLRATIO) / freq; //division factor fvco/freq (will be integer part of a+b/c)
  double rm; //remainder
  unsigned long p1, p2;
  
  a = (unsigned long) fdiv;
  rm = fdiv - a;  //(equiv. to fractional part b/c)
  b = rm * c;
  p1  = 128 * a + (unsigned long) (128 * b / c) - 512;
  p2 = 128 * b - c * (unsigned long) (128 * b / c);
      
  //Write data to multisynth registers of synth n
  si5351_write(synth, 0xFF);      //1048575 MSB
  si5351_write(synth + 1, 0xFF);  //1048575 LSB
  si5351_write(synth + 2, (p1 & 0x00030000) >> 16);
  si5351_write(synth + 3, (p1 & 0x0000FF00) >> 8);
  si5351_write(synth + 4, (p1 & 0x000000FF));
  si5351_write(synth + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  si5351_write(synth + 6, (p2 & 0x0000FF00) >> 8);
  si5351_write(synth + 7, (p2 & 0x000000FF));
}

    //////////////////////////////
   //                          // 
  // Radio frequency commands //      
 //                          //
//////////////////////////////
  

//Set LO freq to correct sideband
void set_lo_frequency(long f)
{
    si5351_set_freq(SYNTH_MS_0, f);	
}

//Set LO freq to correct sideband
void set_vfo_frequency(long f)
{
    si5351_set_freq(SYNTH_MS_1, f);	
}

  ///////////
 //LO ADJ //
///////////
void adj_lo_frequency(int sb)
{
	int key = 0;
		
    //LO FREQ USB or LSB
	key = 0;	
	show_frequency(f_lo[sb], 1); 
	if(!sb)
	{
	    oled_putstring(1, 6, "fBFO USB", 0, 0);
	}
	else
	{
	    oled_putstring(1, 6, "fBFO LSB", 0, 0);
	}
	    
	show_txrx(get_txrx());
	set_lo_frequency(f_lo[sb]);
	
	while(!key)
	{
		if(tuningknob < -2)  
		{    
		    f_lo[sb] += 10;
		    tuningknob = 0;
			show_frequency(f_lo[sb], 0);
			set_lo_frequency(f_lo[sb]);
		}	
		
		if(tuningknob > 2)  
		{    
		    f_lo[sb] -= 10;
			tuningknob = 0;
			show_frequency(f_lo[sb], 0);
			set_lo_frequency(f_lo[sb]);
		}	
		
	    key = get_keys();    
	}	
	
	while(get_keys());
}

//Scan memories if correct frequency in mem space
int scan_memories(int thresh)
{
	int key = 0;
	int sval;
    int m = 0;
    long runseconds10_scan = runseconds10, f_tmp;
        
    while(get_keys());
    
    oled_putstring(0, 0, "SCAN MEMORIES", 0, 0);
    
    while(!key)
    {
		show_mem_num(m, 0);
		f_tmp = load_frequency(0, m);
		if(is_mem_freq_ok(f_tmp))
		{
			show_frequency(f_tmp, 1);
			set_vfo_frequency(f_tmp + INTERFREQUENCY);
			runseconds10_scan = runseconds10;
			while(runseconds10_scan + 50 > runseconds10 && !key)
			{
				oled_putnumber(0, 7, 5 - (runseconds10 - runseconds10_scan) / 10, -1, 0, 0);;
				key = get_keys();
				sval = get_s_value();
				show_meter(sval);	
				if(thresh) //Halt at this frequency as long as there is stronger signal
		        {
				    while(sval > thresh && !key)
				    {
					    sval = get_s_value();
		                show_meter(sval);
		                key = get_keys();
		            }    
		        } 
			}	
		}	
		
		if(key == 2)
	    {
		    return m;
	    }
	    
		if(m < 15)
		{
			m++;
		}
		else
		{
			m = 0;
		}	
	}
	
	return -1;
}	

//Scan from VFOA to VFOB
//Swap frequencies if neccessary
long scan_vfo(int thresh)
{
	long f0, f1, f_tmp;
	int key = 0;
	int sval;
	
	f0 = f_vfo[0];
	f1 = f_vfo[1];
	
	while(get_keys());
	
	oled_putstring(0, 0, "SCAN VFOA > VFOB", 0, 0);
	
	if(f0 > f1)
	{
		f_tmp = f0;
		f0 = f1;
		f1 = f_tmp;
	}
	
	show_frequency(f0, 1);	
	oled_putnumber(0, 7, f0 / 10, 2, 0, 0);
	oled_putnumber(12 * FONTWIDTH, 7, f1 / 10, 2, 0, 0);
	
	while(!key)
	{
	    for(f_tmp = f0; f_tmp < f1; f_tmp += 10)
	    {	
		    set_vfo_frequency(f_tmp + INTERFREQUENCY);
		    show_frequency(f_tmp, 0);	
		    sval = get_s_value();
		    show_meter(sval);
		    
		    if(thresh) //Halt at this frequency as long as there is stronger signal
		    {
				while(sval > thresh && !key)
				{
					sval = get_s_value();
		            show_meter(sval);
		            key = get_keys();
		        }    
		    }        
	    	
	        key = get_keys();
	        if(key == 2)
	        {
			    return f_tmp;
		    }	
		}    
	}    
	
	return -1;
}

//Define S-Value where scnanning halts
int set_scan_threshold(int cur_thresh)
{
	int key = 0;
	int l_thresh =  cur_thresh;
	
	while(get_keys());
	
	oled_putstring(0, 0, "SCAN THRESH", 0, 0);
	//Draw bar graph
    oled_write_section(0, l_thresh, 6, 0x1E);
    oled_write_section(l_thresh, 128, 6, 0);
    oled_putnumber(2 * FONTWIDTH, 4, l_thresh, -1, 0, 0);
    
	while(!get_keys())
	{
		if(tuningknob < -2)  
		{    
		    if(l_thresh < 100)
		    {
				l_thresh++;
				//Draw bar graph
                oled_write_section(0, l_thresh, 6, 0x1E);
                oled_write_section(l_thresh, 128, 6, 0);
                oled_putnumber(2 * FONTWIDTH, 4, l_thresh, -1, 0, 0);
			}		 
			tuningknob = 0;
		}	
		
		if(tuningknob > 2)  
		{    
		    if(l_thresh > 0)
		    {
				l_thresh--;
				//Draw bar graph
                oled_write_section(0, l_thresh, 6, 0x1E);
                oled_write_section(l_thresh, 128, 6, 0);
                oled_putnumber(2 * FONTWIDTH, 4, l_thresh, -1, 0, 0);
			}		 
			tuningknob = 0;
		}	
		
	    key = get_keys();    
	    
	    if(key == 2)
	    {
			store_scan_thresh(l_thresh);
			return l_thresh;
		}	
	}
	return -1;
}	

//Calc increment/decrement rate from tuning speed
int calc_tuningfactor(void)
{
	return (tuningcount * tuningcount * -1); //-1 reverses tuning direction
}	

  /////////////////
 //  TONE HI/LO //
/////////////////
void set_tone(int toneset)
{
	if(!toneset)
	{
		DDRB |= (1 << PB2);
		PORTB &= ~(1 << PB2); //PIN lo => Tone lo 
    }
    else
    {
		DDRB &= ~(1 << PB2);  //Switch PIN to tri-state mode
    }   
}  

  ///////////////////
 // AGC FAST/SLOW //
///////////////////
void set_agc(int agcval)
{
	if(!agcval)
	{
		DDRB |= (1 << PB1);
		PORTB &= ~(1 << PB1); //PIN lo => Tone lo 
    }
    else
    {
		DDRB &= ~(1 << PB1);  //Switch PIN to tri-state mode
    }   
}  

  /////////////////////////
 //   E  E  P  R  O  M  //
/////////////////////////
void store_frequency(long f, int vfo, int memory)
{
    long hiword, loword;
    unsigned char hmsb, lmsb, hlsb, llsb;
	
    int start_adr =0;
    
    if(memory == -1)
    {
        start_adr = vfo * 4;
    }
	else
	{
        start_adr = memory * 4 + 16;
    }	    
	
    
	cli();
    hiword = f >> 16;
    loword = f - (hiword << 16);
    hmsb = hiword >> 8;
    hlsb = hiword - (hmsb << 8);
    lmsb = loword >> 8;
    llsb = loword - (lmsb << 8);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr, hmsb);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr + 1, hlsb);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr + 2, lmsb);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr + 3, llsb);
    
    sei();	
	
}

//Load a frequency from memory by memplace
long load_frequency(int vfo, int memory)
{
    long rf;
    unsigned char hmsb, lmsb, hlsb, llsb;
    int start_adr =0;
    
    if(memory == -1)
    {
        start_adr = vfo * 4;
    }
	else
	{
        start_adr = memory * 4 + 16;
    }	    
		
    cli();
    hmsb = eeprom_read_byte((uint8_t*)start_adr);
    hlsb = eeprom_read_byte((uint8_t*)start_adr + 1);
    lmsb = eeprom_read_byte((uint8_t*)start_adr + 2);
    llsb = eeprom_read_byte((uint8_t*)start_adr + 3);
	sei();
	
    rf = (long) 16777216 * hmsb + (long) 65536 * hlsb + (unsigned int) 256 * lmsb + llsb;
    //rf = (long) (hmsb << 24) + (long) (hlsb << 16) + (long) (lmsb << 8) + llsb;
		
	return rf;
}

//Check if freq is in 20m-band
int is_mem_freq_ok(long f)
{
	if(f >= 13999990 && f <= 14400000)
	{
		return 1;
	}	
	else
	{
		return 0;
	}		
}	

//Store last VFO used
void store_last_vfo(int vfonum)
{
    int start_adr = 8;
    
	cli();
    
    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr, vfonum);

    sei();	
}

//Store last VFO stored
int load_last_vfo(void)
{
    int start_adr = 8;
    int vfonum;
    
    cli();
    
	vfonum = eeprom_read_byte((uint8_t*)start_adr);
    
    sei();
	
	if(vfonum >= 0 && vfonum <= 1)
	{
		return vfonum;
	}
	else
	{
		return -1;
	}	
}

//Last memory used
void store_last_mem(int val)
{
	eeprom_write_byte((uint8_t*)11, val); //LAST MEM
}	

int recall_last_mem(void)
{
	int x =	eeprom_read_byte((uint8_t*)11); //LAST MEM
	if((x >= 0) && (x <= 15))
	{
		return x;
	}
	return -1;	
}

void store_tone(int val)
{
	eeprom_write_byte((uint8_t*)9, val); //TONE
}	

int recall_tone(void)
{
	int x =	eeprom_read_byte((uint8_t*)9); //TONE
	if((x == 0) || (x == 1))
	{
		return x;
	}
	return -1;	
}	

void store_agc(int val)
{
	eeprom_write_byte((uint8_t*)10, val); //AGC
}	

int recall_agc(void)
{
	int x =	eeprom_read_byte((uint8_t*)10); //AGC
	if((x == 0) || (x == 1))
	{
		return x;
	}
	return -1;
}	

void store_scan_thresh(int val)
{
	eeprom_write_byte((uint8_t*)12, val); //SCAN THRESH
}	

int recall_scan_thresh(void)
{
	int x =	eeprom_read_byte((uint8_t*)12); //SCAN THRESH
	if((x >= 0) && (x <= 100))
	{
		return x;
	}
	return -1;
}	

//////////////////////////////
//
//    M   E   N   U
//
//////////////////////////////
//Read keys via ADC0
//Short key press ret value 1 or 2
//Long key press ret value 11 or 12
/*
int get_keys(void)
{

    int key_value[] = {88, 143};
    int t1;
            
    //TEST display of ADC value 
   
    oled_putstring(0, 5, "----", 0, 0);    
    oled_putnumber(0, 5, adcval, -1, 0, 0); 
    _delay_ms(100);   
   
   	long rsecs0;
   	
    for(t1 = 0; t1 < 2; t1++)
    {
		if(get_adc(0) > key_value[t1] - 10 && get_adc(0) < key_value[t1] + 10)
        {
			rsecs0 = runseconds10;
			while(get_adc(0) > key_value[t1] - 10 && get_adc(0) < key_value[t1] + 10)
			{
				oled_putnumber(0, 7, (runseconds10 - rsecs0) / 10, -1, 0, 0);
			};
			
			if(runseconds10 > rsecs0 + 20)
			{
			    return t1 + 11;
            }
            else
            {
			    return t1 + 1;
            }    
        }
    }
    return 0;
}*/

//Read keys via ADC0
int get_keys(void)
{

    int key_value[] = {88, 143};
    int t1;
    int adcval = get_adc(0);
        
    //TEST display of ADC value 
    /*
    lcd_putstring(0, 5, "    ", 0, 0);    
    oled_putnumber(0, 5, adcval, -1, 0, 0);    
   	*/
    for(t1 = 0; t1 < 2; t1++)
    {
        if(adcval > key_value[t1] - 10 && adcval < key_value[t1] + 10)
        {
             return t1 + 1;
        }
    }
    return 0;
}


//////////////////////
//
//   A   D   C   
//
/////////////////////
//Read ADC value
int get_adc(int adc_channel)
{
	
	int adc_val = 0;
	
	ADMUX = (1<<REFS0) + adc_channel;     // Kanal adcmode aktivieren
    _delay_ms(1);
	
    ADCSRA |= (1<<ADSC);
    
	_delay_ms(1);
	
	adc_val = ADCL;
    adc_val += ADCH * 256;   
	
	return adc_val;
	
}	

int get_s_value(void)
{
	//oled_putnumber(0, 0, get_adc(1), -1, 0, 0);
	return ((get_adc(1) >> 2) + (get_adc(1) >> 3));
}	

int get_tx_pwr_value(void)
{
	//oled_putnumber(0, 0, get_adc(3), -1, 0, 0);
	return (get_adc(3) << 1);
}	

int get_txrx(void)
{
	if(PIND & (1<<PD0)) //PD0
	{
		return 1;
	}
	else
	{
		return 0;
	}
}	

int get_temp(void)
{
	int adc = get_adc(6);
	double ux = (double) (5 * adc) / 1023;
    double rx = 3000 / (5 / ux - 1);
	double temp = (rx - 1630) / 17.62;
	
	return (int) temp;
}	

  //////////
 // MENU //
//////////
void print_menu_head(char *head_str0, int m_items)
{	
    int xpos0 = 9;
			
	oled_cls(0);
	oled_drawbox(xpos0 * FONTWIDTH, 0, 18 * FONTWIDTH, m_items + 2);
	oled_putstring(0 , 0, head_str0, 0, 0);
}

void print_menu_item(char *m_str, int ypos, int inverted)
{
	int xpos1= 10;
	
	oled_putstring(xpos1  * FONTWIDTH, ypos + 1, m_str, 0, inverted);
}
	
//Print the itemlist or single item
void print_menu_item_list(int m, int item, int invert)
{
	int menu_items[] =  MENUITEMS; 
	
	char *menu_str[6][5] =    {{"VFO SWAP", "VFO B=A ", "VFO A=B ", "VFO>MEM ", "MEM>VFO "}, 
		                       {"USB     ", "LSB     ", "        ", "        "},
		                       {"TONE LO ", "TONE HI ", "AGC SLO ", "AGC FST "},
		                       {"MEMORY  ", "VFOs    ", "THRESH  ", "        "},
		                       {"SPLT OFF", "SPLT ON ", "        ", "        "},
		                       {"SET USB ", "SET LSB ", "        ", "        "}};
    int t1;
    
    if(item == -1)
    {
        //Print item list for menu
	    for(t1 = 0; t1 < menu_items[m] + 1; t1++)
	    {
		    print_menu_item(menu_str[m][t1], t1, 0);   
	    }	
	}
	else	
	{
		print_menu_item(menu_str[m][item], item, invert);   
	}	
}

//Returns menu_pos if OK or -1 if aborted
int navigate_thru_item_list(int m, int maxitems)
{
	int menu_pos = 0, menu_pos_old = -1;
	
	print_menu_item_list(m, menu_pos, 1)   ;     //Write 1st entry in normal color
	
	int key = get_keys();
	
    while(key == 0)
	{
		if(tuningknob < -2) //Turn CW
		{
			print_menu_item_list(m, menu_pos, 0); //Write old entry in normal color
		    if(menu_pos < maxitems)
		    {
				menu_pos++;
			}
			else	
			{
				menu_pos = 0;
			}
			print_menu_item_list(m, menu_pos, 1); //Write new entry in reverse color
		    tuningknob = 0;
		}

		if(tuningknob > 2)  //Turn CCW
		{    
		    print_menu_item_list(m, menu_pos, 0); //Write old entry in normal color
		    if(menu_pos > 0)
		    {
				menu_pos--;
			}
			else	
			{
				menu_pos = maxitems;
			}
			print_menu_item_list(m, menu_pos, 1); //Write new entry in reverse color
		    tuningknob = 0;
		}		
		
		//Preview of certain settings
		if(menu_pos != menu_pos_old)
		{
			switch(m)
			{
				case 0: oled_putnumber(0, 7, f_vfo[0] / 10, 2, 0, 0);    
				        oled_putnumber(10 * FONTWIDTH, 7, f_vfo[1] / 10, 2, 0, 0);    
				        break;
				case 1: set_lo_frequency(f_lo[menu_pos]);
				        oled_putnumber(0, 7, f_lo[menu_pos] / 10, 2, 0, 0);    
				        break;      
			}	         
			menu_pos_old = menu_pos;
		}		
		key = get_keys();
	}
		
	while(get_keys());
		
	switch(key)
	{
		case 1: return -1;
	            break; 
	    case 2: return menu_pos;
	            break; 
		case 11:return -11;
		        break;
	    default:return -1;
	}
	
	return -1;
}	

//Print mem numbers to grid in
//meu for user selection of mem place
void show_mem_menu_item(int m_item, int inv)
{
	int col, row;
	int x0 = 3, y0 = 2;
	long f_tmp;
		
	row = m_item / 4;
	col = m_item - row * 4;
	
	if(m_item < 10)
	{
		oled_putstring((col * 4 + x0) * FONTWIDTH, row + y0, "0", 0, inv);
		oled_putnumber((col * 4 + x0 + 1) * FONTWIDTH, row + y0, m_item, -1, 0, inv);
	}	
	else
	{
	    oled_putnumber((col * 4 + x0) * FONTWIDTH, row + y0, m_item, -1, 0, inv);
	} 
	
	//Preview
	f_tmp = load_frequency(0, m_item);
	if(is_mem_freq_ok(f_tmp) && inv)
	{
		set_vfo_frequency(f_tmp + INTERFREQUENCY);
		oled_putnumber(0, 7, f_tmp / 10, 2, 0, 0);
	}
	else
	{
		oled_putstring(0, 7, "********", 0, 0);
	}	
		
}	

int mem_select(int c_mem, int smode)
{
	int t1;
	int key = 0;
	int c = c_mem;
		
	int inv;
	
	while(get_keys());
	oled_cls(0);
	
	if(!smode)
	{
	    oled_putstring(2, 0, "VFO -> MEM", 0, 0);
	}
	else    
	{
	    oled_putstring(2, 0, "MEM -> VFO", 0, 0);
	}
	
	//Draw basic structure
	for(t1 = 0; t1 < 16; t1++)
	{
		
		if(t1 != c_mem)
		{
		    inv = 0;
		}
		else	
		{
			inv = 1;
		}
		show_mem_menu_item(t1, inv);
	}	
	
	key = get_keys();
	show_mem_menu_item(c_mem, 1);
	while(!key)
	{
		if(tuningknob < -2) //Turn CW
		{
			show_mem_menu_item(c, 0);
		    
		    if(c < 15)
		    {
				c++;
			}
			else	
			{
				c = 0;
			}
			show_mem_menu_item(c, 1);
		    tuningknob = 0;
		}

		if(tuningknob > 2)  //Turn CCW
		{    
		    show_mem_menu_item(c, 0);
		    
		    if(c > 0)
		    {
				c--;
			}
			else	
			{
				c = 15;
			}
			show_mem_menu_item(c, 1);
		    
		    tuningknob = 0;
		}		
		key = get_keys();
	}	
	
	switch(key)
	{
		case 1: return -1;
	            break; 
	    case 2: return c;
	            break; 
		case 11:return -11;
		        break;
	    default:return -1;
	 }
}	
			
long menux(long f, int c_vfo)
{
	
	int result = 0;
	int menu;
	int menu_items[] = MENUITEMS;
	
	////////////////
	// VFO FUNCS  //
	////////////////
	while(get_keys());
	menu = 0;
	print_menu_head("VFO/MEM", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, -1, 0);              //Print item list in full
	
	//Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu]);
	if(result > -1)
	{
		return(menu * 10 + result);
	}	
	switch(result)
	{	
		case -11:   return -1; //Quit menu         
	                break;   
	}
	
	////////////////
	// SIDEBAND  //
	////////////////
	while(get_keys());
	menu = 1;
	print_menu_head("SIDEBAND", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, -1, 0);              //Print item list in full
	
	//Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu]);
	
	if(result > -1)
	{
		return(menu * 10 + result);
	}	
	
	switch(result)
	{	
		case -11:   return -1; //Quit menu         
	                break;   
	}
	
	  //////////////////
	 // TONE AGC SET //
	//////////////////
	while(get_keys());
	menu = 2;
	print_menu_head("TONE/AGC", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, -1, 0);              //Print item list in full
	
	//Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu]);
	
	if(result > -1)
	{
		return(menu * 10 + result);
	}	
	
	switch(result)
	{	
		case -11:   return -1; //Quit menu         
	                break;   
	}
	
    /////////////////
	// SCAN MODES  //
	/////////////////
	while(get_keys());
	
	menu = 3;
	print_menu_head("SCAN", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, -1, 0);              //Print item list in full
	   
	//Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu]);
	
	if(result > -1)
	{
		return(menu * 10 + result);
	}	
	
	switch(result)
	{	
		case -11:   return -1; //Quit menu         
	                break;   
	}
	
	  /////////////////
	 // SPLIT MODE  //
	/////////////////
	while(get_keys());
	
	menu = 4;
	print_menu_head("SPLIT", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, -1, 0);              //Print item list in full
	   
	//Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu]);
		
	if(result > -1)
	{
		return(menu * 10 + result);
	}	
	
	switch(result)
	{	
		case -11:   return -1; //Quit menu         
	                break;   
	}
	
	/////////////////
	// LO SET MODE //
	/////////////////
	while(get_keys());
	
	menu = 5;
	print_menu_head("LO FREQ", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, -1, 0);              //Print item list in full
	   
	//Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu]);
					
	
	if(result > -1)
	{
		return(menu * 10 + result);
	}	
	
	switch(result)
	{	
		case -11:   return -1; //Quit menu         
	                break;   
	}
	
    
	return -2; //Nothing to do in main()
}

int main(void)
{
    int t1;
    int txrx = 0;
    int key = 0;
    int rval = 0;
    int split = 0;
    
    long runseconds10x = 0;
    
    //Volts measurement
    int adc_v;
    double v1;	
    
    //Tone
    int toneset = 0;
    
    //AGC
    int agcset = 0;
    
    //MEM
    int cur_mem = 0;
    
    //Temprary freq
    long f_tmp = 0;
    
    //Scan thresh
    int scan_thresh = 0;
    
	//INPUT
	PORTC = (1<<PC4) | (1<<PC5); //pullup Rs for I²C-Bus lines: PC4=SDA, PC5=SCL
    PORTC |= (1 << PC0); //Keys
    PORTD = (1 << PD5)|(1 << PD6); //Pull-up for Rotary Encoder
                
	//TWI init
	_delay_ms(100);
	twi_init();
	_delay_ms(100);
	
	//si5351
	si5351_start();
			
	//OLED
	oled_init();
	_delay_ms(20);
	oled_cls(0);	
		
	//Interrupt definitions for rotary encoder PD5 and PD6
	PCMSK2 |= ((1<<PCINT21) | (1<<PCINT22));  //enable encoder pins as interrupt source
	PCICR |= (1<<PCIE2);                      // enable pin change interupts 
	
	//ADC config and ADC init
    ADCSRA = (1<<ADPS0) | (1<<ADPS1) | (1<<ADEN); //Prescaler 64 and ADC on
	get_adc(0); //One dummy conversion
	
    //Timer 1 as counter for 10th seconds
    TCCR1A = 0;             // normal mode, no PWM
    TCCR1B = (1 << CS10) | (1 << CS12) | (1<<WGM12);   // Prescaler = 1/1024 based on system clock 16 MHz
                                                       // 15625 incs/sec
                                                       // and enable reset of counter register
	OCR1AH = (1562 >> 8);                             //Load compare values to registers
    OCR1AL = (1562 & 0x00FF);
	TIMSK1 |= (1<<OCIE1A);

    //Load VFO data and VFO number
    cur_vfo = load_last_vfo();
    if(cur_vfo < 0 || cur_vfo > 1)
    {
		cur_vfo = 0;
	}
	
	//Load VFO with last stored freq
	for(t1 = 0; t1 < 2; t1++)    
	{
        f_vfo[t1] = load_frequency(t1, -1);
        
        //oled_putnumber(0 + t1 * 60, 1, f_vfo[t1], -1, 0, 0);
        
        //Check if freq is in 20m band
        if(!is_mem_freq_ok(f_vfo[t1]))
        {
		    f_vfo[t1] = 14200000;
		    store_frequency(f_vfo[t1], t1, -1);
		}    
    }			
	 
	//Get setting for TONE from EEPROM    
	toneset = recall_tone();    
	if(toneset == -1)
	{
		toneset = 0;
	}	
	
	agcset = recall_agc();    
	if(agcset == -1)
	{
		agcset = 0;
	}	
	
	cur_mem = recall_last_mem();
	if(cur_mem == -1)
	{
		cur_mem = 0;
	}	
	
	scan_thresh = recall_scan_thresh();
	if(scan_thresh == -1)
	{
		scan_thresh = 0;
	}	
		
    set_vfo_frequency(f_vfo[cur_vfo] + INTERFREQUENCY);
    set_lo_frequency(f_lo[sideband]);
    show_frequency(f_vfo[cur_vfo], 1);
    show_vfo(cur_vfo, 0);
    show_sideband(sideband, 0);
    draw_meter_scale(0);
    show_txrx(txrx);
    
    set_tone(toneset);
    show_tone(toneset, 0);
    
    set_agc(agcset);
    show_agc(agcset, 0);

    show_split(split);
        
    show_mem_num(cur_mem, 0);
    
    sei();
    	  
    for(;;)
    {
		//TUNING		
		if(tuningknob > 2 && !txrx)  
		{    
		    f_vfo[cur_vfo] += calc_tuningfactor();  
		    set_vfo_frequency(f_vfo[cur_vfo] + INTERFREQUENCY);
			tuningknob = 0;
			show_frequency(f_vfo[cur_vfo], 0);
		}
		
		if(tuningknob < -2 && !txrx)
		{
		    f_vfo[cur_vfo] -= calc_tuningfactor();  
		    set_vfo_frequency(f_vfo[cur_vfo] + INTERFREQUENCY);
			tuningknob = 0;
			show_frequency(f_vfo[cur_vfo], 0);
		}
		
		if(runseconds10 > runseconds10x)
		{
			key = get_keys(); 
			runseconds10x = runseconds10;
		}	
		
		if(key == 1)
		{
			rval = menux(f_vfo[cur_vfo], cur_vfo);
			while(get_keys());
			oled_cls(0);
			switch(rval)
			{
				case 0: if(cur_vfo) //0 = Swap VFOs
				        {
							cur_vfo = 0;
						}
						else
						{
							cur_vfo = 1;
						}		
						store_last_vfo(cur_vfo);
				        set_vfo_frequency(f_vfo[cur_vfo] + INTERFREQUENCY);
				        break;
				 
				 case 1:f_vfo[1] = f_vfo[0]; //B=A
				        break;
				 
				 case 2:f_vfo[0] = f_vfo[1]; //A=B
				        break;
				                
				 case 3:cur_mem = mem_select(cur_mem, 0);
				        if(cur_mem > -1)
				        {
					        store_frequency(f_vfo[cur_vfo], 0, cur_mem);
				        }	
				        break;
				
				case 4: cur_mem = mem_select(cur_mem, 1); //MEM >>> VFO
				        if(cur_mem > -1)
				        {
					        f_tmp = load_frequency(0, cur_mem);
				            if(is_mem_freq_ok(f_tmp))
				            {
			 		            f_vfo[cur_vfo] = f_tmp;
			 		            set_vfo_frequency(f_vfo[cur_vfo] + INTERFREQUENCY);
			 		        }    
					        
				        }	
				        break;
				
			}	
						
			if(rval == 10 || rval == 11) //New sideband selected (USB or LSB)
			{
				sideband = rval - 10;
				show_sideband(sideband, 0);
				set_lo_frequency(f_lo[sideband]);
			}	
			
			if(rval == 20 || rval == 21) //New sideband selected (USB or LSB)
			{
				toneset = rval - 20;
				set_tone(toneset);
				store_tone(toneset);
			}	
			
			if(rval == 22 || rval == 23) //AGC
			{
				agcset = rval - 22;
				set_agc(agcset);
				store_agc(agcset);
			}	
			
						
			if(rval == 30) //SCAN MEMORIES
			{
				t1 = scan_memories(scan_thresh);
				//oled_putnumber( 10, 7, t1, -1, 0, 0);
				//_delay_ms(2000);
				if(t1 != -1)
				{
				    f_tmp = load_frequency(0, t1);
				    if(is_mem_freq_ok(f_tmp))
				    {
					    f_vfo[cur_vfo] = f_tmp;
					    set_vfo_frequency(f_tmp + INTERFREQUENCY);
				        cur_mem = t1;
				    }	
		        }
		    }    
		    
		    if(rval == 31) //SCAN VFOs
			{
				f_tmp = scan_vfo(scan_thresh);
				    
				 if(is_mem_freq_ok(f_tmp))
				 {
				    f_vfo[cur_vfo] = f_tmp;
					set_vfo_frequency(f_tmp + INTERFREQUENCY);
				 }	
		    }
			
			if(rval == 32) //Set scan threshold
			{
				scan_thresh = set_scan_threshold(scan_thresh);
				oled_putnumber(0, 5, scan_thresh, -1, 0, 0);
			}
			
			if(rval >= 40 && rval <= 42) //Split
			{
				split = rval - 40;
			}	
									
			if(rval == 50 || rval == 51) //LO FREQUENCIES
			{
				adj_lo_frequency(rval - 50);
			}	
							
			key = 0;
			set_vfo_frequency(f_vfo[cur_vfo] + INTERFREQUENCY);
			set_lo_frequency(f_lo[sideband]);
			//Show data
			oled_cls(0);
			show_frequency(f_vfo[cur_vfo], 1);
			show_vfo(cur_vfo, 0);
			show_mem_num(cur_mem, 0);
            show_sideband(sideband, 0);
            show_temp(get_temp());
            draw_meter_scale(0);
            show_txrx(txrx);
            show_tone(toneset, 0);
            show_agc(agcset, 0);
            show_split(split);
            draw_meter_scale(0);	
		}	
		
		if(key == 2)
		{
			store_last_vfo(cur_vfo);
			store_last_mem(cur_mem);
			store_frequency(f_vfo[cur_vfo], cur_vfo, -1); //Store VFO
			store_frequency(f_vfo[cur_vfo], cur_vfo, cur_mem); //Store current memory
			//oled_putstring(60, 0, "OK", 0, 0);
	    }
	    
	    if(!txrx)
	    {
            show_meter(get_s_value());		
        }
        else    
        {
            show_meter(get_tx_pwr_value());		
        }
        
        //Delete max value of meter every 2 seconds
        //Show volta0ge
		if(runseconds10 > runseconds10s + 20)
		{
			//S-Value reset
			reset_smax();
			show_meter(get_s_value());			
			
			//Voltage
			v1 = (double) get_adc(2) * 5 / 1024 * 5 * 10;
		    adc_v = (int) v1;
   		    show_voltage(adc_v);
   		    
   		    //PA Temp.
   		    show_temp(get_temp());
		}
				
		if(get_txrx()) //TX
		{
			if(!txrx)
			{
				draw_meter_scale(1);
				show_meter(0);
			    txrx = 1;
				show_txrx(txrx);
				
				if(split)
				{
					if(!cur_vfo)
					{
						cur_vfo = 1;
					}
					else	
					{
						cur_vfo = 0;
					}
					set_vfo_frequency(f_vfo[cur_vfo] + INTERFREQUENCY);
					show_frequency(f_vfo[cur_vfo], 1);
				}	
			}     
		}
		
		if(!get_txrx()) //RX
		{
			if(txrx) 
			{
				draw_meter_scale(0);
				show_meter(0);
			    txrx = 0;
				show_txrx(txrx);
				
				if(split)
				{
					if(!cur_vfo)
					{
						cur_vfo = 1;
					}
					else	
					{
						cur_vfo = 0;
					}
					set_vfo_frequency(f_vfo[cur_vfo] + INTERFREQUENCY);
					show_frequency(f_vfo[cur_vfo], 1);
				}	
			}
		}	
    }
	return 0;
}
