/*  Code for Production 3_2_15
 <Patriot_507_Alpha_Rev01, Basic Software to operate a 2 band SSB/CW QRP Transceiver.
 See PROJECT PATRIOT SSB/CW QRP below>
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
//  https://groups.yahoo.com/neo/groups/TenTec507Patriot/info/
// !! Disclaimer !!  !! Disclaimer !!  !! Disclaimer !!  !! Disclaimer !!  !! Disclaimer !!
//  Attention ****  Ten-Tec Inc. is not responsile for any modification of Code 
//  below. If code modification is made, make a backup of the original code. 
//  If your new code does not work properly reload the factory code to start over again.
//  You are responsible for the code modifications you make yourself. And Ten-Tec Inc.
//  Assumes NO libility for code modification. Ten-Tec Inc. also cannot help you with any 
//  of your new code. There are several forums online to help with coding for the ChipKit UNO32.
//  If you have unexpected results after writing and programming of your modified code. 
//  Reload the factory code to see if the issues are still present. Before contacting Ten_Tec Inc.
//  Again Ten-Tec Inc. NOT RESPONSIBLE for modified code and cannot help with the rewriting of the 
//  factory code!
/*

/*********  PROJECT PATRIOT SSB/CW QRP  *****************************
 * Program for the ChipKit Uno32
 * This is a simple program to demonstrate a 2 band QRP Amateur Radio Transceiver
 * Amateur Programmer Bill Curb (WA4CDM).
 * This program will need to be cleaned up a bit and no doubt be made more efficient!
 * Compiled using the MPIDE for the ChipKit Uno32.
 * 
 * Prog for ad9834
 * Serial timming setup for AD9834 DDS
 * start > Fsync is high (1), Sclk taken high (1), Data is stable (0, or 1),
 * Fsync is taken low (0), Sclk is taken low (0), then high (1), data changes
 * Sclk starts again.
 * Control Register D15, D14 = 00, D13(B28) = 1, D12(HLB) = X,
 * Reset goes high to set the internal reg to 0 and sets the output to midscale.
 * Reset is then taken low to enable output. 
 ***************************************************   
 * This is real basic code to get things working. 
 *****************************************************************
 * The pinout for the LCD is as follows: Also the LCD is setup up for 4 lines 20 charactors.
 * LCD RS pin to digital pin 26
 * LCD Enable pin to digital pin 27
 * LCD D4 pin to digital pin 28
 * LCD D5 pin to digital pin 29
 * LCD D6 pin to digital pin 30
 * LCD D7 pin to digital pin 31
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)    analogWrite(Side_Tone, 127);
 *****************************************************************
 * SELECT button steps from in 
 * BW ( <Wide, green>, <Medium, yellow>, <Narrow, red> ).
 * STEP ( <100 hz, green, <1Khz, yellow>, 10Khz, red> ).
 * BND ( < 40M >, < 20M >, < , > ) OTHER has yet to be defined
 * 
 * Default Band_width will be wide ( Green led lite ).
 * When pressing the function button one of three leds will lite. 
 * as explained above the select button will choose which setting will be used. 
 * The Orange led in the Ten-Tec logo will flash to each step the STEP is set 
 * too when tuning.  As it will also turn on when at the BAND edges.
 * The TT logo led will also flash to indicate ALC. Input levels should be kept low enough
 * to only flash this led on Peaks.  
 * Default frequency on power up will be the calling frequency of the 
 * 40 meter band. 
 * I.F. Frequency used is 9.0 mhz.
 * DDS Range is: 
 * 40 meters will use HI side injection.
 * 9(I.F.) + 7(40m) = 16mhz.  9(I.F.) + 7.30 = 16.3 mhz.
 * 20 meters will use LO side injection.
 * 14(20m) - 9(I.F.) = 5mhz.  14.350(20m) - 9(I.F.) = 5.35 mhz.
 * 
 * The Headphone jack can supply a headphone or speaker. The header pins(2) 
 * if shorted will drive a speaker.
 * Unshorted inserts 100 ohm resistors in series with the headphone to limit 
 * the level to the headphones.
 * 
 * The RIT knob will be at 0 offset in the Top Dead Center position. And will 
 * go about -500 hz to +500 hz when turned to either extreme. Total range 
 * about +/- 500 hz. This may change!
 * 
 **************************************************************************************  
 * 
 * Added an MCP23017 16-bit I/O Expander with Serial Interface to free up
 * some I/O pins on the ChipKit Uno32 board.
 * The parts of the 507 being controlled by this ic will be the Multi-purpose
 * leds, the Select leds and the Wide/medium/Narrow control.
 * 5/1/2014 added a couple of routines to keep the filter wide on TX of SSB or CW
 * Port B can be used by the user for external control.
 * 
 * GPAO (21) Select Green led
 * GPA1 (22) Select Yellow led
 * GPA2 (23) Select Red led
 * GPA3 (24) MP_A Green led
 * GPA4 (25) MP_B Yellow led
 * GPA5 (26) MP_C Red led
 * GPA6 (27) Medium A8 BW_control
 * GPA7 (28) Narrow A9 BW_control
 * 
 * A mask function will be used to combine the various bits together.
 */
/*  Modifications by W8CQD
 *   3/17/2015 - Fix Serial Dump to report values for all fields
 *   3/22/2015 - Comment out Serial Dump
 *   3/24/2015 - Integrate WD9GYM's S-Meter changes into base LCD code
 *   3/24/2015 - Change version to 1_11_W8CQD
 *   3/31/2015 - Version 12, remove "Band" and replace with "PWR" for output power
 *               Consolidate STEP/BW display and add Voltage display
 *   4/08/2015 - Version 13, change lcd to I2C based lcd, change lcd display to:  
 *                Line 0: RX Frequency, RIT Offset (with labels)
 *                Line 1: TX Frequency, TX Power (with labels)
 *                Line 2: Input Voltage, S Meter (with labels)
 *                Line 3: Filter Bandwith, Tuning Step, TX/RX Indicator, Mode (no labels)
 *               Changed band change code to clear TX frequency when switching bands
 *               Also did some code cleanup
 *  4/14/2015  - Version 14, change frequency defaults to 7.076 and 14.076
 *                           and default band to 20m
 *               Added from Patriot Alliance mod beta 5:
 *               1.  Correct DDS offset
 *               2.  CAT based on K3 protocol
 *               3.  Faster Encoder (abt 3.5 times faster)
 *               4.  Pressing FUNCTION>.5 seconds will put TRX into TUNE (carrier) mode
 */

// various defines
#define SDATA_BIT                           11          // 
#define SCLK_BIT                            12          //  
#define FSYNC_BIT                           13          //  
#define RESET_BIT                           10          //  
#define FREQ_REGISTER_BIT                   9           //  
#define PHASE_REGISTER_BIT                  8
#define AD9834_FREQ0_REGISTER_SELECT_BIT    0x4000      //  
#define AD9834_FREQ1_REGISTER_SELECT_BIT    0x8000      //  
#define FREQ0_INIT_VALUE                    0x00000000  // 0x01320000

#define led                                 13
#define MUTE                                4
#define MIC_LINE_MUTE                       34

#define Side_Tone                           3           // 

#define PTT_SSB                             22          // ptt input pulled high 
#define SSB_CW                              42          // control line for /SSB_CW switches output
// high for cw , low for ssb
#define TX_Dah                              33          // 
#define TX_Dit                              32          //  
#define TX_OUT                              38          //  
#define Band_End_Flash_led                  24          // // also this led will flash every 100/1khz/10khz is tuned
#define Band_Select                         41          // output for band select
#define Multi_Function_Button               5           //
#define Flash                               Band_End_Flash_led

#define Select_Button                       2           // 

#define Wide_BW                             0           // 
#define Medium_BW                           1           // 
#define Narrow_BW                           2           // 


#define Step_100_Hz                         0
#define Step_1000_hz                        1
#define Step_10000_hz                       2

#define  Other_1_user                       0           // 40 meters
#define  Other_2_user                       1           // 20 meters
#define  Other_3_user                       2           // anything you desire!

/**************************************************
 * WD9GYM modification for S meter on I2C LCD
 **************************************************/

// LCD Bars
byte meter_s1[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11000,
  B11000
};
byte meter_s3[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00011,
  B11011,
  B11011
};
byte meter_s5[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B11000,
  B11000,
  B11000,
  B11000,
};
byte meter_s7[8] = {
  B00000,
  B00000,
  B00000,
  B00011,
  B11011,
  B11011,
  B11011,
  B11011,
};
byte meter_s9[8] = {
  B00000,
  B00000,
  B11000,
  B11000,
  B11000,
  B11000,
  B11000,
  B11000,
};
byte meter_s10[8] = {
  B00000,
  B00011,
  B11011,
  B11011,
  B11011,
  B11011,
  B11011,
  B11011,
};
byte meter_s20[8] = {
  B11000,
  B11000,
  B11000,
  B11000,
  B11000,
  B11000,
  B11000,
  B11000,
};
byte meter_s30[8] = {
  B11011,
  B11011,
  B11011,
  B11011,
  B11011,
  B11011,
  B11011,
  B11011,
};

volatile unsigned long smeterTime, smeterLast, smeterInterval;
volatile unsigned long voltsTime, voltsLast, voltsInterval;
volatile unsigned long powerTime, powerLast, powerInterval;

const int RitReadPin        = A0;  // pin that the sensor is attached to used for a rit routine later.
int RitReadValue            = 0;
int RitFreqOffset           = 0;
int old_RitFreqOffset       = 0;

const int SmeterReadPin     = A1;  // To give a realitive signal strength based on AGC voltage.
int SmeterReadValue         = 0;

const int BatteryReadPin    = A2;  // Reads 1/5 th or 0.20 of supply voltage.
int BatteryReadValue        = 0;

const int PowerOutReadPin   = A3;  // Reads RF out voltage at Antenna.
int PowerOutReadValue       = 0;

const int CodeReadPin       = A6;  // Can be used to decode CW. 
int CodeReadValue           = 0;

const int CWSpeedReadPin    = A7;  // To adjust CW speed for user written keyer.
int CWSpeedReadValue        = 0;            
int CWSpeed                 = 0;

// Tune function variables
// this sketch let you key the tx for 10 seconds or less by selecting U3 
// From Patriot Alliance mod beta 5
unsigned long  tuneStartTime    = 0;
unsigned long  tuneElapsedTime  = 0;
int tune = 0;

// define terminal / cat active at start
// From Patriot Alliance mod beta 5
int terminal = 1;                              // terminal active at start
String CatStatus = "T";

unsigned long  catStartTime    = 0;
unsigned long  catElapsedTime  = 0;
// stage buffer to avoid blocking on serial writes when using CAT
#define STQUESIZE 64
unsigned char stg_buf[STQUESIZE];
int stg_in = 0;
int stg_out = 0;

// bsm=0 is 40 meter, bsm=1 is 20 meter from Patriot Alliance mod beta 5
int Band_bsm0_Low              = 7000000;
int Band_bsm0_High             = 7300000;
int Band_bsm1_Low              = 14000000;    
int Band_bsm1_High             = 14350000;

#include "Wire.h"

//Uncomment the two lines referencing the LiquidCrystal if using a serial LCD and comment out the two referencing LiquidCrystal_I2C

//#include <LiquidCrystal.h>    //  LCD Stuff
//LiquidCrystal lcd(26, 27, 28, 29, 30, 31);      //  LCD Stuff

#include <LiquidCrystal_I2C.h>    // WD9GYM modification
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address  WD9GYM modification

const char txt0[7]          = "1_14";
const char txt2[8]          = "W8CQD";
const char txt52[5]         = " ";
const char txt57[6]         = "FREQ:" ;
const char txt60[6]         = "STEP:";
const char txt62[3]         = "RX";
const char txt63[3]         = "TX";
const char txt64[4]         = "RIT";
const char txt65[5]         = "Band";
const char txt66[4]         = "20M";
const char txt67[4]         = "40M";
const char txt69[4]         = "   ";
const char txt70[5]         = "    ";
const char txt71[6]         = "     ";
const char txt72[10]        = "        ";
const char txt85[2]         = "W";
const char txt86[2]         = "M";
const char txt87[2]         = "N";
const char txt90[5]         = "STEP";
const char txt110[4]        = "VLT";
const char txt120[3]        = "BW";
const char txt130[5]        = "MODE";
const char txt132[3]        = "CW";
const char txt135[4]        = "SSB";
const char txt140[5]        = "WIDE";
const char txt150[7]        = "MEDIUM";
const char txt160[7]        = "NARROW";
const char txt170[7]        = "      ";
const char txt175[2]        = "S";
const char txt176[4]        = "PWR";  //W8CQD
const char txt177[3]        = "  "; //W8CQD

String stringFREQ;
String stringREF;
String string_Frequency_Step;
String stringRIT;
String stringBW;

float floatVolts = 0.00;  //W8CQD

int  level; //S meter value for serial dump
int TX_key;
int PTT_SSB_Key;
int old_PTT_SSB_Key;

int band_sel;                           // select band 40 or 20 meter
int band_set;
int bsm;  
int mode;
int old_mode;

int Step_Select_Button          = 0;
int Step_Select_Button1         = 0;
int Step_Multi_Function_Button  = 0;
int Step_Multi_Function_Button1 = 0;

int Selected_BW                 = 0;    // current Band width 
// 0= wide, 1 = medium, 2= narrow
int Selected_Step               = 0;    // Current Step
int Selected_Other              = 0;    // To be used for anything

int old_bsm                     = 0;    //  this helps 5/13/14

int old_BatteryReadValue        = 0;

int old_SMeterReadValue         = 0;

int status_RXTX                 = 0;
int old_status_RXTX             = 0;

byte s = 0x00;                    // s = select
byte m = 0x00;                    // m = multi
byte b = 0x00;                    // b = bandwidth
byte t = 0x00;                    // s + m ored
byte old_b = 0x00;                // for the TX routine

//-----------------------------------------------------
// Encoder Stuff
// including additional functions from Patriot Alliance mod beta 5
const int encoder0PinA          = 6; // reversed for 507
const int encoder0PinB          = 7; // reversed for 507

int val; 
int encoder0Pos                 = 0;
int encoder0PinALast            = LOW;
int encoder0PinBLast            = LOW;
int n                           = LOW;
int o                           = LOW;

//-----------------------------------------------------
long DDS_offset_value           = 400;              // Offset for easy frequency tweaking by HB9MTN copied from Patriot Alliance mod beta 5
long offset;
const long meter_40             = 16.076e6;        // IF + Band frequency, default for 40 - changed to 7.076 by W8CQD
// HI side injection 40 meter 
// range 16 > 16.3 mhz
const long meter_20             = 5.076e6;       // Band frequency - IF, LOW  default for 20 - changed to 14.076 by W8CQD
// side injection 20 meter 
// range 5 > 5.35 mhz
const long Reference            = 50.0e6;   // for ad9834 this may be 
// tweaked in software to 
long frequency_TX;                          // fine tune the Radio
long frequency_40               = meter_40;   //saved 40m freq from Patriot Alliance mod beta 5
long frequency_20               = meter_20;   //saved 20m freq from Patriot Alliance mod beta 5
long TX_frequency;
long RIT_frequency;
long RX_frequency;
long save_rec_frequency;
long Frequency_Step;
long old_Frequency_Step;
long frequency                  = 0;
long frequency_old              = 0;
long frequency_old_TX           = 0;
long frequency_tune             = 0;
long old_frequency_tune         = 0;
long frequency_default          = 0;
long fcalc;
long IF_value                   = 9.00e6;          //  I.F. Frequency
long IF                         = 9.00e6;          //  I.F. Frequency
long TX_Frequency               = 0;

//-----------------------------------------------------
// Debug Stuff

unsigned long   loopCount       = 0;
unsigned long   lastLoopCount   = 0;
unsigned long   loopsPerSecond  = 0;
unsigned int    printCount      = 0;

unsigned long   loopStartTime   = 0;
unsigned long   loopElapsedTime = 0;
float           loopSpeed       = 0;

unsigned long LastFreqWriteTime = 0;

void    serialDump();

//-----------------------------------------------------

void Default_frequency();
void AD9834_init();
void AD9834_reset();
void program_freq0(long freq);
void program_freq1(long freq1);  
void UpdateFreq(long freq);

void RX_Rit();             

//   void Frequency_up();                        
//   void Frequency_down(); 

void TX_routine();
void RX_routine();
void Encoder();
void AD9834_reset_low();
void AD9834_reset_high();

void Change_Band();
void Step_Flash();
void RIT_Read();

void Multi_Function();          
void Step_Selection();           
void Selection();               
void Step_Multi_Function();

//----------------------------------------------------- 

void clock_data_to_ad9834(unsigned int data_word);

//-----------------------------------------------------

void setup() 
{
  // these pins are for the AD9834 control
  pinMode (SCLK_BIT,                 OUTPUT);    // clock
  pinMode (FSYNC_BIT,                OUTPUT);    // fsync
  pinMode (SDATA_BIT,                OUTPUT);    // data
  pinMode (RESET_BIT,                OUTPUT);    // reset
  pinMode (FREQ_REGISTER_BIT,        OUTPUT);    // freq register select

  //---------------  Encoder ----------------------------
  pinMode (encoder0PinA,             INPUT);     // 
  pinMode (encoder0PinB,             INPUT);     // 

  //-----------------------------------------------------
  pinMode (TX_Dit,                   INPUT);     // Dit Key line 
  pinMode (TX_Dah,                   INPUT);     // Dah Key line
  pinMode (TX_OUT,                   OUTPUT);    // control line for TX stuff
  pinMode (Band_End_Flash_led,       OUTPUT);    // line that flashes an led
  pinMode (PTT_SSB,                  INPUT);     // mic key has pull-up
  pinMode (SSB_CW,                   OUTPUT);    // control line for ssb cw switches

  pinMode (Multi_Function_Button,    INPUT);     // Choose from Band width, Step size, Other

  pinMode (Select_Button,            INPUT);     //  Selection from the above

  pinMode (Side_Tone,                OUTPUT);    // sidetone enable

  pinMode (Band_Select,              OUTPUT);

  pinMode (MUTE,                     OUTPUT);

  pinMode (MIC_LINE_MUTE,            OUTPUT);   // low on receive

  digitalWrite (Band_End_Flash_led,  LOW); //  not in 81324

  digitalWrite (MUTE,                LOW);

  Default_Settings();
  // I2C stuff
  Wire.begin();                             // wake up I2C bus
  Wire.beginTransmission(0x20);  
  Wire.send(0x00);                          // IODIRA register
  Wire.send(0x00);                          // set all of port A to outputs
  Wire.endTransmission();
  Wire.beginTransmission(0x20);
  Wire.send(0x01);                          // IODIRB register
  Wire.send(0x00);                          // set all of port B to outputs
  Wire.endTransmission();

  //-----------------------------------------------------
  // DDS
  AD9834_init();
  AD9834_reset();                            // low to high
  //-----------------------------------------------------

  digitalWrite(TX_OUT,              LOW);      // turn off TX
  digitalWrite(SSB_CW,              LOW);      // keeps tx in ssb mode until high

  //-----------------------------------------------------
  Frequency_Step = 100;   //  Can change this whatever step size one wants
  Selected_Step = Step_100_Hz; 
  DDS_Setup();        
  encoder0PinALast = digitalRead(encoder0PinA);   
  //attachInterrupt(encoder0PinA, Encoder, CHANGE);
  //attachInterrupt(encoder0PinB, Encoder, CHANGE);
  attachCoreTimerService(TimerOverFlow);//See function at the bottom of the file.

  Serial.begin(115200);
  Serial.println("Patriot Ready:");

  lcd.begin(20, 4);                          // 20 chars 4 lines
  // or change to suit ones 
  // lcd display 
  lcd.clear();                              // WD9GYM modification adjusted by W8CQD
  lcd.setCursor(0, 0);
  lcd.print("W8CQD - PATRIOT");
  lcd.setCursor(0, 1);
  lcd.print("Ver_1_14_W8CQD");
  lcd.setCursor(0,2);
  lcd.print("4/14/2015");
  delay(3000);
  lcd.clear();
  Display_Setup();

}   //    end of setup

//===================================================================
//===================================================================
void Display_Setup()
{
  // RX
  lcd.setCursor(0, 0);
  lcd.print(txt62);      // RX

  // RIT
  lcd.setCursor(12, 0);
  lcd.print(txt64);      // RIT

  //PWR - W8CQD
  lcd.setCursor(12, 1);
  lcd.print(txt176);       // Transmit Power

  // TX
  lcd.setCursor(0, 1);
  lcd.print(txt63);       // TX 

  // BAND - replaced by PWR W8CQD
  //lcd.setCursor(12, 1);
  //lcd.print(txt65);       // BAND
  // default band
  //lcd.setCursor(17, 1);
  //lcd.print(txt67);       // 40M   change this to txt66 for display of 20M

  // STEP - changed to line 4, changed label W8CQD
  //lcd.setCursor(9, 3);
  //lcd.print(txt178);       // STEP

  // VLT - Input Voltage
  lcd.setCursor(0, 2);
  lcd.print(txt110);      // VLT

  // S - S Meter
  lcd.setCursor(12, 2);
  lcd.print(txt175);    // S

  // BW - changed label W8CQD
  //lcd.setCursor(0, 3);
  //lcd.print(txt177);      // BW
  // default BW
  lcd.setCursor(0, 3);
  lcd.print(txt140);      // Wide

  // MODE - no label needed, self explanatory
  //lcd.setCursor(12, 3);
  //lcd.print(txt130);      // MODE
  // default MODE
  //lcd.setCursor(17, 3);
  //lcd.print(txt69);
  lcd.setCursor(17, 3);
  lcd.print(txt135);  // SSB

  // TX/RX Status
  lcd.setCursor(13,3);
  lcd.print(txt62); // RX
}      // end of Display_Setup

//===================================================================

void Default_Settings()
{
  m = 0x08;                 //     

  s = 0x01;                 //

  bsm = 0;                  //  bsm = 0 is 40 meters bsm = 1 is 20 meters

  frequency_default = meter_40; // change this to meter_20 for 20 meter default
  Default_frequency();
  b = 0x00; // Hardware control of I.F. filter shape wide setting
  Selected_BW = Wide_BW; 

  digitalWrite (TX_OUT,               LOW);                                            
  digitalWrite (Band_End_Flash_led,   LOW);
  digitalWrite (Side_Tone,            LOW);    
  digitalWrite ( FREQ_REGISTER_BIT,   LOW);
  digitalWrite ( SSB_CW,              LOW);
  digitalWrite ( Band_Select,         LOW);
  digitalWrite ( MUTE,                HIGH);
  digitalWrite ( MIC_LINE_MUTE,       LOW);    //  receive mode
}

//-----------------------------------------------------------------
void DDS_Setup()
{
  digitalWrite(FSYNC_BIT,             HIGH);  // 
  digitalWrite(SCLK_BIT,              HIGH);  // 
}

//======================= Main Part =================================
void loop()    
{
  //  TX_routine(); 
  Encoder();
  TX_routine(); 
  RX_Rit();
  Multi_Function(); 
  //CAT function from Patriot Alliance mod beta 5
  if( !un_stage() ) Poll_Cat();              // send data to CAT. If nothing to send, get next command.
  loopCount++;
  loopElapsedTime    = millis() - loopStartTime;

  // Following two lines replace by new S Meter code WD9GYM/W8CQD
  //SmeterReadValue = analogRead(SmeterReadPin);
  //splash_Smeter();

  loopCount++;
  loopElapsedTime    = millis() - loopStartTime;

  if( 1000 <= loopElapsedTime )
  {
    // serialDump();    // comment this out to remove the one second tick
  }
  Splash_Volts(); //Display input volatage - W8CQD
  splash_Smeter();  //Add call to S Meter display routine WD9GYM/W8CQD, changed routine name for consistency
}    //  END LOOP

//-----------------------------------------------------
void  RX_Rit()
{
  RIT_Read();
  frequency_tune  = frequency + RitFreqOffset; // RitFreqOffset is from Rit_Read();
  UpdateFreq(frequency_tune);
  splash_RX_freq();   // this only needs to be updated when encoder changed. 
}

//------------------------------------------------------
void RIT_Read()
{
  int RitReadValueNew =0 ;
  RitReadValueNew = analogRead(RitReadPin);

  RitReadValue = (RitReadValueNew + (12 * RitReadValue)) / 13;//Lowpass filter possible display role if changed
  if(RitReadValue < 500) 
    RitFreqOffset = RitReadValue-500;
  else if(RitReadValue < 523) 
    RitFreqOffset = 0;//Deadband in middle of pot
  else 
    RitFreqOffset = RitReadValue - 523;

  splash_RIT();    //   comment out if display is not needed
}

//-----------------------------------------------------
void UpdateFreq(long freq)
{
  if (LastFreqWriteTime != 0)
  { 
    if ((millis() - LastFreqWriteTime) < 100) return; 
  }
  LastFreqWriteTime = millis();
  if(freq == frequency_old) return;
  program_freq0( freq );
  frequency_old = freq;
}

//--------------------------------------------------------------
void UpdateFreq1(long frequency_TX)
{
  if (LastFreqWriteTime != 0)
  { 
    if ((millis() - LastFreqWriteTime) < 100) return; 
  }
  LastFreqWriteTime = millis();
  if(frequency_TX == frequency_old_TX) return;
  program_freq1( frequency_TX );
  frequency_old_TX = frequency_TX;
}

// From Patriot Alliance mod beta 5 - modified by W8CQD
void TX_on()
{
  TX_Frequency = frequency;
  frequency_tune  = TX_Frequency;             	// RitFreqOffset is from Rit_Read();
  digitalWrite ( FREQ_REGISTER_BIT,   HIGH);  	// 
  UpdateFreq1(frequency_tune);

  old_b = b;                                 	// save original b into old_b
  b = 0x00;                                   	// b is now set to wide filter setting
  Select_Multi_BW_Ored();                     	// b is sent to port expander ic

  if (mode==0) {
    digitalWrite ( MIC_LINE_MUTE, HIGH );       // turns Q35, Q16 off, unmutes mic/line
    digitalWrite ( SSB_CW, HIGH );            	// this causes the ALC line to connect
  } 
  else {
    digitalWrite ( SSB_CW, LOW );               // enables the cw pull down circuit  
    digitalWrite ( Side_Tone, HIGH );           // enables side-tone source
  }
  digitalWrite ( TX_OUT, HIGH );              	// Truns on Q199 (pwr cntrl)(switched lo/dds)
  // Read and display power out  W8CQD
  Splash_MODE();
  status_RXTX = 1; //W8CQD set to TX mode
  Splash_RXTX();  //W8CQD change indicator
  Splash_PWR();  //W8CQD display TX power
}

void TX_off() {
  b = old_b;                                  	// original b is now restored
  Select_Multi_BW_Ored();                     	// original b is sent to port expander

  digitalWrite ( TX_OUT, LOW );               	// turn off TX stuff
  digitalWrite ( FREQ_REGISTER_BIT, LOW );    	// added 6/23/14  
  if (mode==0) { 
    digitalWrite ( MIC_LINE_MUTE, LOW ); 
  }        // turns Q36, Q16 on, mutes mic/line
  if (mode==1) { 
    digitalWrite ( Side_Tone, LOW ); 
  }            // side-tone off
  status_RXTX = 0; //W8CQD set to RX mode
  Splash_RXTX(); //W8CQD change indicator
}

//------------------------------------------------------------------
//##################################################################
//---------------------  TX Routine  -------------------------------
void TX_routine()      

{     //------------------  SSB Portion  ----------------------------

  PTT_SSB_Key = digitalRead( PTT_SSB );               // check to see if PTT is pressed, 
  if   ( PTT_SSB_Key == LOW  )                       // if pressed do the following
  {
    do
    {
      TX_Frequency = frequency;
      frequency_tune  = TX_Frequency;             // RitFreqOffset is from Rit_Read();
      digitalWrite ( FREQ_REGISTER_BIT,   HIGH);      // 
      UpdateFreq1(frequency_tune);
      Splash_MODE();
      splash_TX_freq();

      old_b = b;                                 // save original b into old_b
      b = 0x00;                                // b is now set to wide filter setting
      Select_Multi_BW_Ored();                       // b is sent to port expander ic

      digitalWrite ( MIC_LINE_MUTE, HIGH);        // turns Q35, Q16 off, unmutes mic/line
      digitalWrite (SSB_CW, HIGH);              // this causes the ALC line to connect
      digitalWrite(TX_OUT, HIGH);             // Truns on Q199 (pwr cntrl)(switched lo/dds)
      // mutes audio to lm386
      // Read and display power out  W8CQD
      status_RXTX = 1; //W8CQD set to TX mode
      Splash_RXTX();  //W8CQD change indicator
      Splash_PWR();  //W8CQD display TX power
      PTT_SSB_Key = digitalRead(PTT_SSB);           // check to see if PTT is pressed  
    }     
    while (PTT_SSB_Key == LOW); 

    b = old_b;                                    // original b is now restored
    Select_Multi_BW_Ored();                      // original b is sent to port expander
    digitalWrite(TX_OUT, LOW);                  // turn off TX stuff
    digitalWrite ( FREQ_REGISTER_BIT,   LOW); // added 6/23/14  
    digitalWrite ( MIC_LINE_MUTE, LOW);     // turns Q36, Q16 on, mutes mic/line
    status_RXTX = 0; //W8CQD set to RX mode
    Splash_RXTX(); //W8CQD change indicator
  }  // End of SSB TX routine
  //---------------  CW Portion  --------------------------------
  TX_key = digitalRead(TX_Dit);                      //  Maybe put these on an interrupt!
  if ( TX_key == LOW)                               // was high   
  {
    do
    {

      TX_Frequency = frequency;
      frequency_tune  = TX_Frequency;          // RitFreqOffset is from Rit_Read();
      digitalWrite ( FREQ_REGISTER_BIT,   HIGH);      //    
      UpdateFreq1(frequency_tune);
      Splash_MODE();  
      splash_TX_freq();

      old_b = b; 
      b = 0x00;                                  // b is now set to wide filter setting
      Select_Multi_BW_Ored();                  // b is sent to port expander ic


      digitalWrite(TX_OUT, HIGH);                    // turns tx circuit on
      digitalWrite (SSB_CW, LOW);                  // enables the cw pull down circuit  
      digitalWrite(Side_Tone, HIGH);             // enables side-tone source
      // Read and display power out  W8CQD
      status_RXTX = 1; //W8CQD set to TX mode
      Splash_RXTX();  //W8CQD change indicator
      Splash_PWR();   //W8CQD display TX power
      TX_key = digitalRead(TX_Dit);            // reads dit key line
    }  
    while (TX_key == LOW);                       // key still down
    b = old_b;                                    // original b is now restored
    Select_Multi_BW_Ored();   
    for (int i=0; i <= 10e2; i++);            // delay
    digitalWrite(TX_OUT, LOW);               // trun off TX cw key is now high
    digitalWrite ( FREQ_REGISTER_BIT,   LOW);    // return to DDS register 0  not in other 
    status_RXTX = 0; //W8CQD set to RX mode
    Splash_RXTX();  //W8CQD change indicator
    digitalWrite(Side_Tone, LOW);                   // side-tone off
  }
}                                                      // end  TX_routine()

//------------------------CAT Routine based on Elecraft K3 -------------------------------
//   some general routines for serial printing
// from Patriot Alliance mod beta 5

int un_stage(){              // send a char on serial 
  char c;
  if( stg_in == stg_out ) return 0;
  c = stg_buf[stg_out++];
  stg_out &= ( STQUESIZE - 1);
  Serial.write(c);
  return 1;
}
void stage( unsigned char c ){
  stg_buf[stg_in++] = c;
  stg_in &= ( STQUESIZE - 1 );
}
void stage_str( String st ){
  int i;
  char c;
  for( i = 0; i < st.length(); ++i ){
    c= st.charAt( i );
    stage(c);
  }    
}
void stage_num( int val ){   // send number in ascii 
  char buf[12];
  char c;
  int i;
  itoa( val, buf, 10 );
  i= 0;
  while( c = buf[i++] ) stage(c);  
}

void Poll_Cat() {
  static String command = "";
  String lcommand;
  char c;
  int rit;

  if (Serial.available() == 0) return;

  while( Serial.available() ){
    c = Serial.read();
    command += c;
    if( c == ';' ) break;
  }

  if( c != ';' ) { 
    terminal = 0; 
    CatStatus = "C"; 
    return; 
  }   // command not complete yet but need to switch of terminal

  lcommand = command.substring(0,2);

  if( command.substring(2,3) == ";" || command.substring(2,4) == "$;" || command.substring(0,2) == "RV" ){      /* it is a get command */
    stage_str(lcommand);    // echo the command 
    if( command.substring(2,3) == "$") stage('$');

    if (lcommand == "IF") {
      RX_frequency = frequency_tune - IF;
      stage_str("000");
      if( RX_frequency < 10000000 ) stage('0');
      stage_num(RX_frequency);  
      stage_str("     ");
      rit= RitFreqOffset;
      if( rit >= 0 ) stage_str("+0");
      else{
        stage_str("-0"); 
        rit = - rit;
      }
      if( rit < 100 ) stage('0');
      if( rit < 10 ) stage('0');                                  // IF[f]*****+yyyyrx*00tmvspbd1*;
      stage_num(rit);
      stage_str("10 0003000001");                                 // rit,xit,xmit,cw mode fixed filed 
    }
    else if(lcommand == "FA") {                                   // VFO A
      stage_str("000"); 
      if( frequency_tune -IF < 10000000 ) stage('0');  
      stage_num(frequency_tune - IF);  
    } 
    else if(lcommand == "KS") stage_num(CWSpeed);                // KEYER SPEED
    else if(lcommand == "FW") stage_str("0000") , stage_num(Selected_BW+1);
    else if(lcommand == "MD") { 
      if (mode==0 && bsm==0) stage('1'); 
      if (mode==0 && bsm==1) stage('2'); 
      if (mode==1) stage ('3'); 
    }                        // Mode       
    else if(lcommand == "RV" && command.substring(2,3) == "F"){  // battery voltage in Front field 
      stage(command.charAt(2));
      double value = analogRead(BatteryReadPin)*1.7/100;
      int left_part, right_part;
      char buffer[50];
      sprintf(buffer, "%lf", value);
      sscanf(buffer, "%d.%1d", &left_part, &right_part);
      stage(' ');
      stage_num(left_part);
      stage('.');
      stage_num(right_part);
      stage(' ');
    }
    else if(lcommand == "RV" && command.substring(2,3) == "A"){  // Rebel Alliance Mod version in Aux: field
      stage(command.charAt(2));
      stage_str(txt0);
    }
    else if(lcommand == "RV" && command.substring(2,3) == "D"){  // Rebel Alliance Mod in DSP: field   
      stage(command.charAt(2));
      stage_str(txt2);
    }
    else if(lcommand == "RV" && command.substring(2,3) == "M"){  // Keyer Speed in MCU: field
      stage(command.charAt(2));
      stage_num(CWSpeed);
    }
    else if(lcommand == "RV" && command.substring(2,3) == "R"){  // Keyer Speed in MCU: field
      stage(command.charAt(2));
      stage_str(String(Frequency_Step, DEC));
    }
    else if(lcommand == "SM"){
      stage_str("00");
      SmeterReadValue = analogRead(SmeterReadPin);
      SmeterReadValue = map(SmeterReadValue, 1023, 0,41,0);
      if( SmeterReadValue < 10 ) stage('0');
      stage_num(SmeterReadValue);
    }   
    else {
      stage('0');   // send back nill command not know / used
    }
    stage(';');       // response terminator 
  }

  else  {
  } 
  set_cat(lcommand,command);    // else it's a set command 

    command = "";   // clear for next command
}

void set_cat(String lcom, String com ){
  long value;
  int split =0 ;

  if( lcom == "FA" ){    // set vfo freq 
    value = com.substring(2,13).toInt(); 
    if ( (value > Band_bsm0_Low && value < Band_bsm0_High) || (value > Band_bsm1_Low && value < Band_bsm1_High)  ) {
      // valid frequnecy according band configuration?
      if ( (value > Band_bsm0_Low && value < Band_bsm0_High) && bsm == 1) {
        // need to change band?
        bsm = 0;
        Change_Band();
      }
      if ( (value > Band_bsm1_Low && value < Band_bsm1_High) && bsm == 0) {
        // need to change band?
        bsm = 1;
        Change_Band();
      }   
      //if( lcom == "FB" || split == 0 ) frequency = value - IF;
      if( lcom == "FA" && ( value > 1800000 && value < 30000000) ) frequency = value + IF;
      //frequency_tune = value + IF;
      //UpdateFreq(frequency_tune);
    }
  }
  else if( lcom == "FW" ){             // xtal filter select
    value = com.charAt(6) - '0';
    if( value < 4 && value != 0 ){
      if ( value == 1) {  
        b = 0x00; // Hardware control of I.F. filter shape
        Selected_BW = Wide_BW;  // GPA7(28)LOW_GPA6(27)LOW wide
      }
      if ( value == 2) {
        b = 0x40; // Hardware control of I.F. filter shape
        Selected_BW = Medium_BW;  //  GPA7(28)LOW_GPA6(27)HIGH medium
      }
      if ( value == 3) {
        b = 0x80; // Hardware control of I.F. filter shape
        Selected_BW = Narrow_BW;   //  GPA7(28)HIGH_GPA6(27)LOW narrow
      }
      Select_Multi_BW_Ored();                     // original b is sent to port expander
    }
  }
  else if ( lcom == "MD" ){
    value = com.charAt(2) - '0';
    if ( value == 1 || value == 2) { 
      mode=0; 
      frequency_old=0; 
      UpdateFreq(frequency_tune); 
      Splash_MODE(); 
    }
    if ( value == 3) {  
      mode=1; 
      frequency_old=0; 
      UpdateFreq(frequency_tune); 
      Splash_MODE(); 
    }
  }
}


//------------------------------------------------------------------------
//--------------------------- Encoder Routine --------------------- 
// from Patriot Alliance mod beta 5 code by KD0UTH
void Frequency_down() {
  frequency = frequency - Frequency_Step;
  //Step_Flash();
  if ( bsm == 1 ) { 
    Band_20_Limit(); 
  }
  else if ( bsm == 0 ) { 
    Band_40_Limit(); 
  }
}
void Frequency_up() {
  frequency = frequency + Frequency_Step;
  //Step_Flash();
  if ( bsm == 1 ) { 
    Band_20_Limit(); 
  }
  else if ( bsm == 0 ) { 
    Band_40_Limit(); 
  }
}  

void Encoder()  // fast encoder routine
{  
  n = digitalRead(encoder0PinA);
  o = digitalRead(encoder0PinB);

  if ((encoder0PinALast == LOW) && (n == HIGH)) 
  {
    if (o == LOW) 
    {
      Frequency_down();    //encoder0Pos--;

    } 
    else 
    {
      Frequency_up();       //encoder0Pos++;
    }
  } 
  else if ((encoder0PinALast == HIGH) && (n == LOW))  
  {                                                     // now we get 36 steps/rev instead of 10
    if (o == HIGH)
    {
      Frequency_down();
    } 
    else
    {
      Frequency_up();
    }
  } 
  else if ((encoder0PinBLast == LOW) && (o == HIGH))
  {
    if (n == LOW)
    {
      Frequency_up();
    } 
    else
    {
      Frequency_down();
    }
  } 
  else if ((encoder0PinBLast == HIGH) && (o == LOW))
  {
    if (n == HIGH)
    {
      Frequency_up();
    } 
    else
    {
      Frequency_down();
    }
  }
  encoder0PinALast = n;
  encoder0PinBLast = o;
}

//-------------------------------------------------------
//-------------------------------------------------------
// Copied/modified from Patriot Alliance mod beta 5
void Change_Band()
{
  if ( bsm == 1 )                              //  select 40 or 20 meters 1 for 20 0 for 40
  { 
    digitalWrite(Band_Select, HIGH);
    IF = -IF_value;
    offset = -DDS_offset_value;
  }
  else 
  { 
    digitalWrite(Band_Select, LOW);
    IF = IF_value;
    offset = DDS_offset_value;
  }
  Band_Set_40_20M();
}

//------------------ Band Select ------------------------------------
// Modifed from Patriot Alliance mod beta 5 for changes by W8CQD
void Band_Set_40_20M()
{
  if ( old_bsm != bsm)                          //  this helps 5/13/14
  {
    if ( bsm == 1 )                             //  select 40 or 20 meters 1 for 20 0 for 40
    { 
      frequency_40 = frequency;                 //  save and restore memorized frequency
      frequency_default = frequency_20;
    }
    else 
    { 
      frequency_20 = frequency;                 //  save and restore memorized frequency
      frequency_default = frequency_40; 
    }
    splash_Clear_Freq(); //W8CQD clear the transmit frequency on band change
    Default_frequency();
  }
  old_bsm = bsm;                              //  this helps 5/13/14
}

//--------------------Default Frequency-----------------------------
void Default_frequency()
{
  frequency = frequency_default;
  UpdateFreq(frequency);
  splash_RX_freq(); 
}   //  end   Default_frequency

//-----------------------------------------------------
//-----------------------------------------------------
void  Band_40_Limit()
{
  if ( frequency >= 16.3e6 )
  { 
    frequency = 16.3e6;
    stop_led_on();    
  }
  else if ( frequency <= 16.0e6 )  
  { 
    frequency = 16.0e6;
    stop_led_on();
  }
  else { 
    stop_led_off(); 
  }
}
//-----------------------------------------------------  
void  Band_20_Limit()
{
  if ( frequency >= 5.35e6 )
  { 
    frequency = 5.35e6;
    stop_led_on();    
  }
  else if ( frequency <= 5.0e6 )  
  { 
    frequency = 5.0e6;
    stop_led_on();
  } 
  else { 
    stop_led_off(); 
  }
}

//-----------------------------------------------------  
void Step_Flash()
{
  stop_led_on();
  for (int i=0; i <= 25e3; i++); // short delay 
  stop_led_off();   
}

//-----------------------------------------------------
void stop_led_on()  //  band edge and flash
{
  digitalWrite(Band_End_Flash_led, HIGH);
}

//-----------------------------------------------------
void stop_led_off()
{
  digitalWrite(Band_End_Flash_led, LOW);
}

//===================================================================
//===================================================================
// Copied from Patriot Alliance mod beta 5
void Multi_Function() //  pushbutton for BW, Step, Other
{
  // look into a skip rtoutine for this
  Step_Multi_Function_Button = digitalRead(Multi_Function_Button);
  if (Step_Multi_Function_Button == HIGH) 
  {   
    unsigned long time;
    unsigned long start_time;
    unsigned long long_time;
    long_time = millis();

    time = millis();
    while( digitalRead(Multi_Function_Button) == HIGH ){ 

      // function button is pressed longer then 0.5 seconds
      if ( (millis() - long_time) > 500 ) { 
        // tune
        old_mode=mode;
        mode=1; //cw
        TX_on();
        // wait for button release
        while( digitalRead(Multi_Function_Button) == HIGH ){ 
        }   
        TX_off();
        mode=old_mode;
        Splash_MODE();
        return;        
      } 
      start_time = time;
      while( (time - start_time) < 7) {
        time = millis();
      }
    } // Debounce end
    while( digitalRead(Multi_Function_Button) == HIGH ){ 
    }  // added for testing
    for (int i=0; i <= 150e3; i++); // short delay

    Step_Multi_Function_Button1 = Step_Multi_Function_Button1++;
    if (Step_Multi_Function_Button1 > 2 ) 
    { 
      Step_Multi_Function_Button1 = 0; 
    }
  }
  Step_Function();
}  // end Multi_Function()
//==================================================================
//================================================================== 
void Step_Function()
{
  switch ( Step_Multi_Function_Button1 )
  {
  case 0:
    m = 0x08;   // GPA3(24) Controls Function Green led
    Select_Multi_BW_Ored();
    Step_Select_Button1 = Selected_BW; // 
    Step_Select(); //
    Selection();
    for (int i=0; i <= 255; i++); // short delay
    break;   //

  case 1:
    m = 0x10;   // GPA4(25) Controls Function Yellow led
    Select_Multi_BW_Ored();
    Step_Select_Button1 = Selected_Step; //
    Step_Select(); //
    Selection();
    for (int i=0; i <= 255; i++); // short delay
    break;   //

  case 2: 
    m = 0x20;   // GPA5(26) Controls Function Red led
    Select_Multi_BW_Ored();
    Step_Select_Button1 = Selected_Other; //
    Step_Select(); //
    Selection();
    for (int i=0; i <= 255; i++); // short delay
    break;   //  
  }
}  // end Step_Function()

//===================================================================
void  Selection()
{
  Step_Select_Button = digitalRead(Select_Button);
  if (Step_Select_Button == HIGH) 
  {   
    while( digitalRead(Select_Button) == HIGH ){ 
    }  // added for testing
    for (int i=0; i <= 150e3; i++); // short delay

    Step_Select_Button1 = Step_Select_Button1++;
    if (Step_Select_Button1 > 2 ) 
    { 
      Step_Select_Button1 = 0; 
    }
  }
  Step_Select(); 
}  // end Selection()

//-----------------------------------------------------------------------  
void Step_Select()
{
  switch ( Step_Select_Button1 )
  {
  case 0: //   Select_Green   
    s = 0x01;  // GPA0(21) Controls Selection Green led 
    if (Step_Multi_Function_Button1 == 0)
    {
      b = 0x00; // Hardware control of I.F. filter shape
      Selected_BW = Wide_BW;  // GPA7(28)LOW_GPA6(27)LOW wide
    } 
    else if (Step_Multi_Function_Button1 == 1)
    {
      Frequency_Step = 100;   //  Can change this whatever step size one wants
      Selected_Step = Step_100_Hz; 
    } 
    else if (Step_Multi_Function_Button1 == 2)
    {
      bsm = 0;
      Change_Band();
      Encoder();
      Selected_Other = Other_1_user; 
      // Other_1();
    } 
    for (int i=0; i <= 255; i++); // short delay  
    break;

  case 1: //   Select_Yellow  
    s = 0x02;    //  GPA1(22) Controls Selection Green led
    if (Step_Multi_Function_Button1 == 0) 
    {
      b = 0x40; // Hardware control of I.F. filter shape
      Selected_BW = Medium_BW;  //  GPA7(28)LOW_GPA6(27)HIGH medium
    } 
    else if (Step_Multi_Function_Button1 == 1) 
    {
      Frequency_Step = 1e3;   //  Can change this whatever step size one wants
      Selected_Step = Step_1000_hz; 
    }
    else if (Step_Multi_Function_Button1 == 2) 
    {
      bsm = 1;
      Change_Band();
      Encoder();
      Selected_Other = Other_2_user; 

      //   Other_2(); 
    }
    for (int i=0; i <= 255; i++); // short delay   
    break; 

  case 2: //   Select_Red    
    s = 0x04;    //  GPA2(23) Controls Selection Green led
    if (Step_Multi_Function_Button1 == 0) 
    {
      b = 0x80; // Hardware control of I.F. filter shape
      Selected_BW = Narrow_BW;   //  GPA7(28)HIGH_GPA6(27)LOW narrow
    } 
    else if (Step_Multi_Function_Button1 == 1) 
    {
      Frequency_Step = 10e3;    //  Can change this whatever step size one wants
      Selected_Step = Step_10000_hz;
    }
    else if (Step_Multi_Function_Button1 == 2) 
    {
      Selected_Other = Other_3_user; 

      //       Other_3(); 
    }
    for (int i=0; i <= 255; i++); // short delay
    break;     
  }
  Select_Multi_BW_Ored();
  Splash_Step_Size();
  Splash_BW();
}  // end Step_Select()

//----------------------------------------------------------------------------------
void Select_Multi_BW_Ored()
{
  t = s | m | b ;    // or'ed bits

  Wire.beginTransmission(0x20);
  Wire.send(0x12); // GPIOA
  Wire.send(t); // port A  result of s, m, b
  Wire.endTransmission(); 

}  // end  Select_Multi_BW_Ored()

//-----------------------------------------------------------------------------
// ****************  Dont bother the code below  ******************************
// \/  \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
//-----------------------------------------------------------------------------
void program_freq0(long frequency)
{
  AD9834_reset_high();  
  int flow,fhigh;
  fcalc = frequency*(268.435456e6 / Reference );    // 2^28 =
  flow = fcalc&0x3fff;              //  49.99975mhz  
  fhigh = (fcalc>>14)&0x3fff;
  digitalWrite(FSYNC_BIT, LOW);  //
  clock_data_to_ad9834(flow|AD9834_FREQ0_REGISTER_SELECT_BIT);
  clock_data_to_ad9834(fhigh|AD9834_FREQ0_REGISTER_SELECT_BIT);
  digitalWrite(FSYNC_BIT, HIGH);
  AD9834_reset_low();
}    // end   program_freq0

//------------------------------------------------------------------------------

void program_freq1(long frequency_TX)
{
  AD9834_reset_high(); 
  int flow,fhigh;
  fcalc = frequency*(268.435456e6 / Reference );    // 2^28 =
  flow = fcalc&0x3fff;              //  use for 49.99975mhz   
  fhigh = (fcalc>>14)&0x3fff;
  digitalWrite(FSYNC_BIT, LOW);  
  clock_data_to_ad9834(flow|AD9834_FREQ1_REGISTER_SELECT_BIT);
  clock_data_to_ad9834(fhigh|AD9834_FREQ1_REGISTER_SELECT_BIT);
  digitalWrite(FSYNC_BIT, HIGH);  
  AD9834_reset_low();
}  

//------------------------------------------------------------------------------
void clock_data_to_ad9834(unsigned int data_word)
{
  char bcount;
  unsigned int iData;
  iData=data_word;
  digitalWrite(SCLK_BIT, HIGH);  //portb.SCLK_BIT = 1;  
  // make sure clock high - only chnage data when high
  for(bcount=0;bcount<16;bcount++)
  {
    if((iData & 0x8000)) digitalWrite(SDATA_BIT, HIGH);  //portb.SDATA_BIT = 1; 
    // test and set data bits
    else  digitalWrite(SDATA_BIT, LOW);  
    digitalWrite(SCLK_BIT, LOW);  
    digitalWrite(SCLK_BIT, HIGH);     
    // set clock high - only change data when high
    iData = iData<<1; // shift the word 1 bit to the left
  }  // end for
}      // end  clock_data_to_ad9834

//-----------------------------------------------------------------------------
void AD9834_init()      // set up registers
{
  AD9834_reset_high(); 
  digitalWrite(FSYNC_BIT, LOW);
  clock_data_to_ad9834(0x2300);  // Reset goes high to 0 the registers and enable the output to mid scale.
  clock_data_to_ad9834((FREQ0_INIT_VALUE&0x3fff)|AD9834_FREQ0_REGISTER_SELECT_BIT);
  clock_data_to_ad9834(((FREQ0_INIT_VALUE>>14)&0x3fff)|AD9834_FREQ0_REGISTER_SELECT_BIT);
  clock_data_to_ad9834(0x2200); // reset goes low to enable the output.
  AD9834_reset_low();
  digitalWrite(FSYNC_BIT, HIGH);  
}  //  end   AD9834_init()

//----------------------------------------------------------------------------   
void AD9834_reset()
{
  digitalWrite(RESET_BIT, HIGH);  // hardware connection
  for (int i=0; i <= 2048; i++);  // small delay

  digitalWrite(RESET_BIT, LOW);   // hardware connection
}  // end AD9834_reset()

//-----------------------------------------------------------------------------
void AD9834_reset_low()
{
  digitalWrite(RESET_BIT, LOW);
}  // end AD9834_reset_low()

//..............................................................................     
void AD9834_reset_high()
{  
  digitalWrite(RESET_BIT, HIGH);
}  // end  AD9834_reset_high()

//^^^^^^^^^^^^^^^^^^^^^^^^^  DON'T BOTHER CODE ABOVE  ^^^^^^^^^^^^^^^^^^^^^^^^^ 
//=============================================================================

//------------------------Display Stuff below-----------------------------------

// All the code for display below seems to work well. 6-18-14

//------------------- Splash RIT -----------------------------------------------  
void splash_RIT()
{ 
  if ( old_RitFreqOffset != RitFreqOffset) // only if RIT changes
  { 
    lcd.setCursor(16, 0);
    lcd.print(txt70);                       // spaces

    lcd.setCursor(16, 0); 
    stringRIT = String( RitFreqOffset, DEC);

    lcd.print(stringRIT);
  }
  old_RitFreqOffset = RitFreqOffset;      // test for Rit change
}

void splash_Smeter() //was writeSmeter() id WD9GYM's code, renamed for consistency
{     
  smeterTime = millis(); // grab current time
  smeterInterval = smeterTime - smeterLast; // calculate interval between this and last event

    if (smeterInterval > 200) // ignore checks less than 200mS after initial edge
  {

    SmeterReadValue = analogRead(SmeterReadPin);   // read value of signal 0 - 1023
    level = map(SmeterReadValue, 1023, 0,41,0);    // constrain the value into db 0 - 40


      lcd.setCursor(13, 2);                      // print the s meter on line 3
    lcd.print("       ");                       // blank out bars

    if (level > 30) level = 40;
    if (level > 20 && level < 31) level = 30;
    if (level > 10 && level < 21) level = 20;
    if (level > 7 && level < 10) level = 9;
    if (level > 5 && level < 8) level = 7;
    if (level > 3 && level < 6) level = 5;
    if (level > 1 && level < 4) level = 3;
    if (level > 0 && level < 2) level = 1;
    Serial.print(" level = "); 
    Serial.println(level);   
    switch( level ){                           // write each bar required
    case 40:                                 // do not put break between case statements
      lcd.createChar(4, meter_s30);         // let the code fall through
      lcd.setCursor(18,2); 
      lcd.write( 4 );
    case 30: 
      lcd.createChar(4, meter_s30);
    case 20: 
      if( level == 20 ) lcd.createChar(4, meter_s20); 
      lcd.setCursor(17,2); 
      lcd.write( 4 );
    case 10: 
      lcd.createChar(3, meter_s10);
    case 9: 
      if( level == 9 ) lcd.createChar(3, meter_s9); 
      lcd.setCursor(16,2); 
      lcd.write( 3 );
    case 7: 
      lcd.createChar(2, meter_s7);
    case 5: 
      if( level == 5 ) lcd.createChar(2, meter_s5); 
      lcd.setCursor(15,2); 
      lcd.write( 2 );
    case 3: 
      lcd.createChar(1, meter_s3);
    case 1: 
      if( level == 1 ) lcd.createChar(1, meter_s1); 
      lcd.setCursor(14,2); 
      lcd.write( 1 );
    case 0:
    default: 
      break;
    }
    smeterLast = smeterTime; // set up for next event
  }
}

//------------------------------------------------------------------------------
void splash_TX_freq()
{
  long TXD_frequency;  // ADDED 6-18-14 OK

  if ( bsm == 1 )                      // test for 20M
  { 
    TXD_frequency = frequency_tune ; 
  }

  else if ( bsm == 0 )                  // test for 40M
  { 
    TXD_frequency = frequency_tune ; 
  } 
  //---------------------------------------------------

  if ( TXD_frequency < 5.36e6 )
  { 
    TXD_frequency = TXD_frequency + 9e6; 
  }

  else if ( TXD_frequency > 15.95e6 )
  { 
    TXD_frequency = TXD_frequency - 9e6; 
  } 
  //--------------------------------------------------

  //stringFREQ = String(TXD_frequency / 10, DEC); - replaced by W8CQD, not sure what this was trying to do
  lcd.setCursor(3, 1);
  stringFREQ = String(TXD_frequency , DEC);
  lcd.print(stringFREQ);

}
//------------------------------------------------------------------------------
void splash_RX_freq()
{
  long RXD_frequency; // ADDED 6-18-14 OK

  if ( old_frequency_tune != frequency_tune )
  {
    if ( bsm == 1 )                      // test for 20M
    { 
      RXD_frequency = frequency_tune ; 
    }

    else if ( bsm == 0 )                 // test for 40M
    { 
      RXD_frequency = frequency_tune ; 
    }
    //-------------------------------------------

    if ( RXD_frequency < 5.36e6 )
    { 
      RXD_frequency = RXD_frequency + 9e6; 
    }

    else if ( RXD_frequency > 15.95e6 )
    { 
      RXD_frequency = RXD_frequency - 9e6; 
    }
    //--------------------------------------------

    lcd.setCursor(3, 0);
    lcd.print(txt72);                       // spaces

    lcd.setCursor(3, 0);
    stringFREQ = String(RXD_frequency , DEC);
    lcd.print(stringFREQ);
  }
  old_frequency_tune = frequency_tune;
}
//-----------------------------------------------------------------
void splash_Clear_Freq()
{
  lcd.setCursor(3, 0);
  lcd.print(txt72);
  lcd.setCursor(3, 1);
  lcd.print(txt72);

}
//-----------------------------------------------------------------
//void Splash_Band()
//{
//  if ( bsm == 1 )               // test for 20M
//  {
//    lcd.setCursor(17, 1);
//    lcd.print(txt66);        // 20 meters
//  }
//  else                         
//  {
//    lcd.setCursor(17, 1);
//    lcd.print(txt67);        // 40 meters
//  } 
//}   

//---------------------Display Output Power----------------------------------------
//   Added by W8CQD
void Splash_PWR()
{
  powerTime = millis(); //Grab Current Time
  powerInterval = powerTime - powerLast; //Calculate interval between this and last event
  if (powerInterval>100 or powerLast == 0) { //update every 100 ms
    PowerOutReadValue = analogRead(PowerOutReadPin);
    PowerOutReadValue = map(PowerOutReadValue, 0, 1023, 0, 14);
    lcd.setCursor(16, 1);
    lcd.print(txt70);
    lcd.setCursor(16,1);
    lcd.print(PowerOutReadValue);
    powerLast = powerTime;
  }
}   

//---------------------Display Battery Level----------------------------------------
//   Added by W8CQD
void Splash_Volts()
{
  voltsTime = millis(); // Grab Current Time
  voltsInterval = voltsTime - voltsLast; // Calculate interval between this and last event
  if (voltsInterval>30000 or voltsLast == 0) { // update every minute
    BatteryReadValue = analogRead(BatteryReadPin);
    floatVolts = (BatteryReadValue*1.7/100);
    lcd.setCursor(7, 2);
    lcd.print(txt71);
    lcd.setCursor(5,2);
    lcd.print(floatVolts, 2);
    voltsLast = voltsTime; //set up for next event
  }
}   

//---------------------Display TX/RX Status----------------------------------------
//    Added by W8CQD
void Splash_RXTX()
{
  if (status_RXTX != old_status_RXTX) {
    if (status_RXTX == 1) {
      lcd.setCursor(13,3);
      lcd.print(txt177);
      lcd.setCursor(13,3);
      lcd.print(txt63); // TX
    }
    else {
      lcd.setCursor(13,3);
      lcd.print(txt177);
      lcd.setCursor(13,3);
      lcd.print(txt62); // RX
    }
    old_status_RXTX = status_RXTX;
  }
}
//---------------------------------------------------------------------------------

void Splash_Step_Size()
{
  if ( old_Frequency_Step != Frequency_Step ) // 
  { 
    lcd.setCursor(7, 3);
    lcd.print(txt71);                       // spaces

    lcd.setCursor(7, 3); 

    string_Frequency_Step = String(Frequency_Step, DEC);
    lcd.print(string_Frequency_Step);
  }
  old_Frequency_Step = Frequency_Step;      // test for Rit change 
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
void Splash_BW()
{
  if ( old_b != b )
  {
    if ( b == 0x00 )
    { 
      lcd.setCursor(0, 3);
      lcd.print(txt170);
      lcd.setCursor(0, 3);
      lcd.print(txt140);  // wide
    }
    else if ( b == 0x40 )
    { 
      lcd.setCursor(0, 3);
      lcd.print(txt170);
      lcd.setCursor(0, 3);
      lcd.print(txt150);  // medium
    }
    else {
      lcd.setCursor(0, 3);
      lcd.print(txt170);
      lcd.setCursor(0, 3);
      lcd.print(txt160);  // narrow
    }
  }
  old_b = b ;
}

//--------------------------------------------------------------------
void Splash_MODE()
{
  if ( old_PTT_SSB_Key != PTT_SSB_Key )
  { 
    if ( PTT_SSB_Key == LOW )
    {
      lcd.setCursor(17, 3);
      lcd.print(txt69);
      lcd.setCursor(17, 3);
      lcd.print(txt135);    // SSB
    }
    else                         
    {
      lcd.setCursor(17, 3);
      lcd.print(txt69);
      lcd.setCursor(17, 3);
      lcd.print(txt132);    // CW
    }
  }
  old_PTT_SSB_Key = PTT_SSB_Key;
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//stuff above is for testing using the Display Comment out if not needed  
//-----------------------------------------------------------------------------
uint32_t TimerOverFlow(uint32_t currentTime)
{
  return (currentTime + CORE_TICK_RATE*(1));//the Core Tick Rate is 1ms
}

//------------------ Debug data output ------------------------------

void    serialDump()
{
  loopStartTime   = millis();
  loopsPerSecond  = loopCount - lastLoopCount;
  loopSpeed       = (float)1e6 / loopsPerSecond;
  lastLoopCount   = loopCount;

  Serial.print    ( "uptime: " );
  Serial.print    ( ++printCount );
  Serial.println  ( " seconds" );

  Serial.print    ( "loops per second:    " );
  Serial.println  ( loopsPerSecond );
  Serial.print    ( "loop execution time: " );
  Serial.print    ( loopSpeed, 3 );
  Serial.println  ( " uS" );

  Serial.print    ( "Freq Rx: " );

  Serial.println  ( frequency_tune - IF );
  Serial.println  ( RX_frequency );

  Serial.print    ( "Freq Tx: " );

  Serial.println  ( frequency - IF );
  Serial.println  ( TX_frequency );

  Serial.print    ( "RIT: " );
  Serial.println  (  RitFreqOffset );

  Serial.print    ( "BW: " );
  if ( b == 0x00 )
  { 
    Serial.println(txt140);  // wide
  }
  else if ( b == 0x40 )
  { 
    Serial.println(txt150);  // medium
  }
  else {
    Serial.println(txt160);  // narrow
  }

  Serial.print    ( "BAND: " );
  if ( bsm == 1 )               // test for 20M
  {
    Serial.println(txt66);        // 20 meters
  }
  else                         
  {
    Serial.println(txt67);        // 40 meters
  }

  Serial.print    ( "MODE: " );
  if ( PTT_SSB_Key == LOW )
  {
    Serial.println(txt135);    // SSB
    Serial.println(PTT_SSB_Key);
  }
  else                         
  {
    Serial.println(txt132);    // CW
    Serial.println(PTT_SSB_Key);
  }
  Serial.println  ( );

  Serial.print    ( "STEP: " );
  Serial.println  (  Frequency_Step );

  Serial.print("S level = "); 
  Serial.print(level);

  Serial.println  ();

} // end serialDump()




