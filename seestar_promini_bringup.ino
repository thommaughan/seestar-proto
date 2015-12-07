
//############### SEESTAR CAMERA SYSTEM LICENSE ################################
//#
//#  This work is distributed under a Creative Commons Attribution-ShareAlike 4.0 license.
//#  Derivative works should be shared under the same license:
//#    http://creativecommons.org/licenses/by-sa/4.0
//#
//#  See http://bitbucket.org/mbari/seestar for contact information and source
//#
//#  Citation: F. Cazenave, C. Kecy, M. Risi, S.H.D. Haddock (in press)
//#     "SeeStar: a low-cost, modular, and open-source camera system for subsea observations",
//#     IEEE Oceans 2014
//#
//#  Arduino Coder: Thom Maughan is just a bit player (coder and board bringup)
//#               in this project
//#
//##############################################################################

// SeeStar v2 TimeLapse Arduino Controller Code for GoPro Hero3/3+
// This is the Hardware Bringup code - type 'm' for easy bringup, 'menu' for seestar menu
// Author: Thom Maughan
// Date:   7 Dec 2015
// Version: 0.9 (Alpha)
// Requires: seestar2_lib

// Features:
// Works with Arduino Pro Mini AT328 3.3v 8MHz
// Uses MBARI open source hardware shield for GoPro<>Arduio hardware interface
// DS3234 low drift Real Time Clock
// Low Power Sleep with RTC wakeup interrupt (disabled in bringup)
// Uses GoPro interface connnector
// LED Flashlight

// Operation:
//    Connect GoPro, Battery power, and Light connections per the hardware instructionis
//    Connect FTDI cable to ProMini with SeeStar Shield attached, 9600 baud.   Insert jumper J4 in Shield
//    Download this sketch (seestar_promini_bringup.ino)
//    Bring up serial console
//    Type MENU to see the commands, Note: the 'm' bringup commands
//    DEPLOYDUR=12   sets the deployment duration to 12 hours
//    VIDEODUR=0     sets the video duration to zero, this setting is used for image timelapse (note: ash script needs to be loaded in GoPro)
//    INTERVAL=0     sets the timelapse interval
//    DELAYSTART=15  startup delay of 15 seconds before timelapse begins
//    START          starts the intervalometer (after startup delay), writes StartFlg to EEPROM for powerup autostart
//    STOP           stops the intervalometer, clears StartFlg in EEPROM

// Status of Code:  Board Bringup version
//    RTC Interrupt (10 sec) does not begin right away, takes 30 secs or so, this also after taking a picture (powerup...)
//    type start after deployment and delay should be ignored (waveglider external power usage)
//    Low power code and wakeup alarm is a work in progress (needs some a day or two of work and serial port)
//    Serial interface (non FTDI) is not tested, likely a PCB assembly problem is preventing this
//    Remove Compiler switch and autodetect shield and enable/disable clock functions automatically.  If shield is not connected, clock functions hang



#include <avr/pgmspace.h>    // needed from PROGMEM strings to save SRAM
#include <SPI.h>
#include <EEPROM.h>
#include <util/atomic.h>

#include <atmega328pwr.h>    // seestar2_lib
#include <ds3234.h>          // seestar2_lib
#include <SerCmd.h>          // seestar2_lib

//uncomment this line to enable real time clock, NOTE: code will hang when it's just the arduino without shield
#define SHIELDPLUGGEDIN 1    // Comment the section below when not plugged into Shield


// Digital IO symbolic definitions (All are Outputs except those with 'IN' comments)
#define ARD_LED        13    // same as SCK (conflicts with DS3234 SPI bus)
#define RTC_INT_L      2     // IN  //WAKEUP_PIN = 2;
#define LIGHT_BUTTON   4     // was PIN4
#define RTC_RESET_L    5
#define UART_SHDN_L    6
#define LIGHT_POWER    7
#define CAM_ON         8
#define CAM_POWER      9
#define RTC_CS_L       10

// Analog INPUTs
#define BAT_VOLT    A0
#define BAT_CURR    A1


//------------------------------------------------
//
// global variable declarations
//
//------------------------------------------------

// EEPROM

// ID of the settings block
#define PROGRAM_VERSION "SeeStar 2 v1.0"
#define CONFIG_VERSION "1.0"
// Tell it where to store your config data in EEPROM
#define CONFIG_START 32    // Config starts 32 bytes from beginning of EEPROM

// Example settings structure
struct ConfigStruct
{
  // The variables of your settings
  unsigned long deployment_secs;    // timelapse duration in secs (user will enter in hours)
  unsigned long start_delay_secs;   // delay in sec before timelapse start
  unsigned long video_dur_secs;    // length of video in seconds
  unsigned long interval_secs;     // time between images/videos
  unsigned long bat_thresh_mv;
  unsigned int  startFlg;
  char version_of_config[4];

}
seeStarConfig =
{
  // The default Config values
  12000L,      // unsigned long deployment_secs;
  45L,        //  unsigned long start_delay_secs;
  0L,          // unsigned long video_dur_secs; set to 0 for image timelapse (no video)
  600L,        // unsigned long interval_secs;
  12500L,      // unsigned long bat_thresh_mv;  12.5v
  0,           // unsigned int startFlg
  CONFIG_VERSION   // config version
};


// Print statement debug control (code is verbose on print in this version)
int dbgCam = 1;   // print statements in powerup and take picture

unsigned int batCurrent = 0;
unsigned int batVoltage = 0;
unsigned int batVoltageLoad = 20000;
unsigned int batCurrentLoad = 0;

// Camera State Machine definitions and vars
#define ST_CAM_INIT            0   // stays in INIT until ready to shoot
#define ST_CAM_GO              1
#define ST_CAM_POWERUP_DELAY   2
#define ST_CAM_LIGHT_POWER     3
#define ST_CAM_ON_BUTTON       4
#define ST_CAM_OFF_BUTTON      5
#define ST_CAM_ON_TO_LIGHT_ON  6
#define ST_LIGHT_ON_BUTTON     7
#define ST_LIGHT_ON_DURATION   8
#define ST_CAM_OFF_DELAY       9

uint16_t camState = 0;
uint16_t camCycleCnt = 1;

// for camera statemachine delays
unsigned long cam_curMillis = 0;
unsigned long cam_prevMillis = 0;
unsigned long cam_intervalMillis = 1;

// for camera statemachine delays
unsigned long light_curMillis = 0;
unsigned long light_prevMillis = 0;
unsigned long light_intervalMillis = 1;



// port bit variables
unsigned int lightPowerState = 0;
unsigned int camPowerState = 0;

#define PRN_BUF_SIZE	80
char prnBuf[PRN_BUF_SIZE];

// DES3234 RAM
#define SRAM_SIZE 256		// size of SRAM in DS3234

// time variables
uint8_t time[8];
uint8_t sleep_period = 5;       // the sleep interval in minutes between 2 consecutive alarms
struct ts t;
unsigned long prev;


// Intervalometer stuff
#define ST_INIT                   0
#define ST_STARTUP_DELAY          1
#define ST_INTERVAL_INIT          2
#define ST_INTERVAL_START         3
#define ST_INTERVAL_WAITNEXT      4     // 
#define ST_DEPLOYMENT_COMPLETE    5
#define ST_SHUTDOWN               6

int intervalometerState = 0;

unsigned long ulTimeCount = 0L;

unsigned long startSeconds = 0L;
unsigned long deltaSeconds = 0L;
unsigned long deploymentStartSec = 0L;
unsigned long deploymentSeconds = 0L;

unsigned long cam_on_duration;

//unsigned long interval = 5000;		// milliseconds
unsigned long interval = 10000;		// milli-seconds, interval between photos / videos

unsigned int alarmSec = 0;            // for keeping track of programming
unsigned int alarmInterval = 10;	// 10 seconds


//do Not adjust MIN_INTERVAL (this is deterimined by how long it takes to power up and take an image and power down
#define MIN_INTERVAL  25000          // 25seconds

// MakerDemo: adjust INTERVAL to determine intervalometer time between photos (30000 = 30 sec)
#define INTERVAL      30000          // msec between pictures, NO less than MIN_INTERVAL

// interrupt service routine variable
volatile int rtc_intVal = 0;		// volatile since value is set in DS3224_wake interrupt service routine

// Serial
#define RXBUF_SIZE  10
byte rxFlg = 0;
byte rxByte;
byte rxBuf[RXBUF_SIZE + 1];

unsigned int rxCnt = 0;

SerCmd SCmd;        // requires #include <SerCmd.h


//----------------------------------------------------------
//
//     utility functions
//
//----------------------------------------------------------
// SERIAL PRINT that uses PROGMEM to save on ram space
// replace Serial.print("string") with SerialPrint("string")
#define SerialPrint(x) SerialPrint_P(PSTR(x), 0)
#define SerialPrintLn(x) SerialPrint_P(PSTR(x), 1)
void SerialPrint_P(PGM_P str, int lnFlg)
{
  for (uint8_t c; (c = pgm_read_byte(str)); str++)
  {
    Serial.write(c);
  }

  if (lnFlg == 1)
  {
    Serial.write(13);  // CR
    Serial.write(10);  // LF
  }

}




//----------------------------------------------------------
//
//     Interrupt Service Routines
//
//----------------------------------------------------------
// use watchdog for timed wakeup
ISR(WDT_vect)
{
  SleepDog::watchdogEvent();

}

// Interrupt Service Routine (ISR) for DS3234 RTC interrupt (active low)
void ds3234_wake(void)
{
  uint8_t reg_val;

  ulTimeCount++;          // counts the interrupts (once every 10 seconds)

  //DS3234_clear_a1f(RTC_INT_L);			// clear the interrupt by writing to the DS3234 status reg

  reg_val = DS3234_get_sreg(RTC_CS_L);
  reg_val &= B11111100;
  DS3234_set_sreg(RTC_CS_L, reg_val);

  rtc_intVal = 1;		// signal foreground that interrupt occurred

}  // end of ds3234_wake



//----------------------------------------------------------
//
//     Arduino Setup()
//
//----------------------------------------------------------
void setup()
{
  // put your setup code here, to run once:

  //pinMode(ARD_LED, OUTPUT);      //  = 13;  (port 13 is also SPI bus SCK
  pinMode(RTC_INT_L, INPUT);       //  = 2;
  pinMode(RTC_RESET_L, OUTPUT);    //  = 5;
  pinMode(UART_SHDN_L, OUTPUT);    //  = 6;
  pinMode(LIGHT_POWER, OUTPUT);    //  = 7;
  pinMode(CAM_ON, OUTPUT);       //  = 8;
  pinMode(CAM_POWER, OUTPUT);    //  = 9;
  pinMode(RTC_CS_L, OUTPUT);       //  = 10;

  // Note: dont use Arduino LED (pin 13) and DS34322 RTC
  //digitalWrite(ARD_LED, LOW);      // LED off
  digitalWrite(RTC_RESET_L, HIGH);  // active lo uart reset
  digitalWrite(UART_SHDN_L, HIGH);  // active lo uart reset (should be low for deployment to save power)
  digitalWrite(LIGHT_POWER, LOW);     // active hi
  digitalWrite(CAM_ON, LOW);        // active hi
  digitalWrite(CAM_POWER, LOW);     // active hi
  digitalWrite(RTC_CS_L, HIGH);     // active lo

  Serial.begin(9600);

  // See #define SHIELDPLUGGEDIN above
#ifdef SHIELDPLUGGEDIN
  // seestar_lib DS3234 init
  DS3234_init(RTC_CS_L, DS3234_INTCN);    // RTC_CS_L = 10

  // user interrupt 0, call ds3234 as isr handler, LOW is the only option for wakeup from IDLE/SLEEP
  attachInterrupt (0, ds3234_wake, LOW);  // attach interrupt handler for RTC_INT wakeup (interrupt when low)

  // read RTC and set alarmSec for 10 sec in future

  alarmInterval = 10;
  alarmSec = 0;
  set_alarm1_sec(alarmSec);
#endif

  // NOTE: define tokens in all caps, the user can type in any case due to strupr used on token in SerCmd.cpp
  SCmd.addCommand("HELP", menu);                    //:         help menu";
  SCmd.addCommand("MENU", menu);
  SCmd.addCommand("PARAM", get_param);              //:  get parameters";
  SCmd.addCommand("TIME", get_time);                //:  get time";
  SCmd.addCommand("SETTIME", set_time);             //:  set time";
  SCmd.addCommand("INTERVAL", set_interval);        //:  Set timelapse interval, time between taking image/video";
  SCmd.addCommand("VIDEODUR", set_videodur);
  SCmd.addCommand("DEPLOYDUR", set_deploydur);        //:  Deployment Duration of video in seconds";
  SCmd.addCommand("DELAYSTART", set_startdelay);    //:  Set Delay in seconds after START commmand before starting timelapse";
  SCmd.addCommand("STARTDELAY", set_startdelay);    //:  Set Delay in seconds after START commmand before starting timelapse";
  SCmd.addCommand("SETDEFAULT", set_default_config);
  SCmd.addCommand("SETUP", setup_seeStar);

  SCmd.addCommand("START", intervalometer_go);       //: Start intervalometer timelapse";    // was CamGo
  SCmd.addCommand("STOP",  intervalometer_stop);
  SCmd.addCommand("GETBAT", get_battery);            //: Get battery voltage and current";
  SCmd.addCommand("SETBAT", set_battery);            //: Get battery voltage and current";

  SCmd.addDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")


  loadConfig();

  while (!Serial);
  SerialPrintLn("SeeStar2, v1.0,  software by Thom Maughan");
  SerialPrintLn("Setup is Complete");


#ifndef SHIELDPLUGGEDIN
  SerialPrintLn("SHIELD disabled, #define SHIELDPLUGGEDIN 1 and recompile sketch" );
#endif

  autostart_handler();    // check if autostart is enabled (seeStarConfig.startFlg=1)

}  // end setup()


//----------------------------------------------------------
//
//     Arduino loop() - main loop function called repeatedly by the arduino framework, no blocking code
//
//----------------------------------------------------------
void loop()
{
  uint8_t reg_val;
  unsigned long now = millis();

  SCmd.readSerial();     // process serial commands
  serial_handler();      // process single letter commands (helpful for board testing)

  rtc_int_handler();

  battery_handler();    // check battery and if too low, stop timelapse and sleep
  
  //delay(100);      // DEBUG, wait for serial to complete it's transmit

  //SleepDog::delay_msec(100);  // sleep a little bit but still be responsive to serial commands

  //Serial.write('U');    // CHAD, uncomment this line to DEBUG serial transmit problem - it will send a stream of 0x55
  
  intervalometer_handler();
  
 // sleep_handler();  

#ifdef SHIELDPLUGGEDIN
  // show time once in a while (DEBUG code for event times)
  if ( (unsigned long) (now - prev) > interval )
  {
    DS3234_get(RTC_CS_L, &t);
    snprintf(prnBuf, PRN_BUF_SIZE, "%d.%02d.%02d %02d:%02d:%02d", t.year,
             t.mon, t.mday, t.hour, t.min, t.sec);
    Serial.println(prnBuf);
    prev = now;

  }
#endif

}

void sleep_handler(void)
{
     // check for activity, if none then
   //SleepDog::PowerDown();  
}

//----------------------------------------------------------
//
// foreground process for RTC interrupt, providing an RTC alarm based time tick for now
//
//----------------------------------------------------------
void rtc_int_handler(void)
{
  if (rtc_intVal == 1)
  {
    rtc_intVal = 0;			// isr variable from ds3234_wake (no atomic write protect needed - slow)
    Serial.println(get_seconds());    // DEBUG

    //alarmInterval = 10;  hardcoded alarm interval
    alarmSec += alarmInterval;
    if (alarmSec >= 60)
    {
      alarmSec = alarmSec % 60;
    }
    set_alarm1_sec(alarmSec);  // program alarm1 for next wakeup

    //SerialPrintLn("RTC interrupt happened");
    //    SerialPrintLn("RTC int");

  }
}

//----------------------------------------------------------
//
// Handle the power on a autostart of SeeStar
//
//----------------------------------------------------------
void autostart_handler(void)
{
  if(seeStarConfig.startFlg == 1)
  {    
    intervalometer_go();
  }
}


//----------------------------------------------------------
//
// battery management for stopping the intervalometer when batter is too low.  batVoltageLoad is set under full electrical load (a bit of a hack for now)
//
//----------------------------------------------------------
void battery_handler(void)
{
  // NOT DONE YET

  // perform the measurement while the camera is turned on, possible the light also and handle the condition here
  if (batVoltageLoad < seeStarConfig.bat_thresh_mv)
  {
    //Serial.print(batVoltageLoad);
    //SerialPrint("  ");
    //Serial.println(seeStarConfig.bat_thresh_mv);
  }



}

//----------------------------------------------------------
//
// SeeStar menu (Note: match the commands to addCommand in setup()
//
//----------------------------------------------------------
void menu(void)
{

  SerialPrintLn("HELP        Help Menu");
  SerialPrintLn("MENU        Help Menu");
  SerialPrintLn("PARAM       Get Parameters");
  SerialPrintLn("TIME        Get Time");
  SerialPrintLn("SETTIME     Set time:  SETIME YYYY.MM.DD HH:MM:SS");
  SerialPrintLn("INTERVAL    Set interval between images/videos in sec:  INTERVAL=60");
  SerialPrintLn("VIDEODUR    Set duration of video segments in sec:  VIDEODUR=0");
  SerialPrintLn("DEPLOYDUR   Set deployment duration in hours: DEPLOYDUR=240");
  SerialPrintLn("DELAYSTART  Set delay to start timelapse in sec:  DELAYSTART=15");
  SerialPrintLn("START       Start timelapse, set autostart after power cycle");
  SerialPrintLn("STOP        Stop timelapse, sets Start=0, no autostart");
  SerialPrintLn("GETBAT      Get battery voltage and current");
  SerialPrintLn("SETBAT      Set battery voltage threshold in mV, timelapse stops if below: SETBAT=12500");
  SerialPrintLn(" ");
  SerialPrintLn("m           Simple board test command menu");
  SerialPrintLn(" ");
  SerialPrintLn("___________________|-------|________________|-------|________________|--- / /-----|_");
  SerialPrintLn("<- startup delay ->");
  SerialPrintLn("                   |<------ interval ------>|<------ interval ------>|<---/ /---->|");
  SerialPrintLn("                   |<-Dur->|  VideoDur ");
  SerialPrintLn("                   |<------------- Deployment Duration -------------------/ /---->|");
  SerialPrintLn(" ");

}

//----------------------------------------------------------
//
// Board bringup
//
//----------------------------------------------------------
void light(void)
{
  SerialPrintLn("Light power toggle");
  if (lightPowerState == 0)
    light_power_on();
  else
    light_power_off();
}

//----------------------------------------------------------
//
// Example for processing commands with SerCmd (can be deleted)
//
//----------------------------------------------------------
void howdy(void)
{
  SerialPrintLn("Howdy");
}

//----------------------------------------------------------
//
// Example for processing commands with SerCmd (can be deleted)
//
//----------------------------------------------------------
void SayHello()
{
  char *arg;
  arg = SCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
  {
    Serial.print("Hello ");
    Serial.println(arg);
  }
  else {
    SerialPrintLn("Hello, whoever you are");
  }
}


//----------------------------------------------------------
//
// Example for processing commands with SerCmd (can be deleted)
//
//----------------------------------------------------------
void process_command()
{
  int aNumber;
  char *arg;

  Serial.println("We're in process_command");
  arg = SCmd.next();
  if (arg != NULL)
  {
    aNumber = atoi(arg);  // Converts a char string to an integer
    SerialPrint("First argument was: ");
    Serial.println(aNumber);
  }
  else {
    SerialPrintLn("No arguments");
  }

  arg = SCmd.next();
  if (arg != NULL)
  {
    aNumber = atol(arg);
    SerialPrint("Second argument was: ");
    Serial.println(aNumber);
  }
  else {
    SerialPrintLn("No second argument");
  }

}

//----------------------------------------------------------
//
// Set as the default handler for SerCmd, gets called when no other command matches.
//
//----------------------------------------------------------
void unrecognized()
{
  //  if (SCmd.getCharCmd() == 0)
  Serial.println("What?");
}



//get_DS3234_alarm1
//get_DS3234_alarm2
//set_DS3234_alarm1
//set_DS3234_alarm2
//get_DS3234_aging_reg
//reset_DS3234_aging_reg
//get_DS3234_sram
//get_DS3234_temperature
//reset_DS3234_status_reg_alarm
//get_DS3234_status_reg
//init_rtc
//get_time


//----------------------------------------------------------
//
// handle single character commands, useful for board bringup, SerCmd will say what?
//
//----------------------------------------------------------
int serial_handler(void)
{
  unsigned int alarmSec = 5;
  // send data only when you receive data:
  //rxByte = 0;

  //if (Serial.available() > 0)
  rxByte = SCmd.getCharCmd();

  if (rxByte > 0)
  {
    //Serial.println(rxByte, DEC);   // DEBUG
    //rxByte = Serial.read();   // use this when not using SerCmd library

    switch (rxByte)
    {
      case 'm':
      case 'h':
        //      SerialPrintLn("1 - Alarm 1Hz");
        //      SerialPrintLn("2 - Alarm generic");
        //      SerialPrintLn("3 - Alarm Seconds ");
        //      SerialPrintLn("4 - Light toggle");

        SerialPrintLn("5 - Light power toggle");
        SerialPrintLn("6 - Light button");

        SerialPrintLn("7 - Camera power toggle");
        SerialPrintLn("8 - Cam button");



        SerialPrintLn("m - menu");
        SerialPrintLn("c - Save Config");
        SerialPrintLn("d - Load Config");


        //        SerialPrintLn("h - menu");
        SerialPrintLn("s 2015.12.25 5 14:25:07 - set time YYYY MM DD W HH MM SS");
        SerialPrintLn("r - read time");
        SerialPrintLn("g - get time");
        SerialPrintLn("t - get temperature");
        SerialPrintLn("b - battery");
        break;

      case '1':
        // 1 per second sq wave
        //        SerialPrintLn("Alarm 1Hz");
        //       // set_alarm_1hz();   // this causes a hang
        break;

      case '2':
        //        SerialPrintLn("alarm generic");
        //        set_alarm1_XXX();
        break;

      case '3':
        // set alarm in seconds
        alarmSec = 5;
        Serial.print("Alarm Seconds ");
        Serial.println(alarmSec, DEC);
        set_alarm1_sec(alarmSec);
        break;

      case '4':
        break;

      case '5':
        SerialPrintLn("Light power toggle");
        if (lightPowerState == 0)
          light_power_on();
        else
          light_power_off();
        break;

      case '6':
        SerialPrintLn("Light button");
        light_button();
        break;

      case '7':
        SerialPrintLn("Camera power toggle");
        if (camPowerState == 0)
          cam_power_on();
        else
          cam_power_off();
        break;

      case '8':
        SerialPrintLn("Cam button");
        cam_button();
        break;

      case 'c':
        SerialPrintLn("Save Config");
        saveConfig();
        break;

      case 'd':
        SerialPrintLn("Load Config");
        loadConfig();
        printConfig();
        break;

      case 'g':
        //SerialPrintLn("get time");
        get_time ();
        break;

      case 'i':
        rtc_init();
        break;

      //      case 'r':
      //        SerialPrintl=Ln("ReadTimeDate");
      //        Serial.println(ReadTimeDate());
      //        break;

      case 's':
        // set time
        SerialPrintLn("set time");
        // TODO: get user to enter the time
        set_DS3234_time ("T355720603122015");
        break;

      case 't':
        get_temperature();
        break;

      case 'b':
        SerialPrintLn("read_battery");
        read_battery(1);
        break;

      default:
        break;
    }
  }
}

//----------------------------------------------------------
//
// Routine for reading voltage and current
//
//----------------------------------------------------------
unsigned long read_battery(int prnFlg)
{
  unsigned long ulBat, batPower;
  unsigned int voltCnt, currCnt;
  float batVolt;

  voltCnt = (unsigned int) analogRead(A0);
  currCnt = (unsigned int) analogRead(A1);

  ulBat = (unsigned long) voltCnt;   //2590 is 2.518   16.59 is 14.00
  ulBat *= 2687;   //3184;  //2137;      //3184;              // 504 cnts = 16.05v implies 31.85 scale factor
  ulBat /= 100;
  batVoltage = (unsigned int) ulBat;

  batCurrent = currCnt * 49; //28;    // 49;    // scale to milliamps (approx) (4.88 mA / cnt)
  batCurrent /= 10;            // 4.9

  if (prnFlg == 1)
  {
    Serial.print("Voltage = ");
    batVolt = (float)batVoltage;
    batVolt /= 1000;
    Serial.print(batVolt);
    Serial.print(" V ");
    //Serial.print(voltCnt);    // uncomment to check calibration
    //Serial.print(" cnt ");

    Serial.print("  Current = ");
    Serial.print(batCurrent);
    Serial.print(" mA ");
    //Serial.print(currCnt);
    //Serial.print(" cnt");
    Serial.println(" ");
  }

  batPower = (unsigned long) batVoltage;
  batPower *= (unsigned long) batCurrent;
  return (batPower);

}

//----------------------------------------------------------
//
// read temperature register in RTC
//
//----------------------------------------------------------
void get_temperature(void)
{
  get_DS3234_temperature ();
}


//----------------------------------------------------------
//
// routines using the RTC interrupt for timing, granularity is set by alarmInterval (default is 10 seconds)
//
//----------------------------------------------------------
unsigned long set_deployment_start(void)
{
  deploymentStartSec = get_seconds();
  return (deploymentStartSec);

}

unsigned long get_deployment_seconds(void)
{
  deploymentSeconds = get_seconds() - deploymentStartSec;
  return (deploymentSeconds);

}

//----------------------------------------------------------
//
// routines for counting seconds using regular interrupt input (alarmInterval=10 typically) from RTC
//
//----------------------------------------------------------
unsigned long get_seconds(void)
{
  unsigned long sec;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    sec = ulTimeCount * alarmInterval;
  }
  return (sec);
}

unsigned long delta_seconds(void)
{
  deltaSeconds = get_seconds();
  deltaSeconds -= startSeconds;
  return (deltaSeconds);
}


//----------------------------------------------------------
//
// routine for reading and printing the time
//
//----------------------------------------------------------

void get_time (void)
{
  DS3234_get(RTC_CS_L, &t);
  //if (prnFlg == 1)
  //{
  snprintf(prnBuf, PRN_BUF_SIZE, "%d.%02d.%02d %02d:%02d:%02d", t.year,
           t.mon, t.mday, t.hour, t.min, t.sec);
  Serial.println(prnBuf);
  //}
}


//----------------------------------------------------------
//
// initialize RTC DS3234
//
//----------------------------------------------------------
void rtc_init (void)
{
  DS3234_init(RTC_CS_L, DS3234_INTCN);
}

// Note on DS3234
// Active-Low Interrupt or Square-Wave Output. This open-drain pin requires an external pullup resistor. It can be
// left open if not used. This multifunction pin is determined by the state of the INTCN bit in the Control Register
// (0Eh). When INTCN is set to logic 0, this pin outputs a square wave and its frequency is determined by RS2
// and RS1 bits. When INTCN is set to logic 1, then a match between the timekeeping registers and either of the
// alarm registers activates the INT // /SQW pin (if the alarm is enabled). Because the INTCN bit is set to logic 1
// when power is first applied, the pin defaults to an interrupt output with alarms disabled. The pullup voltage can
// be up to 5.5V, regardless of the voltage on VCC. If not used, this pin can be left unconnected.

//----------------------------------------------------------
//
// 1 per second sq wave
//
//----------------------------------------------------------

void set_alarm_1hz(void)
{
  DS3234_set_creg(RTC_CS_L, 0);
}

//----------------------------------------------------------
//
// set the alarm time (hour,min,sec)
//
//----------------------------------------------------------

void set_alarm1_XXX(void)
{

  // flags define what calendar component to be checked against the current time in order
  // to trigger the alarm - see datasheet
  // A1M1 (seconds) (0 to enable, 1 to disable)
  // A1M2 (minutes) (0 to enable, 1 to disable)
  // A1M3 (hour)    (0 to enable, 1 to disable)
  // A1M4 (day)     (0 to enable, 1 to disable)
  // DY/DT          (dayofweek == 1/dayofmonth == 0)
  uint8_t flags[5] = { 0, 0, 0, 1, 1 };

  // time when to wake up
  uint8_t wake_HOUR = 15;
  uint8_t wake_MINUTE = 46;
  uint8_t wake_SECOND = 9;

  // set Alarm1
  DS3234_set_a1(RTC_CS_L, wake_SECOND, wake_MINUTE, wake_HOUR, 0, flags);

  // activate Alarm1
  //DS3234_set_creg(cs, DS3234_INTCN | DS3234_A1IE);
  //DS3234_set_creg(RTC_CS_L, 0);
  DS3234_set_creg(RTC_CS_L, DS3234_INTCN | DS3234_A1IE);


}

//----------------------------------------------------------
//
// set the alarm time (hour,min,sec)
//
//----------------------------------------------------------

void set_alarm1_sec(unsigned int seconds)
{
  // use alarm1

  //	if(seconds > 59)
  //	{
  //		seconds = 59;
  //	}
  //
  //	seconds = 40;

  //Serial.print("Seconds ");
  //Serial.println(DEC, seconds);

  //seconds = 10;		// DEBUG

  // flags define what calendar component to be checked against the current time in order
  // to trigger the alarm - see datasheet
  // A1M1 (seconds) (0 to enable, 1 to disable)
  // A1M2 (minutes) (0 to enable, 1 to disable)
  // A1M3 (hour)    (0 to enable, 1 to disable)
  // A1M4 (day)     (0 to enable, 1 to disable)
  // DY/DT          (dayofweek == 1/dayofmonth == 0)
  uint8_t flags[5] = { 0, 1, 1, 1, 1 };

  // time when to wake up
  //uint8_t wake_HOUR = 15;
  //uint8_t wake_MINUTE = 46;
  //uint8_t wake_SECOND = 30;

  // set Alarm1
  //DS3234_set_a1(RTC_CS_L, wake_SECOND, 0, 0, 0, flags);
  DS3234_set_a1(RTC_CS_L, seconds, 0, 0, 0, flags);

  // activate Alarm1
  //DS3234_set_creg(cs, DS3234_INTCN | DS3234_A1IE);
  //DS3234_set_creg(RTC_CS_L, 0);
  DS3234_set_creg(RTC_CS_L, DS3234_INTCN | DS3234_A1IE);

}

//----------------------------------------------------------
//
// set alarm minutes (not hour,day) using DS3234 alarm 2
//
//----------------------------------------------------------

void set_next_alarm2(void)
{
  struct ts t;
  unsigned char wakeup_min;

  DS3234_get(RTC_CS_L, &t);

  // calculate the minute when the next alarm will be triggered
  wakeup_min = (t.min / sleep_period + 1) * sleep_period;
  if (wakeup_min > 59)
  {
    wakeup_min -= 60;
  }

  // flags define what calendar component to be checked against the current time in order
  // to trigger the alarm
  // A2M2 (minutes) (0 to enable, 1 to disable)
  // A2M3 (hour)    (0 to enable, 1 to disable)
  // A2M4 (day)     (0 to enable, 1 to disable)
  // DY/DT          (dayofweek == 1/dayofmonth == 0)
  uint8_t flags[4] = { 0, 1, 1, 1 };

  // set Alarm2. only the minute is set since we ignore the hour and day component
  DS3234_set_a2(RTC_CS_L, wakeup_min, 0, 0, flags);

  // activate Alarm2
  DS3234_set_creg(RTC_CS_L, DS3234_INTCN | DS3234_A2IE);
}


//----------------------------------------------------------
//
// a wizard for setting up SeeStar (just and idea for now
//
//----------------------------------------------------------
void setup_seeStar (void)
{
  SerialPrintLn("Setup is a sequence of prompts to setup for deployment, NOT IMPLEMENTED YET, example");
  
  SerialPrintLn("Set Interval=60, the time between images/videos");
  SerialPrintLn("Set VideoDur=0, video length, set to zero for images (need to have the right GoPro ash script");
  SerialPrintLn("Set DeployDur=240, set the deployment duration in hours (this example is 10 days");
  

}

//----------------------------------------------------------
//
// routine for setting the time using SerCmd
//
//----------------------------------------------------------
void set_time (void)
{
  char *arg;

  SerialPrintLn("set time ...");
  // 2015.12.25 5 14:52:35


  arg = SCmd.next();
  Serial.println(arg);
  t.year = inp2toi(arg, 0) * 100 + inp2toi(arg, 2);
  Serial.print("year: ");
  Serial.println(t.year, DEC);

  arg = SCmd.next();
  Serial.println(arg);
  t.mon = inp2toi(arg, 0);
  Serial.print("mon: ");
  Serial.println(t.mon, DEC);

  arg = SCmd.next();
  Serial.println(arg);
  t.mday = inp2toi(arg, 0);
  Serial.print("mday: ");
  Serial.println(t.mday, DEC);

  arg = SCmd.next();
  Serial.println(arg);
  //  t.wday = inp2toi(arg, 0);
  t.wday = *arg & 0x0f;
  Serial.print("wday: ");
  Serial.println(t.wday, DEC);

  arg = SCmd.next();
  Serial.println(arg);
  t.hour = inp2toi(arg, 0);
  Serial.print("hour: ");
  Serial.println(t.hour, DEC);

  arg = SCmd.next();
  Serial.println(arg);
  t.min = inp2toi(arg, 0);
  Serial.print("min: ");
  Serial.println(t.min, DEC);

  arg = SCmd.next();
  Serial.println(arg);
  t.sec = inp2toi(arg, 0);
  Serial.print("sec: ");
  Serial.println(t.sec, DEC);


  DS3234_set(RTC_CS_L, t);
  SerialPrintLn("OK");



}
//----------------------------------------------------------
//
// Start delay is the delay before timelapse intervalometer begins
//
//----------------------------------------------------------
void set_startdelay (void)
{
  SerialPrintLn("set start delay in sec ...");

  char *arg;

  // set the duration of timelapse deployment
  SerialPrint("Delay before timelapse intervalometer begins:  ");
  arg = SCmd.next();

  if (arg != NULL)
  {
    String str = String(arg);

    long start_delay = str.toInt();

    Serial.print(start_delay, DEC);
    SerialPrintLn(" sec");

    seeStarConfig.start_delay_secs = start_delay;

    saveConfig();   // write to EEPROM
  }
  else
  {
    SerialPrintLn("No start delay secs arg");
  }

}

//----------------------------------------------------------
//
//
//----------------------------------------------------------
void set_led_delay (void)
{
  SerialPrintLn("set led delay ...");
}

//----------------------------------------------------------
//
//
//----------------------------------------------------------
void set_led_duration (void)
{
  SerialPrintLn("set led duration ...");
}

//----------------------------------------------------------
//
// print parameters (config structure)
//
//----------------------------------------------------------
void get_param (void)
{
  float  hrs;
  //SerialPrintLn("get param is ...");

  hrs = (float)seeStarConfig.deployment_secs;
  hrs /= 3600;
  SerialPrint("Duration ");
  Serial.print(seeStarConfig.deployment_secs, DEC);
  SerialPrint(" sec,  ");
  Serial.print(hrs);
  SerialPrintLn(" hrs");

  SerialPrint("Start Delay ");
  Serial.print(seeStarConfig.start_delay_secs);
  SerialPrintLn(" sec");

  SerialPrint("Video Duration ");
  Serial.print(seeStarConfig.video_dur_secs);
  SerialPrintLn(" sec");

  SerialPrint("Interval between videos/images ");
  Serial.print(seeStarConfig.interval_secs);
  SerialPrintLn(" sec");

  SerialPrint("Low battery threshold  ");
  Serial.print(seeStarConfig.bat_thresh_mv);
  SerialPrintLn(" mV");
  
  if (seeStarConfig.startFlg == 0)
    SerialPrintLn("Start Flag is OFF");
  else
    SerialPrintLn("Start Flag is ON");  

  SerialPrint("Config Version ");
  Serial.println(seeStarConfig.version_of_config);


}

//----------------------------------------------------------
//
// set defaults config (TODO: these need to be #defines set at top of code)
//
//----------------------------------------------------------
void set_default_config(void)
{

  seeStarConfig.deployment_secs = 240;   // 10 days
  Serial.println(seeStarConfig.deployment_secs);
  seeStarConfig.deployment_secs *= 3600;   // 10 days
  Serial.println(seeStarConfig.deployment_secs);

  seeStarConfig.start_delay_secs = 600;      // 10 minutes
  seeStarConfig.video_dur_secs = 60;        // 1 minute

  seeStarConfig.interval_secs = 300;        // 5 minutes
  seeStarConfig.bat_thresh_mv = 12500;      // low battery as 12500
  seeStarConfig.startFlg = 0;

  saveConfig();
  loadConfig();
  get_param();

}

//----------------------------------------------------------
//
// check the validity of the config parameters
//
//----------------------------------------------------------
int checkConfig(void)
{

  //        Serial.println(seeStarConfig.deployment_secs, DEC);
  //        Serial.println(seeStarConfig.start_delay_secs, DEC);
  //        Serial.println(seeStarConfig.video_dur_secs, DEC);
  //        Serial.println(seeStarConfig.interval_secs, DEC);
  //        Serial.println(seeStarConfig.version_of_config);
  int retVal = 0;

  if (seeStarConfig.interval_secs <= seeStarConfig.video_dur_secs)
  {
    SerialPrintLn("Error, Interval is less than Video Duration, please fix");
    retVal = -1;
  }
  if (seeStarConfig.deployment_secs <= seeStarConfig.interval_secs)
  {
    SerialPrintLn("Error, Deployment is less than Interval, please fix");
    retVal = -1;
  }

  if (seeStarConfig.interval_secs <= 30)
  {
    SerialPrintLn("Error, Interval is less than 30 sec, please fix");
    retVal = -1;
  }

  return (retVal);
}

//----------------------------------------------------------
//
// Set the Interval between videos/images with SerCmd
//
//----------------------------------------------------------
void set_interval (void)
{
  char *arg;

  // set the duration of timelapse deployment
  SerialPrint("Interval between videos/images:  ");
  arg = SCmd.next();

  if (arg != NULL)
  {
    String str = String(arg);

    long interval = str.toInt();

    Serial.print(interval, DEC);
    SerialPrintLn(" sec");

    seeStarConfig.interval_secs = interval;

    checkConfig();

    saveConfig();   // write to EEPROM
  }
  else
  {
    SerialPrintLn("No interval secs arg");
  }

}

//----------------------------------------------------------
//
// Set the duration of video with SerCmd (videodur=0 for images)
//
//----------------------------------------------------------
void set_videodur (void)
{
  char *arg;

  // set the duration of timelapse deployment
  SerialPrintLn("Set Video Duration in seconds");
  arg = SCmd.next();

  if (arg != NULL)
  {
    //aNumber=atoi(arg);    // Converts a char string to an integer
    //Serial.print("First argument: ");
    //Serial.println(arg);

    String str = String(arg);
    long dur = str.toInt();

    Serial.print(dur, DEC);
    SerialPrintLn(" secs");

    seeStarConfig.video_dur_secs = dur;

    set_cam_on_duration();

    saveConfig();

  }
  else
  {
    SerialPrintLn("No Deployment duration hours arg");
  }

}

//----------------------------------------------------------
//
// Set the deployment duration with SerCmd, this determines how long the intervalometer runs
//
//----------------------------------------------------------
void set_deploydur (void)
{
  char *arg;

  // set the duration of timelapse deployment
  SerialPrintLn("Set Deployment Duration in hours");
  arg = SCmd.next();

  if (arg != NULL)
  {
    //aNumber=atoi(arg);    // Converts a char string to an integer
    Serial.print("First argument: ");
    Serial.println(arg);

    String str = String(arg);

    long dur = str.toInt();

    Serial.print(dur, DEC);
    SerialPrint(" hours,   ");
    dur *= 3600;

    seeStarConfig.deployment_secs = dur;

    saveConfig();

    Serial.print(dur, DEC);
    SerialPrintLn(" secs");
  }
  else
  {
    SerialPrintLn("No Deployment duration hours arg");
  }

}



//----------------------------------------------------------
//
// DS3234 RTC Functions
//
//----------------------------------------------------------
void get_DS3234_alarm1(void)
{
  DS3234_get_a1(RTC_CS_L, &prnBuf[0], 59);
  Serial.println(prnBuf);
}

void get_DS3234_alarm2(void)
{
  DS3234_get_a2(RTC_CS_L, &prnBuf[0], 59);
  Serial.println(prnBuf);
}

void get_DS3234_aging_reg(void)
{
  Serial.print("aging reg is ");
  Serial.println(DS3234_get_aging(RTC_CS_L), DEC);
}

void get_DS3234_sram(void)
{
  int i;
  char sramByte;
  for (i = 0; i < SRAM_SIZE - 1; i++)
  {
    sramByte = DS3234_get_sram_8b(RTC_CS_L, i);

    Serial.print(sramByte, DEC);
    Serial.print(" ");

  }
}

//Set Alarm1 SSMMHHDD
void set_DS3234_alarm1(void)
{
  char *arg;

  //#ifdef  SERCMDON
  arg = SCmd.next();
  //#endif
  if (arg != NULL)
  {
    //aNumber=atoi(arg);    // Converts a char string to an integer
    Serial.print("First argument: ");
    Serial.println(arg);
  }
  else
  {
    SerialPrintLn("No SSMMHHDD arg");
  }

  DS3234_set_creg(RTC_CS_L, DS3234_INTCN | DS3234_A1IE);
  //SSMMHHDD
  //for (i = 0; i < 4; i++)
  //{
  //    time[i] = (cmd[2 * i + 1] - 48) * 10 + cmd[2 * i + 2] - 48; // ss, mm, hh, dd
  //}
  // SS.MM.HH.DD
  time[0] = (arg[0] & 0x0f) * 10 + (arg[1] & 0x0f);
  time[1] = (arg[3] & 0x0f) * 10 + (arg[4] & 0x0f);
  time[2] = (arg[6] & 0x0f) * 10 + (arg[7] & 0x0f);
  time[3] = (arg[9] & 0x0f) * 10 + (arg[10] & 0x0f);

  uint8_t flags[5] = { 0, 0, 0, 0, 0 };
  DS3234_set_a1(RTC_CS_L, time[0], time[1], time[2], time[3], flags);
  DS3234_get_a1(RTC_CS_L, &prnBuf[0], 59);
  Serial.println(prnBuf);
}

void set_DS3234_alarm2(void)
{
  char *arg;

  //#ifdef SERCMDON
  arg = SCmd.next();
  //#endif
  if (arg != NULL)
  {
    //aNumber=atoi(arg);    // Converts a char string to an integer
    Serial.print("First argument: ");
    Serial.println(arg);
  }
  else
  {
    SerialPrintLn("No SS.MM.HH arg");
  }


  DS3234_set_creg(RTC_CS_L, DS3234_INTCN | DS3234_A2IE);
  // SS.MM.HH
  time[0] = (arg[0] & 0x0f) * 10 + (arg[1] & 0x0f);
  time[1] = (arg[3] & 0x0f) * 10 + (arg[4] & 0x0f);
  time[2] = (arg[6] & 0x0f) * 10 + (arg[7] & 0x0f);


  uint8_t flags[5] = { 0, 0, 0, 0, 0 };
  DS3234_set_a2(RTC_CS_L, time[0], time[1], time[2], flags);
  DS3234_get_a2(RTC_CS_L, &prnBuf[0], 59);
  Serial.println(prnBuf);

  //Serial.println("B - set DS3234 Alarm 2, format mm.hh.dd, ex. B15.09.10 is 10th day at 9:15 am");
}


void get_DS3234_temperature (void)
{
  SerialPrint("temperature reg is ");
  Serial.println(DS3234_get_treg(RTC_CS_L));

  //Serial.println("C - get DS3234 temperature register");

}

void reset_DS3234_status_reg_alarm (void)
{
  uint8_t reg_val;

  // "D" - reset status register alarm flags
  reg_val = DS3234_get_sreg(RTC_CS_L);
  reg_val &= B11111100;
  DS3234_set_sreg(RTC_CS_L, reg_val);
  SerialPrintLn("reset status reg alarm flags");

  //Serial.println("D - reset DS3234 status register alarm flags");

}


void reset_DS3234_aging_reg (void)
{
  // "G" - reset aging status register
  DS3234_set_aging(RTC_CS_L, 0);
}


void get_DS3234_status_reg (void)
{
  Serial.print("status reg is ");
  Serial.println(DS3234_get_sreg(RTC_CS_L), DEC);

  //Serial.println("S - get DS3234 status register");
}


void set_DS3234_time (char *arg)
{
  //  char *arg;
  struct ts t;
  // TODO, get from arg
  //T355720619112011

  Serial.println(arg);   // debug


  //#ifdef SERCMDON
  //35.57.20.6.19.11.2011
  arg = SCmd.next();
  //#endif

  t.sec = inp2toi(arg, 1);
  Serial.print(t.sec, DEC);  // DEBUG

  t.min = inp2toi(arg, 3);
  t.hour = inp2toi(arg, 5);
  t.wday = inp2toi(arg, 7);
  t.mday = inp2toi(arg, 8);
  t.mon = inp2toi(arg, 10);
  t.year = inp2toi(arg, 12) * 100 + inp2toi(arg, 14);
  DS3234_set(RTC_CS_L, t);
  SerialPrintLn("OK");

}



//----------------------------------------------------------
//
//  Function for turning light on
//
//----------------------------------------------------------
void led_light_on (void)
{
  unsigned int dur;

  // turn power on to the LED flashlight - this does not turn on the light, this enables the LED flashlight controller
  SerialPrintLn("Light Power ON");
  digitalWrite(LIGHT_POWER, HIGH);

  read_battery(1);
  delay(500);

  // press the LED flashlight button
  SerialPrintLn("Light Button Push");
  digitalWrite(LIGHT_BUTTON, HIGH);   // was PIN4
  delay(200);
  digitalWrite(LIGHT_BUTTON, LOW);

  delay(100);
  read_battery(1);

  lightPowerState = 1;

}

//----------------------------------------------------------
//
//  Function for turning light off
//
//----------------------------------------------------------
void led_light_off (void)
{
  unsigned int dur;

  read_battery(1);

  // Turn off the LED Flashlight
  SerialPrintLn("Light Power OFF");
  digitalWrite(LIGHT_POWER, LOW);

  lightPowerState = 0;

  delay(100);

  read_battery(1);
}

//----------------------------------------------------------
//
//  Function for turning camera power on
//
//----------------------------------------------------------
void cam_power_on(void)
{
  SerialPrintLn("Camera Power ON");
  digitalWrite(CAM_POWER, HIGH);
  camPowerState = 1;
}

//----------------------------------------------------------
//
//  Function for turning camera power off
//
//----------------------------------------------------------
void cam_power_off(void)
{
  SerialPrintLn("Camera Power OFF");
  digitalWrite(CAM_POWER, LOW);
  camPowerState = 0;
}

//----------------------------------------------------------
//
//  Function for pressing camera button, uses delay
//
//----------------------------------------------------------
unsigned int cam_button(void)
{
  unsigned int msec_dur = 100;

  SerialPrintLn("cam button");

  digitalWrite(CAM_ON, HIGH);
  delay(msec_dur);          // small 'button push'
  digitalWrite(CAM_ON, LOW);

  return (msec_dur);

}

//----------------------------------------------------------
//
//  Function for turning light power on (needs light button press to turn light on)
//
//----------------------------------------------------------
void light_power_on(void)
{
  SerialPrintLn("Light Power ON");
  digitalWrite(LIGHT_POWER, HIGH);
  lightPowerState = 1;
}

//----------------------------------------------------------
//
//  Function for turning light power off
//
//----------------------------------------------------------
void light_power_off(void)
{
  SerialPrintLn("Light Power OFF");
  digitalWrite(LIGHT_POWER, LOW);
  lightPowerState = 0;
}

unsigned int light_button(void)
{
  unsigned int msec_dur = 200;

  SerialPrintLn("Light Button");
  digitalWrite(LIGHT_BUTTON, HIGH);   //was PIN4
  delay(msec_dur);
  digitalWrite(LIGHT_BUTTON, LOW);

  return (msec_dur);
}

//----------------------------------------------------------
//
//  Function for setting low battery threshold with SerCmd
//
//----------------------------------------------------------
void set_battery (void)
{
  char *arg;

  // set the duration of timelapse deployment
  SerialPrintLn("Set battery threshold in millivolt");
  arg = SCmd.next();

  if (arg != NULL)
  {

    String str = String(arg);
    long batThresh = str.toInt();

    Serial.print(batThresh, DEC);
    SerialPrintLn(" secs");

    seeStarConfig.bat_thresh_mv = batThresh;

    saveConfig();

  }
  else
  {
    SerialPrintLn("No battery threshold millivolts");
  }


}

//----------------------------------------------------------
//
//  read and print battery state
//
//----------------------------------------------------------
void get_battery (void)
{
  read_battery(1);
}



//----------------------------------------------------------
//
//  start intervalometer - set state machine in motion, write autostart flag in eeprom
//
//----------------------------------------------------------
void intervalometer_go (void)
{
  // take picture or shoot video after waiting for the startup delay
  SerialPrintLn("Intervalometer Go");

  intervalometerState = ST_STARTUP_DELAY;   // Transition out of ST_INIT
  
  if(seeStarConfig.startFlg == 0)    // if not already set, then set it in EEPROM
  {
    seeStarConfig.startFlg = 1;
  
    saveConfig();
  }
}

//----------------------------------------------------------
//
//  stop intervalometer - camera state machine will finish then intervalometer will stop, write autostart flag to off in eeprom
//
//----------------------------------------------------------
void intervalometer_stop (void)
{
  // take picture or shoot video after waiting for the startup delay
  SerialPrintLn("Intervalometer Stop");

  intervalometerState = ST_SHUTDOWN;   // Transition out of ST_INIT
  
  seeStarConfig.startFlg = 0;
  
  saveConfig();
  
}



//----------------------------------------------------------
//
// intervalometer statemachine - calls a separate camera_handler state machine
//

//____________________________|-------|________________|-------|________________|-------|________________|-------|_
//<--- startup delay -------->
//                            |<----- interval ------>|<----- interval ------>|<----- interval ------>|
//                            |<-Dur->|
//                            |<------------- Deployment Duration ----------------------------------------------->|
//
//
//----------------------------------------------------------
void intervalometer_handler(void)
{
  unsigned long startTime;
  unsigned long deltaTime;
  unsigned long msec_spent;

  //        Serial.println(seeStarConfig.deployment_secs, DEC);
  //        Serial.println(seeStarConfig.start_delay_secs, DEC);
  //        Serial.println(seeStarConfig.video_dur_secs, DEC);
  //        Serial.println(seeStarConfig.interval_secs, DEC);
  //        Serial.println(seeStarConfig.version_of_config);

  switch (intervalometerState)
  {
    case ST_INIT:
      startSeconds = get_seconds();
      // state transitions is done via intervalometer_go() routine
      break;

  case ST_SHUTDOWN:
      // this state is due to a STOP command from the user
      if (camera_handler(1) == 1)
      {
        SerialPrintLn("Camera Done");
        intervalometerState = ST_INIT;        
      }
    break;


    case ST_STARTUP_DELAY:
      if (delta_seconds() >= seeStarConfig.start_delay_secs)
      {
        set_deployment_start();    // deploymentStartSec, deployment starts after the delay
        if (dbgCam) SerialPrintLn("Transition to ST_INTERVAL_INIT");
        intervalometerState = ST_INTERVAL_INIT;
      }
      break;


    case ST_INTERVAL_INIT:
      startSeconds = get_seconds();   //delta seconds will be used for timing interval
      camera_go();                 // needed to start camera_handler()
      if (dbgCam) SerialPrintLn("Transition to ST_INTERVAL_START");

      intervalometerState = ST_INTERVAL_START;
      break;

    case ST_INTERVAL_START:
      // call camera_handler state machine until it completes
      if (camera_handler(1) == 1)    // returns 1 when complete (prnFlg is passed in
      {
        SerialPrintLn("Camera Done");
        intervalometerState = ST_INTERVAL_WAITNEXT;
      }
      break;


    case ST_INTERVAL_WAITNEXT:
      if (delta_seconds() >= seeStarConfig.interval_secs)
      {
        if (dbgCam) SerialPrintLn("Transition to ST_INTERVAL_INIT");
        intervalometerState = ST_INTERVAL_INIT;
      }

      if (get_deployment_seconds() >= seeStarConfig.deployment_secs)
      {
        intervalometerState = ST_DEPLOYMENT_COMPLETE;
      }
      break;

    case ST_DEPLOYMENT_COMPLETE:
      SerialPrint("Deployment is Complete  ");
      Serial.print(seeStarConfig.deployment_secs / 3600L);
      SerialPrintLn(" hrs");
      intervalometerState = ST_INIT;
      break;

    default:
      break;

  }
}





//----------------------------------------------------------
//
// start camera state machine
//
//----------------------------------------------------------
void camera_go(void)
{
  //camLoopCnt = 0;
  //camCycleCnt = cycles;

  SerialPrintLn("Camera Go, ST_CAM_GO");
  camState = ST_CAM_GO;
}



//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
int delay_cam_timeout(unsigned long msec_delay)
{
  // NOTE Need to detect rollover
  cam_curMillis = millis();
  //if (cam_curMillis - cam_prevMillis > msec_delay)
  if ((unsigned long)(cam_curMillis - cam_prevMillis) > msec_delay)
  {
    cam_prevMillis = cam_curMillis;
    return 1;
  }
  return 0;
}

//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void start_cam_timeout(void)
{
  cam_prevMillis = millis();
}


//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
int delay_light_timeout(unsigned long msec_delay)
{
  // NOTE Need to detect rollover
  light_curMillis = millis();
  //if (light_curMillis - light_prevMillis > msec_delay)
  if ((unsigned long)(light_curMillis - light_prevMillis) > msec_delay)
  {
    light_prevMillis = light_curMillis;
    return 1;
  }
  return 0;
}

//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void start_light_timeout(void)
{
  light_prevMillis = millis();
}



// Camera statemachine is camera_handler


//CAMPOWER: _|--------------------------------------------------------------------------------------------------------|__
//CAM_ON:   ____||_______________________________________________________________________________________________________
//LEDPOWER: ____|-----------------------------------------------------------------------|________________________________
//LED_ON:   ______________________________________________________________||_____________________________________________
//Take Pix: __________________________________________________________________________||_________________________________
//Seconds:   0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19   20   21

// on the root of the GoPro uSD is autoexec.ash, it should look like this (with Unix line terminations)
//################################################
//# Hero3+: Silver: wait 5, PhotoMode, wait 5, Take one image, wait 5 seconds (for image write?) and power down #
//################################################
//
//sleep 5
//t app appmode photo
//sleep 5
//t app button shutter PR
//sleep 5
//poweroff yes
//reboot yes


// autoexec.ash delays (useed in the defines below)
#define ASH_BOOTDELAY      5000
#define ASH_MENUDELAY      5000   // delay after photomode and before shutter 
#define ASH_TAKEPHOTODELAY 5000

// definitions for the camera power and light power sequence / timeline
#define CAM_POWERUP_DELAY  1000   // was 1000
#define CAM_ON_BUTTON      100
#define LIGHT_ON_DELAY     2000   // delay after shutter to turn on Light
#define CAM_ON_TO_LIGHT_ON ASH_BOOTDELAY + ASH_MENUDELAY + LIGHT_ON_DELAY  // 12000  // ash script delays before PR are here: 5000ash + 5000ash + 2000
#define LIGHT_ON_BUTTON    200    // button press duration 100 does not work
#define LIGHT_ON_DURATION  3000   // 3000
#define CAM_OFF_DELAY      6000


int camera_handler(int prnBatFlg)
{

  //unsigned int i;
  //unsigned int dur;
  //unsigned int msec = 0;
  int retVal = 0;
  //unsigned long startTime;
  //unsigned long deltaTime;



  retVal = 0;
  switch (camState)
  {
    case ST_CAM_INIT:
      Serial.write('I');
      // state transitions comes from camera_go()
      retVal = 1;      // Handle the case of STOP command and the camera is doing nothing (THOMTHOM)
      break;

    case ST_CAM_GO:
      // START command issued by user
      Serial.print("Go");
      //  prnBatFlg = 1;    // print battery readings, 1 says print
      prnBatFlg = dbgCam;    // print battery readings, 1 says print
      read_battery(prnBatFlg);

      // turn on the 16v input power to the CamDo battery eliminator in the GoPro
      if (dbgCam) SerialPrintLn("Camera Power ON");
      digitalWrite(CAM_POWER, HIGH);

      start_cam_timeout();

      camState = ST_CAM_POWERUP_DELAY;
      break;

    case ST_CAM_POWERUP_DELAY:
      if (delay_cam_timeout(CAM_POWERUP_DELAY)) //
      {
        read_battery(prnBatFlg);
        camState = ST_CAM_LIGHT_POWER;
      }
      break;


    case ST_CAM_LIGHT_POWER:
      // turn power on to the LED flashlight - this does not turn on the light, this enables the LED flashlight controller
      if (dbgCam) SerialPrintLn("Light Power ON");
      digitalWrite(LIGHT_POWER, HIGH);
      camState = ST_CAM_ON_BUTTON;
      break;

    case ST_CAM_ON_BUTTON:
      // Push the GoPro camera 'on' button (pin 12 on GoPro bus)
      if (dbgCam) SerialPrintLn("Take Picture");
      digitalWrite(CAM_ON, HIGH);
      camState = ST_CAM_OFF_BUTTON;
      break;

    case ST_CAM_OFF_BUTTON:
      if (delay_cam_timeout(CAM_ON_BUTTON)) //
      {
        read_battery(prnBatFlg);
        digitalWrite(CAM_ON, LOW);
        camState = ST_CAM_ON_TO_LIGHT_ON;
      }
      break;

    case ST_CAM_ON_TO_LIGHT_ON:
      // Delay between GoPro Camera ON button push and turning on LED flashlight (ash script delays + scene adjust alg)
      // LED flash light should come on with a couple seconds to allow GoPro auto white balance algorithm time to adjust
      if (delay_cam_timeout(CAM_ON_TO_LIGHT_ON)) //
      {
        read_battery(prnBatFlg);
        if (dbgCam) SerialPrintLn("Light Button Push");
        digitalWrite(LIGHT_BUTTON, HIGH);  
        camState = ST_LIGHT_ON_BUTTON;
      }
      break;

    case ST_LIGHT_ON_BUTTON:
      if (delay_cam_timeout(LIGHT_ON_BUTTON)) //
      {
        digitalWrite(LIGHT_BUTTON, LOW);
        read_battery(prnBatFlg);
        set_cam_on_duration();
        camState = ST_LIGHT_ON_DURATION;
      }
      break;


    case ST_LIGHT_ON_DURATION:
      // light on duration, light should turn off just after GoPro takes picture (photo num increments in GoPro display)
      //if (delay_cam_timeout(LIGHT_ON_DURATION))     //seeStarConfig.video_dur_secs or LIGHT_ON_DURATION
      if (delay_cam_timeout(cam_on_duration))      // either LIGHT_ON_DURATION or seeStarConfig.video_dur_secs * 1000
      {
        //read_battery(prnBatFlg);
        read_battery(1);            // read battery under full load
        batVoltageLoad = batCurrent;   // globals used in battery_handler
        batCurrentLoad = batCurrent;

        // Turn off the LED Flashlight
        if (dbgCam) SerialPrintLn("Light Power OFF");
        digitalWrite(LIGHT_POWER, LOW);
        camState = ST_CAM_OFF_DELAY;
      }
      break;

    case ST_CAM_OFF_DELAY:
      if (delay_cam_timeout(CAM_OFF_DELAY)) //
      {
        if (dbgCam) SerialPrintLn("Camera Power OFF");
        digitalWrite(CAM_POWER, LOW);

        camState = ST_CAM_INIT;
        retVal = 1;
      }
      break;

    default:
      camState = ST_CAM_INIT;
      SerialPrintLn("Default CamState - ST_CAM_INIT");
      break;
  }

  return (retVal);
}

void set_cam_on_duration(void)
{
  cam_on_duration = seeStarConfig.video_dur_secs * 1000L;
  if (cam_on_duration == 0L)
    cam_on_duration = LIGHT_ON_DURATION;
}

// OLD UNUSED SEQUENTIAL ROUTINE
unsigned int powerup_take_image_powerdown()
{
  unsigned int i;
  unsigned int dur;
  unsigned int msec = 0;
  int prnBatFlg;
  unsigned long startTime;
  unsigned long deltaTime;

  //  prnBatFlg = 1;    // print battery readings, 1 says print
  prnBatFlg = dbgCam;    // print battery readings, 1 says print


  startTime = millis();
  if (dbgCam) Serial.println(startTime);

  deltaTime = (unsigned long)(millis() - startTime);
  if (dbgCam) Serial.println(deltaTime);

  read_battery(prnBatFlg);

  //deltaTime = millis() - startTime;
  deltaTime = (unsigned long)(millis() - startTime);
  if (dbgCam) Serial.println(deltaTime);

  // turn on the 16v input power to the CamDo battery eliminator in the GoPro
  if (dbgCam) SerialPrintLn("Camera Power ON");
  digitalWrite(CAM_POWER, HIGH);

  // powerup delay for camera
  dur = CAM_POWERUP_DELAY;
  if (dbgCam) Serial.print(dur);
  if (dbgCam) SerialPrintLn(" msec");
  delay(dur);
  msec += dur;

  read_battery(prnBatFlg);

  //deltaTime = millis() - startTime;
  deltaTime = (unsigned long)(millis() - startTime);
  if (dbgCam) Serial.println(deltaTime);

  // turn power on to the LED flashlight - this does not turn on the light, this enables the LED flashlight controller
  if (dbgCam) SerialPrintLn("Light Power ON");
  digitalWrite(LIGHT_POWER, HIGH);

  // Push the GoPro camera 'on' button (pin 12 on GoPro bus)
  dur = CAM_ON_BUTTON;
  if (dbgCam) SerialPrintLn("Take Picture");
  digitalWrite(CAM_ON, HIGH);
  delay(dur);          // small 'button push' delay (TODO: find out how small)
  digitalWrite(CAM_ON, LOW);
  msec += dur;

  // Delay between GoPro Camera ON button push and turning on LED flashlight (ash script delays + scene adjust alg)
  // LED flash light should come on with a couple seconds to allow GoPro auto white balance algorithm time to adjust
  dur = CAM_ON_TO_LIGHT_ON;
  if (dbgCam) Serial.print(dur);
  if (dbgCam) SerialPrintLn(" msec");
  delay(dur);
  read_battery(prnBatFlg);
  msec += dur;

  //deltaTime = millis() - startTime;
  deltaTime = (unsigned long)(millis() - startTime);
  if (dbgCam) Serial.println(deltaTime);
  read_battery(prnBatFlg);


  // press the LED flashlight button
  dur = LIGHT_ON_BUTTON;
  if (dbgCam) SerialPrintLn("Light Button Push");
  digitalWrite(LIGHT_BUTTON, HIGH);   //was PIN4
  delay(dur);
  digitalWrite(LIGHT_BUTTON, LOW);
  msec += dur;

  read_battery(prnBatFlg);

  // light on duration, light should turn off just after GoPro takes picture (photo num increments in GoPro display)
  dur = LIGHT_ON_DURATION;
  if (dbgCam) Serial.print(dur);
  if (dbgCam) SerialPrintLn(" msec");
  delay(dur);
  msec += dur;

  read_battery(prnBatFlg);
  read_battery(1);            // read battery under full load
  batVoltageLoad = batCurrent;   // globals used in battery_handler
  batCurrentLoad = batCurrent;

  //deltaTime = millis() - startTime;
  deltaTime = (unsigned long)(millis() - startTime);
  if (dbgCam) Serial.println(deltaTime);

  // Turn off the LED Flashlight
  if (dbgCam) SerialPrintLn("Light Power OFF");
  digitalWrite(LIGHT_POWER, LOW);

  read_battery(prnBatFlg);

  // wait a period of time to insure image is written to uSD card
  dur = CAM_OFF_DELAY;
  if (dbgCam) Serial.print(dur);
  if (dbgCam) SerialPrintLn(" msec");
  for (i = 0; i < 10; i++)
  {
    delay(dur / 10);
    msec += dur / 10;
    read_battery(prnBatFlg);
    if (batCurrent < 80)   // batCurrent is 60mA to 80mA typically after GoPro shuts itself down
    {
      // a little more delay just to be safe
      delay(dur / 10);
      msec += dur / 10;
      SerialPrintLn("GoPro powered off");
      //break;   // break and GoPro will be powered off based on current, break commmented for safe delay
    }
  }

  //deltaTime = millis() - startTime;
  deltaTime = (unsigned long)(millis() - startTime);
  Serial.println(deltaTime);

  if (dbgCam) SerialPrintLn("Camera Power OFF");
  digitalWrite(CAM_POWER, LOW);

  return (msec);
}

//----------------------------------------------------------
//
//
//----------------------------------------------------------
void saveConfig()
{
  for (unsigned int t = 0; t < sizeof(seeStarConfig); t++)
  { // writes to EEPROM
    EEPROM.write(CONFIG_START + t, *((char*)&seeStarConfig + t));
    // and verifies the data
    if (EEPROM.read(CONFIG_START + t) != *((char*)&seeStarConfig + t))
    {
      // error writing to EEPROM
    }
  }
}

//----------------------------------------------------------
//
//
//----------------------------------------------------------
void loadConfig()
{
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (//EEPROM.read(CONFIG_START + sizeof(seeStarConfig) - 1) == seeStarConfig.version_of_config[3] // this is '\0'
    EEPROM.read(CONFIG_START + sizeof(seeStarConfig) - 2) == seeStarConfig.version_of_config[2] &&
    EEPROM.read(CONFIG_START + sizeof(seeStarConfig) - 3) == seeStarConfig.version_of_config[1] &&
    EEPROM.read(CONFIG_START + sizeof(seeStarConfig) - 4) == seeStarConfig.version_of_config[0])
  { // reads seeStarConfig from EEPROM
    for (unsigned int t = 0; t < sizeof(seeStarConfig); t++)
      *((char*)&seeStarConfig + t) = EEPROM.read(CONFIG_START + t);
    SerialPrintLn("Config loaded from EEPROM");
    printConfig();
    SerialPrintLn(" ");

  }
  else
  {
    // seeStarConfig aren't valid! will overwrite with default seeStarConfig

    SerialPrintLn("EEPROM has invalid config, restoring defaults");
    saveConfig();
  }
}

//----------------------------------------------------------
//
//
//----------------------------------------------------------
void printConfig(void)
{
  //  unsigned long deployment_secs;    // timelapse duration in secs (user will enter in hours)
  //  unsigned long start_delay_secs;   // delay in sec before timelapse start
  //  unsigned long video_dur_secs;    // length of video in seconds
  //  unsigned long interval_secs;     // time between images/videos
  //  unsigned long bat_thresh_mv;
  //  unsigned int  startFlg;

  get_param();
#ifdef OLDCODE
  Serial.println(seeStarConfig.deployment_secs, DEC);
  Serial.println(seeStarConfig.start_delay_secs, DEC);
  Serial.println(seeStarConfig.video_dur_secs, DEC);
  Serial.println(seeStarConfig.interval_secs, DEC);
  Serial.println(seeStarConfig.bat_thresh_mv, DEC);
  if (seeStarConfig.startFlg == 0)
    SerialPrintLn("Start Flag is OFF");
  else
    SerialPrintLn("Start Flag is ON");

  Serial.println(seeStarConfig.version_of_config);
#endif  
}

#ifdef OLDCODE
String ReadTimeDate()
{
  String temp;
  int TimeDate [7]; //second,minute,hour,null,day,month,year

  for (int i = 0; i <= 6; i++)
  {
    if (i == 3)
      i++;
    digitalWrite(RTC_CS_L, LOW);
    SPI.transfer(i + 0x00);
    unsigned int n = SPI.transfer(0x00);
    digitalWrite(RTC_CS_L, HIGH);

    int a = n & B00001111;
    if (i == 2)
    {
      int b = (n & B00110000) >> 4; //24 hour mode
      if (b == B00000010)
        b = 20;
      else if (b == B00000001)
        b = 10;
      TimeDate[i] = a + b;
    }
    else if (i == 4)
    {
      int b = (n & B00110000) >> 4;
      TimeDate[i] = a + b * 10;
    }
    else if (i == 5)
    {
      int b = (n & B00010000) >> 4;
      TimeDate[i] = a + b * 10;
    }
    else if (i == 6)
    {
      int b = (n & B11110000) >> 4;
      TimeDate[i] = a + b * 10;
    }
    else
    {
      int b = (n & B01110000) >> 4;
      TimeDate[i] = a + b * 10;
    }
  }
  temp.concat(TimeDate[4]);
  temp.concat("/") ;
  temp.concat(TimeDate[5]);
  temp.concat("/") ;
  temp.concat(TimeDate[6]);
  temp.concat("     ") ;
  temp.concat(TimeDate[2]);
  temp.concat(":") ;
  temp.concat(TimeDate[1]);
  temp.concat(":") ;
  temp.concat(TimeDate[0]);
  return (temp);
}
#endif

#ifdef NOCODE

int sleepStatus = 0;             // variable to store a request for sleep
int count = 0;                   // counter

void wakeUpNow()        // here the interrupt is handled after wakeup
{
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
}


//----------------------------------------------------------
//
//  Low Power examples
//----------------------------------------------------------
void poop()
{
  // display information about the counter
  Serial.print("Awake for ");
  Serial.print(count);
  Serial.println("sec");
  count++;
  delay(1000);                           // waits for a second

  // compute the serial input
  if (Serial.available()) {
    int val = Serial.read();
    if (val == 'S') {
      Serial.println("Serial: Entering Sleep mode");
      delay(100);     // this delay is needed, the sleep
      //function will provoke a Serial error otherwise!!
      count = 0;
      sleepNow();     // sleep function called here
    }
    if (val == 'A') {
      Serial.println("Hola Caracola"); // classic dummy message
    }
  }

  // check if it should go to sleep because of time
  if (count >= 10) {
    Serial.println("Timer: Entering Sleep mode");
    delay(100);     // this delay is needed, the sleep
    //function will provoke a Serial error otherwise!!
    count = 0;
    sleepNow();     // sleep function called here
  }
}

void sleepNow()         // here we put the arduino to sleep
{
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
   * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
   * there is a list of sleep modes which explains which clocks and
   * wake up sources are available in which sleep mode.
   *
   * In the avr/sleep.h file, the call names of these sleep modes are to be found:
   *
   * The 5 different modes are:
   *     SLEEP_MODE_IDLE         -the least power savings
   *     SLEEP_MODE_ADC
   *     SLEEP_MODE_PWR_SAVE
   *     SLEEP_MODE_STANDBY
   *     SLEEP_MODE_PWR_DOWN     -the most power savings
   *
   * For now, we want as much power savings as possible, so we
   * choose the according
   * sleep mode: SLEEP_MODE_PWR_DOWN
   *
   */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

  sleep_enable();          // enables the sleep bit in the mcucr register
  // so sleep is possible. just a safety pin

  /* Now it is time to enable an interrupt. We do it here so an
   * accidentally pushed interrupt button doesn't interrupt
   * our running program. if you want to be able to run
   * interrupt code besides the sleep function, place it in
   * setup() for example.
   *
   * In the function call attachInterrupt(A, B, C)
   * A   can be either 0 or 1 for interrupts on pin 2 or 3.
   *
   * B   Name of a function you want to execute at interrupt for A.
   *
   * C   Trigger mode of the interrupt pin. can be:
   *             LOW        a low level triggers
   *             CHANGE     a change in level triggers
   *             RISING     a rising edge of a level triggers
   *             FALLING    a falling edge of a level triggers
   *
   * In all but the IDLE sleep modes only LOW can be used.
   */
  attachInterrupt(0, wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function
  // wakeUpNow when pin 2 gets LOW

  sleep_mode();            // here the device is actually put to sleep!!
  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

  sleep_disable();         // first thing after waking from sleep:
  // disable sleep...
  detachInterrupt(0);      // disables interrupt 0 on pin 2 so the
  // wakeUpNow code will not be executed
  // during normal running time.

}

#endif
