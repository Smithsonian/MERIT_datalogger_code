
/* Code for for Wadden Sea Warming System
   Roy Rich
   Modbus communication via xbee sc2 unit

   Before installing this program:
   1) set RTC chip for accurate time (remove battery if timeset is wonky)
   2) set slaveID for each chip to EPROM
   3) Xbee modules also need to be configured using XCTU using configuration profiles
   I have included comments as needed and left code in place as need might arise

2019 changes to be implemented:
-Diagnostic ports for testing heating systems 5 seconds on for each port
-fix -7999 data
-overall checks
-remove hard resets
- changes boudaries for doing avergaes to -25 to 60
- add visual alarms for overheated on screenand led blinking to pin 13

*/

char program[16] = "HalligWarmv1.17"; // record program version in data.file

/*
  From Modus library
   SimpleModbusSlaveV10 supports function 3, 6 & 16.

   This example code will receive the adc ch0 value from the arduino master.
   It will then use this value to adjust the brightness of the led on pin 9.
   The value received from the master will be stored in address 1 in its own
   address space namely holdingRegs[].

   In addition to this the slaves own adc ch0 value will be stored in
   address 0 in its own address space holdingRegs[] for the master to
   be read. The master will use this value to alter the brightness of its
   own led connected to pin 9.

   The modbus_update() method updates the holdingRegs register array and checks
   communication.

   Note:
   The Arduino serial ring buffer is 64 bytes or 32 registers.
   Most of the time you will connect the arduino to a master via serial
   using a MAX485 or similar.

   In a function 3 request the master will attempt to read from your
   slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
   and two BYTES CRC the master can only request 58 bytes or 29 registers.

   In a function 16 request the master will attempt to write to your
   slave and since a 9 bytes is already used for ID, FUNCTION, ADDRESS,
   NO OF REGISTERS, NO OF BYTES and two BYTES CRC the master can only write
   54 bytes or 27 registers.

   Using a USB to Serial converter the maximum bytes you can send is
   limited to its internal buffer which differs between manufactures.

  Note that delays can mess up this communication methods; Use millis() as needed instead.

*/
// LIBRARIES
#include <Wire.h>
#include <LiquidCrystal_I2C.h> //LCD display
#include <SPI.h>// Communication with SD card on REVB shield
#include <SD.h> //SD card
#include <Average.h> //Data averages
#include <RTClib.h> // Real Time Clock
#include <SimpleModbusSlave.h> //Modbus
#include <EEPROM.h>
#include <avr/wdt.h>

// Background
#define LED 41/// for modbus testing but not used  

int lcd_state;
int lcd_state2;

// Setup for Screen

#define BACKLIGHT_PIN (3) /// keep (not sure for reason)
#define LED_ADDR (0x27) // might need to be 0x3F, if 0x27 doesn't work for Sainsmart LCD display
LiquidCrystal_I2C lcd(LED_ADDR, 2, 1, 0, 4, 5, 6, 7, BACKLIGHT_PIN, POSITIVE) ;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Setup for RTC datalogging chip

RTC_PCF8523 rtc;
const int chipSelect = 10;
const int Pin53 = 53; // ???for SD writing perhaps

///SET THESE FOR USER
File dataFile;//datafile name for co2 logger sd chip (change to be on eprom)
//int sl_id= 71;// slave id for modbus (use for testing if eeprom not written)
int sl_id = EEPROM.read(0);

//millis timers
unsigned long previousMillis1= 0;
unsigned long previousMillis2= 0;
unsigned long previousMillis3= 0;
unsigned long previousMillis4= 0;
unsigned long previousMillis5= 0;
unsigned long previousMillis6= 0;
unsigned long previousMillis7= 0;
unsigned long previousMillis8= 0;
unsigned long previousMillis9= 0;
unsigned long previousMillis10= 0; // software resest
unsigned long previousMillis11= 0; 
unsigned long previousMillis12= 0;
unsigned long previousMillis13= 0; 
unsigned long previousMillis14= 0; 
unsigned long intervalheating = 100000;
unsigned long intervalheating2 = 100000;

// Arduino code to run Therm109 sensors from Campbell Scientific.
// Steinhart/ Hart coefficients for the thermistor (usually 3000-4000) change for different thermistors

/// from Campbell Scientific 109 thermistor manual
#define A_val 0.001129241
#define B_val 0.0002341077
#define C_val 0.00000008775468

// the value of the 'other' resistor
#define SERIESRESISTOR 24900

//function for reading thermistors (Analogport, numbersamples, variable_name )
//Temperature variables (and remaining register variables)
unsigned intervalTemp = 15000; // in milliseconds

float pinTemp =-7999; // control pin temperature in Celcius
float lagTemp=-7999; // lagging temperature in Celcius
float surfaceTemp=-7999; // surface temperature in Celcius
float deepTemp=-7999; //  deep soil temperature in Celcius
float aboveTemp=-7999; // aboveground surface temperature in Celcius
float airTemp=-7999; // air temperature
float Arduino_Code; // TO CS 7 digit code for sending error information

float dutyDeep=0; // from CS (interval time on for each heater cycle for PIN)
float dutySurface=0;// from CS (interval time on for each heater cycle for SURFACE)
float CS_Code=5; // Function code for sending control information to Arduino
float time_cs; // posix time from CS
float p_time_cs;
float pinTemp_avg =-7999; // from CS (ambient spatal ambient for PIN)
float surfaceTemp_avg =-7999;// from CS (ambient spatal ambient for SURFACE)

float STavg_5; //.push(variable) averages
float PTavg_5;
float LTavg_5;
float ATavg_5;
float DTavg_5;
float AiTavg_5;

float STsd_5; //.push(variable) standard deviation
float PTsd_5;
float LTsd_5;
float ATsd_5;
float DTsd_5;
float AiTsd_5;

Average <float> ST_5(20);//  3 minute moving window data vector
Average <float> PT_5(20);
Average <float> LT_5(20);
Average <float> AT_5(20);
Average <float> DT_5(20);
Average <float> AiT_5(20);

int StateDeep = LOW;
int StateSurface = LOW;
int StateDIAG = LOW;
int StateALARM = LOW;


String Alarmstate1;
String Alarmstate2;

unsigned long dutyDeep2;
unsigned long dutySurface2;
unsigned long dutyDeepflex;
unsigned long dutySurfaceflex;
unsigned long comstate;

float surface_dev; //
float pin_dev; //

/// for modbus regristries (each register is 16bit but encoded as 32bit float;

enum
{ aa,// Pin_raw (TO CS)
  ab,
  ba,// Lag_raw(TO CS)
  bb,
  ca,// Surface_raw(TO CS)
  cb,
  da,// Deep_raw(TO CS)
  db,
  ea,// Above_raw(TO CS)
  eb,
  fa,// Air_raw(TO CS)
  fb,
  ga,// Arduino_Code(TO CS)
  gb,
  ha,// time_CS (from CS)
  hb,
  ia,// dutyDeep(from CS)
  ib,
  ja,// dutySurface(from CS)
  jb,
  ka,// CS_code(from CS)
  kb,
  la,// pinTemp_avg(from CS)
  lb,
  ma,// surface_avg(from CS)
  mb,
  HOLDING_REGS_SIZE// leave this one
  // total number of registers for function 3 and 16 share the same register array
  // i.e. the same address space
};

/// variable encoding for modbus
uint16_t holdingRegs[HOLDING_REGS_SIZE]; // function 3 and 16 register array
////////////////////////////
union Pun {
  float f;
  uint32_t u;
};
///for variables to logger

void encodeFloat(uint16_t *holdingRegs, float x)
{
  union Pun pun;
  pun.f = x;
  holdingRegs[0] = (pun.u >> 16) & 0xFFFFU;
  holdingRegs[1] = pun.u & 0xFFFFU;
}

///for variables from logger
float decodeFloat(uint16_t *holdingRegs)
{
  union Pun pun;
  pun.u = ((uint32_t)holdingRegs[0] << 16) | holdingRegs[1];
  return pun.f;

}

//void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
//{
//  asm volatile ("  jmp 0");
//}


void softwareReset( uint8_t prescaller) {
  // start watchdog with the provided prescaller
  wdt_enable( prescaller);
  // wait for the prescaller time to expire
  // without sending the reset signal by using
  // the wdt_reset() method
  while(1) {}
}
 
//void softwareReset2(unsigned long delayMillis) {
//  uint32_t resetTime = millis() + delayMillis;
//  while ( resetTime > millis()) { /* wait and do nothing... */}
//  // set digital pin 7 to LOW - reset the MCU
//  digitalWrite(31, LOW);
//}



//function for collecting temperature data
float Therm109(char j, int y) {
  //analogReference(EXTERNAL);
  uint8_t i;
  float average;
  int samples[y];
  // take N samples in a row, with a slight delay
  for (i = 0; i < y; i++) {
    samples[i] = analogRead(j);
    delay(20);
  }
  // average all the but the first samples
  average = 0;
  for (i = 0; i < y; i++) {
    average += samples[i];
  }
  average /= y;

  //  //Serial.print("Average analog reading ");
  //  //Seria1.println(average);
  //  // convert the value to resistance

  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;

  //  //Serial.print("Thermistor resistance ");
  //  //Serial.print(j);
  //  //Serial.println(average);
  //
  float steinhart;
  float ln_rs;
  ln_rs = log (average);
  steinhart = (1 / (A_val + B_val * ln_rs + C_val * pow((ln_rs), 3)) - 273.15);
  //
  //  //Serial.print("Temperature ");
  //  //Serial.print(" ");
  //  //Serial.print(steinhart);
  //  //Serial.println(" *C");
  //   //delay(1000);
  //
  return steinhart;
  average = 0;
}

void setup()
{

//code for hard reset 
//set digital pin 31 mode to OUTPUT 
// it is connected to reset pin via 1kOHM resistor 
//  pinMode( 31, OUTPUT);
//  digitalWrite( 31, HIGH);

analogReference(EXTERNAL);
  //modbus control
  /* parameters(HardwareSerial* SerialPort,
                 long baudrate,
                 unsigned char byteFormat,
                 unsigned char ID,
                 unsigned char transmit enable pin,
                 unsigned int holding registers size,
                 unsigned int* holding register array)
    Valid modbus byte formats are:
      SERIAL_8N2: 1 start bit, 8 data bits, 2 stop bits
      SERIAL_8E1: 1 start bit, 8 data bits, 1 Even parity bit, 1 stop bit
      SERIAL_8O1: 1 start bit, 8 data bits, 1 Odd parity bit, 1 stop bit

      You can obviously use SERIAL_8N1 but this does not adhere to the
      Modbus specifications. That said, I have tested the SERIAL_8N1 option
      on various commercial masters and slaves that were suppose to adhere
      to this specification and was always able to communicate... Go figure.
      These byte formats are already defined in the Arduino global name space.
  */
  //basic command structure for modbus communications
  //void modbus_configure(long baud, unsigned char _slaveID, unsigned char _TxEnablePin, unsigned int _holdingRegsSize, unsigned char _lowLatency)

  modbus_configure(&Serial1, 9600, SERIAL_8N1, sl_id, 41, HOLDING_REGS_SIZE, holdingRegs);
  // modbus_update_comms(baud, byteFormat, id) is not needed but allows for easy update of the
  // port variables and slave id dynamically in any function.
  modbus_update_comms(9600, SERIAL_8N1, sl_id);


  //Serial.begin(9600); //this is for usb com port usage
  //pinMode (BACKLIGHT_PIN, OUTPUT );
  //digitalWrite (BACKLIGHT_PIN, HIGH );

  //lcd.clear();
  lcd.begin(20, 4);              // initialize the lcd
  lcd.home ();

  if (! rtc.begin()) {
    lcd.setCursor ( 0, 0);        // go to the next line
    lcd.print("Couldn't find RTC"); /// real time clock
    while (1);
  }
  if (rtc.begin()) {
    lcd.setCursor ( 0, 0);        // go to the next line
    lcd.print("Found RTC"); /// real time clock
  }
  if (! rtc.initialized()) {
    lcd.setCursor ( 0, 1 );        // go to the next line
    lcd.print("RTC problem");
  }
  if (rtc.initialized()) {
    lcd.setCursor ( 0, 1 );        // go to the next line
    lcd.print("RTC good");
  }

delay (1000);/// why is this not on timer


  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  // sd chip code
  //lcd.setCursor ( 0, 2 );        // go to the next line
  //lcd.println("Init SD...");
  //  make sure that the default chip select pin is set to
  //  output, even if you don't use it:
  
pinMode(SS, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    lcd.clear();
    lcd.setCursor (0, 2 );        // go to the next line
    lcd.print("Card Fail");
    // don't do anything more:
    while (1) ;
  }
  if (SD.begin(chipSelect)) {
    lcd.clear();
    lcd.setCursor (0, 2);     // go to the next line
    lcd.print("CARD PASS");
  }

  // Open up the file we're going to log to
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (! dataFile) {
    lcd.setCursor(0, 3);
    lcd.println("error datalog.txt");
    // Wait forever since we cant write data
    while (1);
  }
  if (dataFile) {
    lcd.setCursor(0, 3);
    lcd.print("datalog.txt cool");
  }
  delay(2000);

  //
  // Switch on the lcd and lcd backlight

  lcd.clear();
  //lcd.begin(20, 4);              // initialize the lcd
  lcd.home ();                   // go home
  lcd.print("MERIT Experiment");
  lcd.setCursor ( 0, 1 );        // go to the next line
  lcd.print (program);
  lcd.setCursor ( 0, 2 );        // go to the next line
  lcd.print ("Moin Moin");
  lcd.setCursor ( 0, 3 );        // go to the next line
  lcd.print ("Climate Science");

  delay(2000);
  
  //CHECK CHANGES MADE
  pinMode(36, OUTPUT); // PWM for pin (deep) // black
  pinMode(37, OUTPUT); // PWM for pin backup
  pinMode(38, OUTPUT); // PWM for surface
  pinMode(39, OUTPUT); // PWM for surface backup // white
  pinMode(46, OUTPUT); // PWM diagnostic 50% on 20 second interval
  pinMode(47, OUTPUT); // PWM  diagnostic 50% on 20 second interval
  pinMode(12, OUTPUT); // ALARM LIGHT for overheating

} /// end setup loop

/// Programming loop starts here

void loop()
{
//For freezing issues  
unsigned long currentMillis10 = millis(); 
if (currentMillis10 - previousMillis10 >= 7200000) { // resets every 2 hours
  // restart in 4 seconds
  softwareReset( WDTO_4S);      
  previousMillis10 = currentMillis10;
 }

//Hard reset code
//  unsigned long currentMillis6 = millis(); 
//  if (currentMillis10 - previousMillis10 >= 9000000) { //  hacker resets every 1.5 hours using 1kohm resistor between reset port and pin 31 
//  // restart in 60 milliseconds
//  softwareReset2(60);      
//   }

unsigned long currentMillis12 = millis(); // collection every 15 seconds
  if (currentMillis12 - previousMillis12 >= 30000) {

  //function set time when more than 30 seconds different
  DateTime now = rtc.now();
  double t = now.unixtime();
  double t2 = t - time_cs;
  t2 = abs(t2);
  if (t2 > 100 && time_cs != 0) {
    rtc.adjust(time_cs);
  }
  previousMillis12 = currentMillis12;
  }
  
 /////Temperature collections
  unsigned long currentMillis3 = millis(); // collection every 15 seconds
  if (currentMillis3 - previousMillis3 >= intervalTemp) {

    pinTemp = Therm109(A8, 10); //Returns value in celcius
    delay(100);
    lagTemp = Therm109(A9, 10);
    delay(100);
    surfaceTemp = Therm109(A10, 10);
    delay(100);
    deepTemp = Therm109(A11, 10);
    delay(100);
    aboveTemp = Therm109(A12, 10);
    delay(100);
    airTemp = Therm109(A13, 10);
    delay(100);

/// change ranges in 2019 and direction of greater than sizes to enable

    if ( surfaceTemp > -25 and surfaceTemp < 55) {
    ST_5.push(surfaceTemp);} /// send newest data to vector for moving averages and other statistics
    
    if ( pinTemp > -25 or pinTemp < 60) {
    PT_5.push(pinTemp);}

    if ( lagTemp > -25 or lagTemp < 60) {
    LT_5.push(lagTemp);}
    
    if ( aboveTemp > -25 or aboveTemp < 60) {
    AT_5.push(aboveTemp);}
    
    if ( deepTemp > -25 or deepTemp < 60) {
    DT_5.push(deepTemp);}
    
    if ( airTemp > -25 or airTemp < 60) {
    AiT_5.push(airTemp);}

    STavg_5 = ST_5.mean(); /// averages surfaceTemp 5 min
    PTavg_5 = PT_5.mean(); /// averages pinTemp 5 min
    LTavg_5 = LT_5.mean(); /// averages lagTemp 5 min
    ATavg_5 = AT_5.mean(); /// averages aboveTemp 5 min
    DTavg_5 = DT_5.mean(); /// averages deepTemp 5 min
    AiTavg_5 = AiT_5.mean(); /// averages deepTemp 5 min

    STsd_5 = ST_5.stddev(); /// sd surfaceTemp 5 min
    PTsd_5 = PT_5.stddev(); /// sd pinTemp 5 min
    LTsd_5 = LT_5.stddev(); /// sd lagTemp 5 min
    ATsd_5 = AT_5.stddev(); /// sd aboveTemp 5 min
    DTsd_5 = DT_5.stddev(); /// sd deepTemp 5 min
    AiTsd_5 = AiT_5.stddev(); /// sd deepTemp 5 min


    ///Arduino Codes
    Arduino_Code = 0;

    if (PTsd_5 > .5) {
      Arduino_Code = Arduino_Code + 1;
    }
    if (PTsd_5 > 1) {
      Arduino_Code = Arduino_Code + 1;
    }
    if ( pinTemp > 55 or pinTemp < -55) {
      Arduino_Code = Arduino_Code + 3;
      pinTemp = -7999;
    }

    if (LTsd_5 > .5) {
      Arduino_Code = Arduino_Code + 10;
    }
    if (LTsd_5 > 1) {
      Arduino_Code = Arduino_Code + 10;
    }
    if (lagTemp > 55 or lagTemp < -55) {
      Arduino_Code = Arduino_Code + 30;
      lagTemp = -7999;
    }

    if (STsd_5 > .5) {
      Arduino_Code = Arduino_Code + 100;
    }
    if (STsd_5 > 1) {
      Arduino_Code = Arduino_Code + 100;
    }
    if (surfaceTemp > 55 or surfaceTemp < -55) {
      Arduino_Code = Arduino_Code + 300;
      surfaceTemp = -7999;
    }

    if (DTsd_5 > .5) {
      Arduino_Code = Arduino_Code + 1000;
    }
    if (DTsd_5 > 1) {
      Arduino_Code = Arduino_Code + 1000;
    }
    if (deepTemp > 55 or deepTemp < -55) {
      Arduino_Code = Arduino_Code + 3000;
      deepTemp = -7999;
    }

    if (ATsd_5 > .5) {
      Arduino_Code = Arduino_Code + 10000;
    }
    if (ATsd_5 > 1) {
      Arduino_Code = Arduino_Code + 10000;
    }
    if (deepTemp > 55 or  aboveTemp < -55) {
      Arduino_Code = Arduino_Code + 30000;
      aboveTemp = -7999;
    }

    if (AiTsd_5 > .5) {
      Arduino_Code = Arduino_Code + 100000;
    }
    if (AiTsd_5 > 1) {
      Arduino_Code = Arduino_Code + 100000;
    }
    if (airTemp > 55 or airTemp < -55) {
      Arduino_Code = Arduino_Code + 300000;
      airTemp = -7999;
    }


    /*reading of coil where i can succefully read value*/
    encodeFloat(&holdingRegs[aa], pinTemp); // temperature variables to send
    encodeFloat(&holdingRegs[ba], lagTemp);
    encodeFloat(&holdingRegs[ca], surfaceTemp);
    encodeFloat(&holdingRegs[da], deepTemp);
    encodeFloat(&holdingRegs[ea], aboveTemp);
    encodeFloat(&holdingRegs[fa], airTemp);
    encodeFloat(&holdingRegs[ga], Arduino_Code); // for trouble shooting
    
    dutyDeep = decodeFloat(&holdingRegs[ha]); //duty cycle for deep heating
    dutySurface = decodeFloat(&holdingRegs[ia]); //duty cycle for surface heating
    CS_Code = decodeFloat(&holdingRegs[ja]); //control code fromcs
    pinTemp_avg =  decodeFloat(&holdingRegs[ka]); //Ambient for comparison with other plots heating operations
    surfaceTemp_avg = decodeFloat(&holdingRegs[la]); //time from Campbell logger
    time_cs =  decodeFloat(&holdingRegs[ma]); //Ambient comparison with other plots check on heating operations
    encodeFloat(&holdingRegs[ja], CS_Code); // for trouble shooting

  //Flex heating
  pin_dev = pinTemp_avg - pinTemp;
  surface_dev= surfaceTemp_avg- surfaceTemp;
  ///// changes more negative when average is higher than measured temperature.
  ///// changes duty cycle by ten percent when in flex mode to keep on target

  
  if(-0.5< pin_dev < 0.5 ) {dutyDeepflex= dutyDeep;}
  if(pin_dev > 0.5) {dutyDeepflex= (dutyDeep * 1.10);} // underheating
  if(pin_dev < -0.5) {dutyDeepflex= (dutyDeep * -1.10);} // overheating
  if(0< dutyDeepflex <10,000){dutyDeepflex= 10000;}

  if(-0.5< surface_dev < 0.5 ) {dutySurfaceflex = dutySurface;}
  if(surface_dev > 0.5) {dutySurfaceflex= (dutySurface * 1.10);} // underheating
  if(surface_dev < -0.5) {dutySurfaceflex= (dutySurface * -1.10);} // overheating
  if(0< dutySurfaceflex <10,000){dutySurfaceflex= 10000;}

  
  previousMillis3 = currentMillis3;
  }

 //ALARMS

//pin_dev =-1;
//surface_dev =0;
   
  if(-1.5 < pin_dev < 1.5){Alarmstate1 =  "pin-ok";}
  if(-2.0< pin_dev <= -1.5 ){Alarmstate1 = "pin-over";} // check overheat
  if(pin_dev <= -2.0 ){Alarmstate1 = "pin-check";} // check immediatedly
  if(pin_dev> 1.5 and pin_dev <= 2.0){Alarmstate1 = "pin-under";} // plot cooler than neigbors
  if(pin_dev >= 2.0 ){Alarmstate1 ="pin-check";} // plot maybe off or cooler than neighbors 
  
  if(-1.5< surface_dev <= 1.5) {Alarmstate2 ="sur-ok";} // routine checks
  if(-2.0< surface_dev <= -1.5 ){Alarmstate2="sur-over";} // check overheat
  if(surface_dev <= -2.0 ){Alarmstate2="sur-check";} // check immediatedly
  if(surface_dev > 1.5 and surface_dev <= 2.0){Alarmstate2="sur-under";} // plot cooler than neigbors
  if(surface_dev >= 2.0 ){Alarmstate2="sur-check";} // plot maybe off or cooler than neighbors 

 modbus_update();//must be in main loop. 
  
  // (not implemented add mode for secondary registry, set for time separate time cycle, every 60 seconds))

  /// communications failure shutdown after 10 minutes
  
 unsigned long currentMillis9 = millis(); 
  
 if (currentMillis9 - previousMillis9 >= 600000) { 
    CS_Code == 5;
    previousMillis9 = currentMillis9;}
    
  if (CS_Code == 0) {
    dutyDeep2 = 0;  /// all off
    dutySurface2 = 0;
  }
  if (CS_Code == 1) {
    dutyDeep2 = dutyDeep;  /// deep on, soils off
    dutySurface2 = 0;
  }
  if (CS_Code == 2) {
    dutyDeep2 = 0;  /// deep on, soils off
    dutySurface2 = dutySurface;
  }
  if (CS_Code == 3) {
    dutyDeep2 = dutyDeep;  /// deep on, soils on
    dutySurface2 = dutySurface;
  }
  if (CS_Code == 4) {
    dutyDeep2 = dutyDeepflex;  /// deep on, soils off
    dutySurface2 = dutySurfaceflex;
  }


  //Heating control
  //Deep Soils


//  //Test
//  CS_Code = 3;
//  dutyDeep = 5000;
//  dutySurface = 2000;

  unsigned long currentMillis7 = millis();
  unsigned long startT1 = intervalheating - dutyDeep2;

if (CS_Code == 5) {
    StateDeep = LOW;
 } else {
  
if (currentMillis7 - previousMillis7 <= startT1) {
    StateDeep = LOW;
  } else {
    StateDeep = HIGH;
  }

 if (currentMillis7 - previousMillis7 >= intervalheating) {
    StateDeep = LOW;
    previousMillis7 = currentMillis7;
  }
  }




  // Surface Soils
  unsigned long currentMillis8 = millis();
  unsigned long startT2 = intervalheating2 - dutySurface2;

if (CS_Code == 5) {
    StateSurface = LOW;
 
 } else {

  if (currentMillis8 - previousMillis8 <= startT2) {
    StateSurface = LOW;
  } else {
    StateSurface = HIGH;
  }

  if (currentMillis8 - previousMillis8 >=intervalheating2) {
    StateSurface = LOW;
  previousMillis8 = currentMillis8;
  }
 }
  digitalWrite(36, StateDeep);
  digitalWrite(37, StateDeep);
  digitalWrite(38, StateSurface);
  digitalWrite(39, StateSurface);

///2019 Diagnostic mode additions turns these ports on and off evrey ten seconds for use testing crydom units without CS connections.
  unsigned long currentMillis13 = millis();
  if (currentMillis13 - previousMillis13 > 20000) {
   previousMillis13 = currentMillis13;
   StateDIAG=LOW;}
  
if (currentMillis13 - previousMillis13 <= 10000) {
      StateDIAG = HIGH;
      } else {
      StateDIAG = LOW;
   }
  
  digitalWrite(46, StateDIAG);
  digitalWrite(47, StateDIAG);

 
 

/// 2019 addition

if(surface_dev >= 2.0 or surface_dev <= -2.0 or pin_dev >= 2.0 or pin_dev <= -2.0){
unsigned long currentMillis14 = millis();

if (currentMillis14 - previousMillis14 > 2000) {
   previousMillis14 = currentMillis14;
   StateALARM=LOW;}
  
if (currentMillis14 - previousMillis14 <= 1000) {
      StateALARM = HIGH;
      } else {
      StateALARM = LOW;
   } 
  } 
  digitalWrite(12, StateALARM);
 

unsigned long currentMillis11 = millis();
  if (currentMillis11 - previousMillis11 <= 20001) {
  
  unsigned long currentMillis4 = millis();
  if (currentMillis4 - previousMillis4 <= 5000) {
    lcd_state = 0;
  }
  if (currentMillis4 - previousMillis4 > 5000) {
    lcd_state = 1;
  }
  if (currentMillis4 - previousMillis4 >= 10000) {
    lcd_state = 2;
  }
  if (currentMillis4 - previousMillis4 >= 15000) {
    lcd_state = 3;
  }
  if (currentMillis4 - previousMillis4 >= 20000) {
    lcd_state2 = 0;
    lcd_state = 0;
    previousMillis4 = currentMillis4;
  }


  //ALARMS

//pin_dev =10;
//surface_dev =10;
   
  if(-1.5< pin_dev <= 1.5){String Alarmstate1 = "pin-ok";
  }
  if(-2.0< pin_dev <= -1.5 ){String Alarmstate1="pin-over";
  } // check overheat
  
  if(pin_dev <= -2.0 ){String Alarmstate1= "pin-check";} // check immediatedly
  if(1.5 < pin_dev <= 2.0){String Alarmstate1= "pin-under";} // plot cooler than neigbors
  if(pin_dev >= 2.0 ){String Alarmstate1 ="pin-check";} // plot maybe off or cooler than neighbors 
  
  if(-1.5< surface_dev <= 1.5) {String Alarmstate2 ="sur-ok";} // routine checks
  if(-2.0< surface_dev <= -1.5 ){String Alarmstate2="sur-over";} // check overheat
  if(surface_dev <= -2.0 ){String Alarmstate2="sur-check";} // check immediatedly
  if(1.5 < surface_dev <= 2.0){String Alarmstate2="sur-under";} // plot cooler than neigbors
  if(surface_dev >= 2.0 ){String Alarmstate2="sur-check";} // plot maybe off or cooler than neighbors 

 // String Alarmstate2="sur-check";
 // String Alarmstate1="sur-check";
  

  if (lcd_state == 0 and lcd_state2 == 0) {
    
    lcd.clear();
    DateTime now = rtc.now();
    lcd.setCursor (0, 0);        // go to the next line
    lcd.print(now.year(), DEC);
    lcd.print('/');
    lcd.print(now.month(), DEC);
    lcd.print('/');
    lcd.print(now.day(), DEC);
    lcd.print(' ');
    lcd.print(now.hour(), DEC);
    lcd.print(':');
    lcd.print(now.minute(), DEC);
    lcd.print(':');
    lcd.print(now.second(), DEC);
    lcd.setCursor (0, 1);        // go to the next line
    lcd.print ("Pin 'C =");
    lcd.print(pinTemp);
    lcd.setCursor (0, 2);
    lcd.print ("Lag 'C =");
    lcd.print(lagTemp);
    lcd.setCursor (0, 3);
    lcd.print ("Sur 'C =");
    lcd.print(surfaceTemp);
    lcd_state2 = 1;
  }

  if (lcd_state == 1 and lcd_state2 == 1) {
    lcd.clear();
    lcd.setCursor (0, 0);        // go to the next line
    lcd.print ("Deep 'C =");
    lcd.print(deepTemp);
    lcd.setCursor (0, 1);        // go to the next line
    lcd.print ("Above 'C =");
    lcd.print(aboveTemp);
    lcd.setCursor (0, 2);        // go to the next line
    lcd.print ("Air 'C =");
    lcd.print(airTemp);
    lcd.setCursor (0, 3);        // go to the next line
    lcd.print ("Ard_Code = ");
    lcd.print(Arduino_Code);
    lcd_state2 = 2;

  }

  if (lcd_state == 2 and lcd_state2 == 2) {
    lcd.clear();
    lcd.setCursor (0, 0);        // go to the next line
    lcd.print ("De_Duty =");
    lcd.print(dutyDeep2);
    lcd.setCursor (0, 1);        // go to the next line
    lcd.print ("Su_Duty =");
    lcd.print(dutySurface2);
    lcd.setCursor (0, 2);        // go to the next line
    lcd.print ("CS_Code =");
    lcd.print(CS_Code);
    lcd.setCursor (0, 3);        // go to the next line
    lcd.print ("TCS=");
    lcd.print(time_cs);
    lcd_state2 = 3;
  }

  if (lcd_state == 3 and lcd_state2 == 3) {
    lcd.clear();
    lcd.setCursor (0, 0);        // go to the next line
    lcd.print ("Pin_avg =");
    lcd.print(pinTemp_avg);
    lcd.setCursor (0, 1);        // go to the next line
    lcd.print ("Sur_avg =");
    lcd.print(surfaceTemp_avg);
    lcd.setCursor (0, 2);        // go to the next line
    lcd.print (Alarmstate1);
    lcd.print (" ");
    lcd.print (Alarmstate2);
    lcd.setCursor (0, 3);        // go to the next line
    lcd.print ("Slave_ID =");
    lcd.print (sl_id);
    lcd_state2 = 4;
  }

 previousMillis11 = currentMillis11;
  }

  unsigned long currentMillis5 = millis();
  if (currentMillis5 - previousMillis5 >= 300000) {
    DateTime now = rtc.now();
    digitalWrite(Pin53, HIGH);
    double t = now.unixtime();
    dataFile.print(t);
    dataFile.print(",");
    dataFile.print(now.year(), DEC);
    dataFile.print('/');
    dataFile.print(now.month(), DEC);
    dataFile.print('/');
    dataFile.print(now.day(), DEC);
    dataFile.print(' ');
    dataFile.print(now.hour(), DEC);
    dataFile.print(':');
    dataFile.print(now.minute(), DEC);
    dataFile.print(':');
    dataFile.print(now.second(), DEC);
    dataFile.print(",");
    dataFile.print(sl_id);
    dataFile.print(",");
    dataFile.print(program);
    dataFile.print(",");

    dataFile.print(pinTemp);
    dataFile.print(",");
    dataFile.print(lagTemp);
    dataFile.print(",");
    dataFile.print(surfaceTemp);
    dataFile.print(",");
    dataFile.print(deepTemp);
    dataFile.print(",");
    dataFile.print(aboveTemp);
    dataFile.print(",");
    dataFile.print(airTemp);
    dataFile.print(",");

    dataFile.print(PTavg_5);
    dataFile.print(",");
    dataFile.print(LTavg_5);
    dataFile.print(",");
    dataFile.print(STavg_5);
    dataFile.print(",");
    dataFile.print(DTavg_5);
    dataFile.print(",");
    dataFile.print(ATavg_5);
    dataFile.print(",");
    dataFile.print(AiTavg_5);
    dataFile.print(",");

    dataFile.print(PTsd_5);
    dataFile.print(",");
    dataFile.print(LTsd_5);
    dataFile.print(",");
    dataFile.print(STsd_5);
    dataFile.print(",");
    dataFile.print(DTsd_5);
    dataFile.print(",");
    dataFile.print(ATsd_5);
    dataFile.print(",");
    dataFile.print(AiTsd_5);
    dataFile.print(",");
    dataFile.print(dutySurface);
    dataFile.print(",");
    dataFile.print(dutyDeep);
    dataFile.print(",");

    dataFile.print(Arduino_Code);
    dataFile.print(",");
    dataFile.print(CS_Code);
    dataFile.print(",");
    dataFile.print(time_cs);
    dataFile.println();
    dataFile.flush();
    previousMillis5 = currentMillis5;
  }

} // ending bracket


