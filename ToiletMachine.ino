#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <HX711_ADC.h>
// #if defined(ESP8266)|| defined(ESP32) || defined(AVR)
// #include <EEPROM.h>
// #endif

//pins:
const uint8_t HX711_dout_1 = 3; //mcu > HX711 no 1 dout pin
const uint8_t HX711_sck_1 = 2; //mcu > HX711 no 1 sck pin
const uint8_t HX711_dout_2 = 4; //mcu > HX711 no 2 dout pin
const uint8_t HX711_sck_2 = 37; //mcu > HX711 no 2 sck pin not sure that these can be the same
const uint8_t HX711_dout_3 = 5; //mcu > HX711 no 1 dout pin
const uint8_t HX711_sck_3 = 29; //mcu > HX711 no 1 sck pin
const uint8_t HX711_dout_4 = 6; //mcu > HX711 no 1 dout pin
const uint8_t HX711_sck_4 = 22; //mcu > HX711 no 1 sck pin

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); //HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); //HX711 2
// int LoadCell_4;
// int LoadCell_3;
HX711_ADC LoadCell_3(HX711_dout_3, HX711_sck_3); //HX711 3
HX711_ADC LoadCell_4(HX711_dout_4, HX711_sck_4); //HX711 4

const int calVal_eepromAdress_1 = 0; // eeprom adress for calibration value load cell 1 (4 bytes)
const int calVal_eepromAdress_2 = 4; // eeprom adress for calibration value load cell 2 (4 bytes)
unsigned long t = 0;

// A simple data logger for the Arduino analog pins

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  14 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 2000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

RTC_DS1307 RTC; // define the Real Time Clock object


// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;
// const float calibration_factor = 1000;


// the logging file
File logfile;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  // digitalWrite(redLEDpin, HIGH);

  while(1);
}

void setup(void)
{
  
  
  Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Starting...");

  float calibrationValue_1; // calibration value load cell 1
  float calibrationValue_2; // calibration value load cell 2

  calibrationValue_1 = 7.3; // uncomment this if you want to set this value in the sketch
  calibrationValue_2 = 73.30; // uncomment this if you want to set this value in the sketch
#if defined(ESP8266) || defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch the value from eeprom
#endif
  //EEPROM.get(calVal_eepromAdress_1, calibrationValue_1); // uncomment this if you want to fetch the value from eeprom
  //EEPROM.get(calVal_eepromAdress_2, calibrationValue_2); // uncomment this if you want to fetch the value from eeprom

  LoadCell_1.begin();
  LoadCell_2.begin();
  LoadCell_3.begin();
  LoadCell_4.begin();
  //LoadCell_1.setReverseOutput();
  //LoadCell_2.setReverseOutput();
  unsigned int stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;
  byte loadcell_3_rdy = 0;
  byte loadcell_4_rdy = 0;
  while ((loadcell_1_rdy + loadcell_2_rdy + loadcell_3_rdy + loadcell_4_rdy) < 4) { //run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
    if (!loadcell_3_rdy) loadcell_3_rdy = LoadCell_3.startMultiple(stabilizingtime, _tare);
    if (!loadcell_4_rdy) loadcell_4_rdy = LoadCell_4.startMultiple(stabilizingtime, _tare);
  }
  if (LoadCell_1.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }
  if (LoadCell_2.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }
  if (LoadCell_3.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.3 wiring and pin designations");
  }
  if (LoadCell_4.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.4 wiring and pin designations");
  }
  LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value (float)
  LoadCell_2.setCalFactor(calibrationValue_2); // user set calibration value (float)
  LoadCell_3.setCalFactor(calibrationValue_2); // user set calibration value (float)
  LoadCell_4.setCalFactor(calibrationValue_2); // user set calibration value (float)
  Serial.println("Startup is complete");
  
  
#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  
  // create a new file
  char filename[] = "S01_00.CSV"; // name file however we want
  for (uint8_t i = 0; i < 100; i++) {
    filename[4] = i/10 + '0';
    filename[5] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);

  // connect to RTC
  Wire.begin();  
  if (!RTC.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
  }
  

  logfile.println("millis,Back Left, Back Right, Front Left, Front Right");    
#if ECHO_TO_SERIAL
  Serial.println("millis, Back Left, Back Right, Front Left, Front Right");
#endif //ECHO_TO_SERIAL
 
  // If you want to set the aref to something other than 5v
  // analogReference(EXTERNAL);
}

void loop(void)
{
  bool flag1;
  bool flag2;
  bool flag3;
  bool flag4;
  float a;
  float b;
  float c;
  float d;

  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell_1.update()) {
    a = LoadCell_1.getData();
    flag1 = true;
  }
  if (LoadCell_2.update()) {
    b = LoadCell_2.getData();
    flag2 = true;
  }
  if (LoadCell_3.update()) {
    c = LoadCell_3.getData();
    flag3 = true;
  }
  if (LoadCell_4.update()) {
    d = LoadCell_4.getData();
    flag4 = true;
  }

  //get smoothed value from data set
  if ((flag1 && flag2 && flag3 && flag4)) {
    if (millis() > t + serialPrintInterval) {
      logfile.print(millis());
      logfile.print(", ");
      logfile.print(a);
      logfile.print(", ");
      logfile.print(b);
      logfile.print(", ");
      logfile.print(c);
      logfile.print(", ");
      logfile.println(d);
      #if ECHO_TO_SERIAL
        Serial.print(millis());
        Serial.print(", ");
        Serial.print(a);
        Serial.print(", ");
        Serial.print(b);
        Serial.print(", ");
        Serial.print(c);
        Serial.print(", ");
        Serial.println(d);
      #endif
      flag1 = false;
      flag2 = false;
      flag3 = false;
      flag4 = false;
      // newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') {
      LoadCell_1.tareNoDelay();
      LoadCell_2.tareNoDelay();
      LoadCell_3.tareNoDelay();
      LoadCell_4.tareNoDelay();
    }
  }

  //check if last tare operation is complete
  if (LoadCell_1.getTareStatus() == true) {
    Serial.println("Tare load cell 1 complete");
  }
  if (LoadCell_2.getTareStatus() == true) {
    Serial.println("Tare load cell 2 complete");
  }
   if (LoadCell_3.getTareStatus() == true) {
    Serial.println("Tare load cell 2 complete");
  }
   if (LoadCell_4.getTareStatus() == true) {
    Serial.println("Tare load cell 2 complete");
  }
  
//   logfile.println();
// #if ECHO_TO_SERIAL
//   Serial.println();
// #endif // ECHO_TO_SERIAL


  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  
  // blink LED to show we are syncing data to the card & updating FAT!
  // digitalWrite(redLEDpin, HIGH);
  logfile.flush();
  // digitalWrite(redLEDpin, LOW);
  
}