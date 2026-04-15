#include <Arduino.h>
//Libraries
#include <EEPROM.h>
#include <TimeLib.h>  //Time library
#include <TimeAlarms.h>
#include <Wire.h>              // I2C library
#include <Adafruit_GFX.h>      // OLED graphics library
#include <Adafruit_SSD1306.h>  // OLED library
#include <IRremote.hpp> // include the library need to install, by shirriff
//#include <IRremote.h>          // Needed to enable line 51 in IRremote.hpp (#define SUPPRESS_ERROR_MESSAGE_FOR_BEGIN, files on my mac in /Users/.../Library/Arduino15/packages/teensy/hardware/avr/1.58.1/libraries/IRremote/src), also needed to change some variables, mainly "irrecv" to "IRreceiver" (see for guidance: https://github.com/Arduino-IRremote/Arduino-IRremote?tab=readme-ov-file#converting-your-2x-program-to-the-4x-version)
//Uncomment line 174 and open serial monitor while pressing remote buttons to see what codes each IR button is generating. Need to make sure code IDs are used in IR definitions
#include <SdFat.h>     //sd cart library needed to make some updates to variables, but all done in code here
#include <SD.h>        //sd library
#include <SPI.h>       // Serial library
#include <ADC.h>       //Voltage divider read needs this
#include <ADC_util.h>  //Voltage divider read needs this also
#include <Encoder.h> //For pump encoders
#include "MS5837.h"          //Library for Bar30 Depth and Temp sensor, Wire2 not supported by 1.1.1 version of library so replace .h and .cpp file with same from https://github.com/bluerobotics/BlueRobotics_MS5837_Library.git
#include <SoftwareSerial.h>  //include the SoftwareSerial library for atlas scientific EC

//DEFINITIONS
// Low power timer
int timerDonePin = 12;         // Pin for sending done signal to the timer
int powerOnPin = 17;           // Pin for sending signal to turn on mosfet for teensy
bool guiMode = false;          // Flag for GUI mode
const long oneMinute = 60000;  // One minute in milliseconds
time_t storedWakeUpTime;
const time_t NO_MORE_SAMPLES = (time_t)(-10);

//Valve declarations
//Make sure you set current limit on stepper driver before powering stepper motor (keep detached)
//With the TB67S249FTG Driver VREF = Current Limit/1.25, set current limit of 1.68A  so VREF = 1.68/1.25 = 1.344V
#define stepperOn 33            //Drive high to turn on mosfet and send power to stepper driver
#define stepDelay 1             //time it takes to step in microseconds
#define photoHomePin 15         // Pin that powers the photogate
#define stepperDir 27
#define stepperStep 28
#define stepperReset 29  //reset pin not enable, enable is needed to sleep
#define stepperMode2 30
#define stepperMode1 31
#define stepperMode0 32
const uint8_t stepperEncAPin = 10;
const uint8_t stepperEncBPin = 11;
volatile int32_t encoderCount = 0;
volatile uint8_t lastState = 0;
// Quadrature transition table
const int8_t quadTable[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};
const int valvePositions = 16;  //16 valve positions
const int valveTargets[valvePositions] = {0, 6, 12, 18, 24, 30, 36, 42, 48, 54, 60, 66, 72, 78, 84, 90};


const int SETTLE_DELAY_MS = 500;
const int STEP_DELAY_US = 1000;  // microseconds between steps
const int TARGET_POSITION = 96;  // 24 detents but quad encoder so 96 detections.
//RotaryEncoder encoder(stepperEncAPin, stepperEncBPin, RotaryEncoder::LatchMode::FOUR3);
int sampleValve = 255;
int valveSelection = 0;
// Global variable to store last stepper direction
int lastDirection = 0;  // 1 for forward, -1 for backward, 0 for initial state
bool directionChange = false;  // Flag to indicate if direction has changed
int currentValvePositionIndex = 1;  // Active valve position (1-16)
int selectedValvePositionIndex = currentValvePositionIndex; // Intermediate variable

//EEPROM
int eepromWakeUpAddress = 0;                                       // EEPROM address to store wakeup time
int eepromValvePosAddress = eepromWakeUpAddress + sizeof(time_t);  // EEPROM address to store valve position
int eepromReedBaselineAddress = eepromValvePosAddress + sizeof(int);  // EEPROM address to store reed baseline

//Pumps
int pumpsOn = 37;  //Write low to turn on enable pin and allow pump mosfets to run
String messageTemp = "";  // Message for GUI
long cleanVolume = 0.0; 
long cleanVolumeChangeMl = 1.0;

//Pump A 195 RPM Premium Planetary Gear Motor w/Encoder
int pumpARunPin = 21;  // Pin to signal driver to run motor A
int pumpADirPin = 22;  // Pin to signal driver to run cw or ccw
int pumpAEncPowerPin = 20;
int pumpAEncAPin = 8;
int pumpAEncBPin = 9;
Encoder pumpAEnc(pumpAEncAPin, pumpAEncBPin);
float pumpAGearRatio = 61.659;  //rotations of input shaft per full rotation of output shaft
float pumpAEncRate = 739.908;   //pulses per revolution of motor shaft
long pumpAVolume = 0.0;
long pumpAVolumeTemp = 0.0;
long pumpAVolumeChangeMl = 10.0;
int calibVolA = 1;
long calibrationVolumeChangeMl = 1.0;  //Units of change for calibration volume
long calibrationACount = 0.0;
long calibrationAVolume = 1.0;
float pumpARunTime = 0.0;
int pumpADirection = 0;  // CW Direction
int waitforit = 0;
bool threeWayValveOpen = false;
bool waitingForThreeWayClose = false;

//Pump B 195 RPM Premium Planetary Gear Motor w/Encoder
int pumpBRunPin = 41;  // Pin to signal driver to run motor B
int pumpBDirPin = 16;  // Pin to signal driver to run cw or ccw
int pumpBEncPowerPin = 38;
int pumpBEncAPin = 3;
int pumpBEncBPin = 4;
Encoder pumpBEnc(pumpBEncAPin, pumpBEncBPin);
float pumpBGearRatio = 61.659;  //rotations of input shaft per full rotation of output shaft
float pumpBEncRate = 739.908;  //pulses per revolution of motor shaft
long pumpBVolume = 0.0;
long pumpBVolumeChangeMl = 1.0;
int calibVolB = 1;
long calibrationBVolume = 1.0;
long calibrationBCount = 0.0;
float pumpBRunTime = 0.0;
int pumpBDirection = 0;  // CW Direction

//Three-way Valve
int threeWayPin = 6;  //Pin to switch state of 3-way valve, on or not

// OLED (Declaration for an SSD1306 display connected to I2C (SDA, SCL pins))
int oledPowerPin = 40;  // Pin that powers the OLED
int oledData = 18;      //No call out, this is the default, so just here for reference
int oledClk = 19;       //No call out, this is the default, so just here for reference
int activeMenu = 0;     // 0: Status, 1: Settings, 2: Valve
// #define OLED_RESET 12// OLED reset
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
unsigned long lastUpdate = 0;
unsigned long interval = 500;  // Total update interval for 0.5 second.
bool oledEnabled = false;  // Set to false if OLED is not connected

// IR
int irPowerPin = 1;       // Pin that powers the IR sensor, 1 pin on PCB
#define IR_RECEIVE_PIN 0  //0 pin on PCB
//NEED TO UPDATE THE IR HEX NUMBERS (BELOW) TO PROPERLY BE READ BY REMOTE. Uncomment line 172 for further troubleshooting in serial monitor
//Normal Remote Below Hexidecimal
#define LEFTIR 0xF708BF00   // IR signal read by sensor that translates to LEFTIR
#define RIGHTIR 0xF50ABF00  // IR signal read by sensor that translates to RIGHTIR
#define UPIR 0xFA05BF00     // IR signal read by sensor that translates to UPIR
#define DOWNIR 0xF20DBF00   // IR signal read by sensor that translates to DOWNIR
#define ENTERIR 0xF609BF00  // IR signal read by sensor that translates to ENTERIR

//Photo transistor
#define lightPin A0
int lightLevel = 0;

//Conductivity sensor
#define ecRX 34
#define ecTX 35
#define ecOffPin 36
SoftwareSerial myserial(ecRX, ecTX);  //define how the soft serial port is going to work
// Declare global variables
String sensorstring = "";
boolean sensor_string_complete = false;
int menuOption = 0;                  // Tracks the selected menu option
int calibrationStep = 0;             // Tracks calibration step (0: Start, 1: Dry, 2: Low, 3: High)
bool isCalibrating = false;          // Flag to show "Wait..." during calibration
int ecValue = -1;                    // Global variable to store EC value
unsigned long lastECUpdateTime = 0;  // Variable to track last EC update time
const long ecUpdateInterval = 1000;  // Interval to update EC value (1 second)

// Reed switch
int reedSwitchPin = 23;  // Pin that reed switch is attached to
int statusLEDPin = 13; //Built in LED on Teensy
uint8_t reedBaseline; // 0 = LOW, 1 = HIGH
uint32_t reedChangeTime = 0;
bool awaitingConfirm = false;

// Bar 30 pressure and temp sensor
#define pressureSCL 24
#define pressureSDA 25
MS5837 sensor;

//Pressure and temp sensor
float nowTemperature = 0.0;
float nowDepth = 0.0;
float nowPressure = 0.0;

// Menu
int menu = 0;  // An index value to establish which program menu is currently selected
int pos = 0;                 // An index value to establish which navigation position (within a menu) is currently selected
int scrollOffset = 0;        // Variable to store the current scroll position
const int numItems = 16;     // Total number of menu items
const int itemsPerPage = 6;  // Number of items that can fit on one screen

// SD Card
#define SD_CONFIG SdioConfig(FIFO_SDIO)
File sampleTimesSd;                                    // Name of sample parameter file saved on SD card
File settingsParamSd;                                  // Name of run parameter file saved on SD card
File dataLog;                                          // Name of data logging file saved on SD card
File myFile;                                           //declare variable
#define SAMPLE_TIMES_Y 14                              // Number of rows in sampleTimesSd.txt file
#define SAMPLE_TIMES_X 5                               // Number of columns in sampleTimesSd.txt file
#define SETTINGS_PARAM 7                               // Number of items in settingsParamSd.txt file
int sampleTimesArray[SAMPLE_TIMES_Y][SAMPLE_TIMES_X];  //an array of integers for the timing of each sample valve in form of hr,min,sec,day,mon,yr,sampling time in sec, pump after sample (1/0)

// Define global variables
const int maxRows = 15;                 //Number rows in setting menu
const int maxSamples = 14;              // Maximum number of rows
String sampleTimes[maxSamples];         // Array to hold the formatted sample times
time_t parsedSampleTimes[maxSamples];   // Array to store parsed times (as time_t)
time_t nextWakeUpTime;                  // Declare a global variable for next wakeuptime
const int chipSelect = BUILTIN_SDCARD;  // SD pin
#define COMPONENT_TITLE 0
#define COMPONENT_ROW_NUMBER 1
#define COMPONENT_MONTH 2
#define COMPONENT_DAY 3
#define COMPONENT_YEAR 4
#define COMPONENT_HOUR 5
#define COMPONENT_MINUTE 6
int selectedComponent = COMPONENT_TITLE;  // Start by selecting the month
bool editing = false;                     // Flag to indicate if we are in edit mode for a component

// RTC
time_t nowSecTime;     // Create a time variable called nowSecTime, representing the time now in seconds since 1970
time_t updateSecTime;  // Create a time variable called updateSecTime, representing the time to be set to the RTC in seconds since 1970
time_t aSecTime;       // Create a time variable called aSecTime, representing the time to fire pump A in seconds since 1970
time_t bSecTime;       // Create a time variable called bSecTime, representing the time to fire pump B in seconds since 1970
time_t currentTime;    //Variable for holding current time from RTC
int nowHr;
int nowMin;
int nowSec;
int nowDay;
int nowMon;
int nowYr;  // create integers for the time now in hours, minutes, seconds, days, months, years
int aHr;
int aMin;
int aSec;
int aDay;
int aMon;
int aYr;  // create integers for the time to fire A in hours, minutes, seconds, days, months, years
long aEndTime;
int aAlarmFlag = 0;

//Voltage Read
float batteryVoltage;
const int voltageReadPin = A15;                   // Pin that reads the voltage, digital pin 39
const int numSamples = 10;                        // Number of samples to average
float voltageSamples[numSamples];                 // Array to store samples
int sampleIndex = 0;                              // Index to track the sample
float sum = 0;                                    // Sum of all samples for averaging
float averagedVoltage = 0;                        // Stabilized voltage to display
const float resistorA = 680;                      //Larger resistor in voltage divider in k ohms
const float resistorB = 100;                      //Smaller resistor in voltage divider in k ohms
const float vRef = 3.3;                           //Reference voltage from teensy 4.1 is 3.3V
const unsigned long voltageUpdateInterval = 100;  // Update every 100 ms
unsigned long lastVoltageUpdateTime = 0;          // Variable to store last update time

//<-------------------------------------------------------------------------------------------------------------------------------------------------------------->
//<----------------------------------------------------------------------SETUP----------------------------------------------------------------------------------->
//<-------------------------------------------------------------------------------------------------------------------------------------------------------------->
void setup() {

  //RTC setup
  setSyncProvider(getTeensy3Time);  // Get the time from the teensy NOW!

  //Communication setup
  Serial.begin(9600);  // Initiate the serial connection, used for debugging
  delay(500);          // Pause for a half second
  Serial.println("Starting setup now.");

  //Voltage read setup
  Serial.println("Voltage setup now.");
  pinMode(voltageReadPin, INPUT);  // Set the voltage read pin as an input, where we will be reading the voltage of the battery pack
  analogReadResolution(12);        // Set ADC resolution to 12-bit (0-4095)

  //Conudctivity read setup
  myserial.begin(9600);         // Set baud rate for the software serial port
  sensorstring.reserve(30);     // Reserve space for the sensor string
  pinMode(ecOffPin, OUTPUT);    //Set mode
  digitalWrite(ecOffPin, LOW);  // Ensure EC sensor starts off

  //Stepper motor
  pinMode(stepperOn, OUTPUT);
  pinMode(photoHomePin, INPUT);
  pinMode(stepperDir, OUTPUT);
  pinMode(stepperStep, OUTPUT);
  pinMode(stepperReset, OUTPUT);
  pinMode(stepperMode0, OUTPUT);
  pinMode(stepperMode1, OUTPUT);
  pinMode(stepperMode2, OUTPUT);
  digitalWrite(stepperReset, HIGH);
  digitalWrite(stepperMode0, LOW);
  digitalWrite(stepperMode1, LOW);
  digitalWrite(stepperMode2, LOW);
  pinMode(stepperEncAPin, INPUT_PULLUP);
  pinMode(stepperEncBPin, INPUT_PULLUP);

  // Read initial state
  lastState = (digitalRead(stepperEncAPin) << 1) | digitalRead(stepperEncBPin);
  // Attach interrupts to both encoder pins
  attachInterrupt(digitalPinToInterrupt(stepperEncAPin), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(stepperEncBPin), encoderISR, CHANGE);

  //Pumps
  pinMode(pumpsOn, OUTPUT);           //Set the pumpsOn pin to output to send LOW signal to driver enable pins for running
  pinMode(pumpARunPin, OUTPUT);       // Set the pumpARunPin to an output because this is the pin that we will use to power pump A
  pinMode(pumpADirPin, OUTPUT);       // Set the pumpADirPin to an output because this is the pin that we will use to change direction of pump A
  pinMode(pumpAEncPowerPin, OUTPUT);  //Set as output to power encoder on pumpA
  pumpAEnc.write(0);
  pinMode(pumpBRunPin, OUTPUT);       // Set the pumpBRunPin to an output because this is the pin that we will use to power pump A
  pinMode(pumpBDirPin, OUTPUT);       // Set the pumpBDirPin to an output because this is the pin that we will use to change direction of pump A
  pinMode(pumpBEncPowerPin, OUTPUT);  //Set as output to power encoder on pumpB
  pumpBEnc.write(0);
  digitalWrite(pumpsOn, HIGH);

  //Three-way Valve
  pinMode(threeWayPin, OUTPUT);    //Set three-way pin to be output for opening and closing valve.
  digitalWrite(threeWayPin, LOW);  // Set the threeway pin to low

  // Low power timer setup
  Serial.println("LPT setup now.");
  pinMode(timerDonePin, OUTPUT);  // Output pin for timer done signal
  pinMode(powerOnPin, OUTPUT);    // Output pin for power on signal

  //Reed interrupt
  Serial.println("Reed setup now.");
  pinMode(reedSwitchPin, INPUT_PULLUP);  //Reef switch input
  pinMode(statusLEDPin, OUTPUT);
  digitalWrite(statusLEDPin, LOW);  // LED off initially  

  // Read the current time
  time_t currentTime = now();
  Serial.println("Current Time: " + String(currentTime));

  // Read stored wake-up time from EEPROM
  EEPROM.get(eepromWakeUpAddress, storedWakeUpTime);
  Serial.println("Stored Wake-Up Time: " + String(storedWakeUpTime));

  //Check if reed baseline state has changed
  initReedBaseline();

  ////////////////////////////////////////////////////////////////////////
  ////// All above basic setup, below program options/////////////////////
  ////////////////////////////////////////////////////////////////////////
  
  uint8_t current = digitalRead(reedSwitchPin);
  bool enterGUI = false;  // flag to trigger GUI after the loop
  /* ---------------- REED GATE ---------------- */
  if (!awaitingConfirm && current != reedBaseline) {
      Serial.println("Reed change detected, awaiting confirmation");
      awaitingConfirm = true;
      reedChangeTime = millis();
      digitalWrite(statusLEDPin, HIGH);  // LED on
      // Blocking 5-second loop
      while (millis() - reedChangeTime <= 5000) {
          current = digitalRead(reedSwitchPin);
          // User reversed reed → trigger GUI
          if (current == reedBaseline) {
              enterGUI = true;
              break;  // exit the blocking loop immediately
          }
      }
      // After loop exits: handle results
      if (enterGUI) {
          menu = 0;
          activeMenu = 0;
          digitalWrite(powerOnPin, HIGH);
          delay(1000);
          digitalWrite(timerDonePin, LOW);
          delay(500);
          digitalWrite(timerDonePin, HIGH);
          digitalWrite(statusLEDPin, LOW);
          Serial.println("Reed switch activated, entering GUI mode...");
          // Initialize SD card
          if (!SD.begin(chipSelect)) {
              Serial.println("SD Card initialization failed!");
              while (1);
          }
          Serial.println("SD Card initialized.");
          Serial.println("Reading sample times and settings next");
          readSampleTimes();
          readSampleSettings();
          delay(3000);
          // Infrared Setup
          Serial.println("IR setup now.");
          pinMode(irPowerPin, OUTPUT);
          digitalWrite(irPowerPin, HIGH);
          IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);
          // PressureTemp sensor setup
          Serial.println("Starting pressure sensor...");
          Wire2.begin();
          if (!sensor.init(Wire2)) {
              Serial.println("Init failed!");
              delay(5000);
          }
          sensor.setModel(MS5837::MS5837_30BA);
          sensor.setFluidDensity(1029);
          // OLED Setup
          setupOLED();
          // Read stored valve position
          EEPROM.get(eepromValvePosAddress, currentValvePositionIndex);
          Serial.println("Stored valve position: " + String(currentValvePositionIndex));
          if (currentValvePositionIndex < 0 || currentValvePositionIndex > 16) {
              currentValvePositionIndex = 1;
              EEPROM.put(eepromValvePosAddress, currentValvePositionIndex);
              delay(100);
              Serial.println("No valid stored valve position found. Setting to 1.");
              EEPROM.get(eepromValvePosAddress, currentValvePositionIndex);
          }
          Serial.println("Stored valve position is now: " + String(currentValvePositionIndex));
          return;  // stop setup here

          } else {

        reedBaseline = current; // Timeout expired → accept new baseline
        EEPROM.put(eepromReedBaselineAddress, reedBaseline ? 1 : 0); // save new baseline
        Serial.println("Reed not confirmed → new baseline set, continuing setup");
        digitalWrite(statusLEDPin, LOW);
    }
    awaitingConfirm = false; // reset for next check
}

  if (storedWakeUpTime == NO_MORE_SAMPLES) {
    Serial.println("Should be here...");
    //PressureTemp sensor setup
    Serial.println("Starting pressure sensor...");
    // Initialize SD card
    if (!SD.begin(chipSelect)) {  //
      Serial.println("SD Card initialization failed!");
      while (1)
        ;  // Loop forever if initialization fails
    }
    Wire2.begin();
    if (!sensor.init(Wire2)) {  //make "while" if don't want code to keep going
      Serial.println("Init failed!");
      }
    // Set sensor model and fluid density
    sensor.setModel(MS5837::MS5837_30BA);  // Set the sensor model (Blue Robotics Bar30)
    sensor.setFluidDensity(1029);          // Set fluid density, for freshwater 997, for seawater 1029 (kg/m^3)
    logData(); //Log all sensor data and time
    Serial.println("No more samples, returning to sleep mode...");
    sleepyTime();
    return;
  }
  if (storedWakeUpTime > 0 && difftime(storedWakeUpTime, currentTime) > 600) {
    // If a valid time is stored and it’s more than 10 minute in the future, go to sleep
    //PressureTemp sensor setup
    Serial.println("Starting pressure sensor...");
    // Initialize SD card
    if (!SD.begin(chipSelect)) {  //
      Serial.println("SD Card initialization failed!");
      while (1)
        ;  // Loop forever if initialization fails
    }
    Wire2.begin();
    if (!sensor.init(Wire2)) {  //make "while" if don't want code to keep going
      Serial.println("Init failed!");
      }
    // Set sensor model and fluid density
    sensor.setModel(MS5837::MS5837_30BA);  // Set the sensor model (Blue Robotics Bar30)
    sensor.setFluidDensity(1029);          // Set fluid density, for freshwater 997, for seawater 1029 (kg/m^3)
    logData(); //Log all sensor data and time
    Serial.println("Stored time is more than 10 minutes in the future. Entering sleep mode...");
    sleepyTime();
    return;
  }
  if (storedWakeUpTime == 0 || difftime(currentTime, storedWakeUpTime) >= 0) {
    // If no time stored or stored time is in the past, proceed to Menu 1
    Serial.println("Stored time is in the past or not set. Setting stored time.");
    //PressureTemp sensor setup
    Serial.println("Starting pressure sensor...");
    Wire2.begin();
    if (!sensor.init(Wire2)) {  //make "while" if don't want code to keep going
      Serial.println("Init failed!");
      }
    // Set sensor model and fluid density
    sensor.setModel(MS5837::MS5837_30BA);  // Set the sensor model (Blue Robotics Bar30)
    sensor.setFluidDensity(1029);          // Set fluid density, for freshwater 997, for seawater 1029 (kg/m^3)   
    menu = 1;
    Serial.println("Entering case 1");
    digitalWrite(powerOnPin, HIGH);
    delay(500);
    // Initialize SD card
    if (!SD.begin(chipSelect)) {  //
      Serial.println("Update SD Card initialization failed!");
      while (1)
        ;  // Loop forever if initialization fails
    }
    Serial.println("Update SD Card initialized.");
    return;
  }

  if (abs(difftime(currentTime, storedWakeUpTime)) <= 600) {
    // If the stored time is within 10 minutes, proceed to Case 2
    Serial.println("Stored time is within 10 minutes. Entering sampling mode.");
    menu = 2;
    digitalWrite(powerOnPin, HIGH);
    delay(500);
    return;
  } else {
    Serial.println("No condition met, punch yourself please...");
    // Initialize SD card
    if (!SD.begin(chipSelect)) {  //
      Serial.println("SD Card initialization failed!");
      while (1)
        ;  // Loop forever if initialization fails
    }
    Wire2.begin();
    if (!sensor.init(Wire2)) {  //make "while" if don't want code to keep going
      Serial.println("Init failed!");
      }
    // Set sensor model and fluid density
    sensor.setModel(MS5837::MS5837_30BA);  // Set the sensor model (Blue Robotics Bar30)
    sensor.setFluidDensity(1029);          // Set fluid density, for freshwater 997, for seawater 1029 (kg/m^3)
    logData(); //Log all sensor data and time
    sleepyTime();
  }
}

//<-------------------------------------------------------------------------------------------------------------------------------------------------------------->
//<------------------------------------------------------------------MAIN LOOP----------------------------------------------------------------------------------->
//<-------------------------------------------------------------------------------------------------------------------------------------------------------------->

void loop() {
  switch (menu) {  //What menu is selected?
    case 0:
      switch (activeMenu) {  //What activeMenu is selected?
        case 0:
          getNow();
          statusMenuNavigation();
          statusMenuDisplay();
          break;  // Loop the Status Menu navigation and display

        case 1:
          sampleParamMenuNavigation();
          sampleParamMenuDisplay();
          break;  //Loop the Settings Menu navigation and display

        case 2:
          sampleTimesMenuNavigation();
          sampleTimesMenuDisplay();
          break;  //Loop the Settings Menu navigation and display

        case 3:
          valveMenuNavigation();
          valveMenuDisplay();
          break;  //Loop the Settings Menu navigation and display

        case 4:
          primeMenuNavigation();
          primeMenuDisplay();
          break;  //Loop the priming menu

        case 5:
          calibrationMenuNavigation();
          calibrationMenuDisplay();
          break;  // Loop the Calibration Menu navigation and display

        case 6:
          timeSetMenuNavigation();
          timeSetMenuDisplay();
          break;  //Loop the time set menu

        case 7:
          zeroValveMenuNavigation();
          zeroValveMenuDisplay();
          break;  // Loop the Zero Valve Menu navigation and display
        
        case 8:
          cleanMenuNavigation();
          cleanMenuDisplay();
          break;  // Loop the Clean Menu navigation and display

        case 9:
          testPumpVolumesMenuNavigation();
          testPumpVolumesMenuDisplay();
          break;  // Loop the Test Pump volumes Menu navigation and display

        // case 8:
        //   calibrationECMenuNavigation();
        //   calibrationECMenuDisplay();
        //   break;  // Loop the Calibration EC Menu navigation and display
      }
      break;  // Prevent fall-through to case 1

    case 1:
      {
        Serial.println(menu);
        // If the stored wake-up time is in the past or invalid
        // Read the next wake-up time from the SD card
        time_t nextWakeUpTime = findNextWakeUpTimeFromSD();
        EEPROM.put(eepromWakeUpAddress, nextWakeUpTime);  // Save the next wake-up time to EEPROM
        Serial.println("Next Wake-Up Time from SD: " + String(nextWakeUpTime));
        // Check if nextWakeUpTime is equal to NO_MORE_SAMPLES (no more samples)
        if (nextWakeUpTime == NO_MORE_SAMPLES) {
          Serial.println("No more samples available.");
          // Handle the case when there are no more samples
          logData(); //Log all sensor data and time
          sleepyTime();  // Put the system back to sleep
          return;        // Exit case 1
        }
        if (abs(difftime(currentTime, nextWakeUpTime)) <= 600) {  // Check if the new wake-up time is within 10 minutes
          menu = 2;
          return;
        } else {
          //PressureTemp sensor setup
          Serial.println("Starting pressure sensor...");
          logData(); //Log all sensor data and time
          Serial.println("Calling sleepyTime()...");
          sleepyTime();
        }
        break;  //Loop the Settings Menu navigation and display
      }
    case 2:
      {
        Serial.println(menu);
        // If sample time is within ten minutes stay on and
        Serial.println("Sampling mode starting.");
        // Initialize SD card
        if (!SD.begin(chipSelect)) {
          Serial.println("Sampling SD Card initialization failed!");
          while (1)
            ;  // Loop forever if initialization fails
        }
        Serial.println("Sampling SD Card initialized.");
        //Initialize pressure sensor
        Wire2.begin();
        if (!sensor.init(Wire2)) {  //make "while" if don't want code to keep going
          Serial.println("Init failed!");
          delay(5000);
        }
        // Set sensor model and fluid density
        sensor.setModel(MS5837::MS5837_30BA);  // Set the sensor model (Blue Robotics Bar30)
        sensor.setFluidDensity(1029);          // Set fluid density, for freshwater 997, for seawater 1029 (kg/m^3)
        // Read stored wake-up time from EEPROM
        EEPROM.get(eepromWakeUpAddress, storedWakeUpTime);
        Serial.println("Stored Wake-Up Time: " + String(storedWakeUpTime));
        EEPROM.get(eepromValvePosAddress, currentValvePositionIndex);
        Serial.println("Stored Valve Position: " + String(currentValvePositionIndex));
        readSampleSettings();  //Read sample settings
        delay(3000);
        enterSamplingMode();
        break;  //Loop the Settings Menu navigation and display
      }
  }
}
///////////////////////////////////////////////////////////////////
/////////////////////////Display Menus/////////////////////////////
///////////////////////////////////////////////////////////////////
void statusMenuDisplay() {  // Text to display in the status menu
  // display.clearDisplay();
  unsigned long currentMillis = millis();
  // First, check for IR commands
  if (IrReceiver.decode()) {  // Assuming irReceiver is your IR receiver object
    // Handle the IR command (you can add your IR command handling here)
    IrReceiver.resume();  // This is required to get the next signal
  }
  if (currentMillis - lastUpdate >= interval) {
    lastUpdate = currentMillis;  // Update the time for the next update cycle
    display.clearDisplay();
    getVoltage();
    getPressureTemp();
    getLight();
    // Set text color to white
    display.setTextColor(SSD1306_WHITE);
    // Set text size
    display.setTextSize(1);
    // Set the text cursor to top-left corner (0, 0)
    display.setCursor(0, 0);
    // Print centered status menu with eye icon
    display.setCursor(22, 0);  // Centered <STATUS MENU>
    display.print("<STATUS MENU>");
    display.setCursor(106, 0);  // Eye just to the right
    if (digitalRead(reedSwitchPin) == LOW) {
      display.print("o");  // Eye open
    } else {
      display.print("-");  // Eye closed
    }
    //Display current time (hour, minute, second)
    display.setCursor(0, 9);  // Position for T:
    display.print("T:");
    display.setCursor(18, 9);  // Position for hour
    display.print(nowHr);
    display.setCursor(30, 9);  // Position for colon
    display.print(":");
    display.setCursor(36, 9);  // Position for minute
    display.print(nowMin);
    display.setCursor(48, 9);  // Position for colon
    display.print(":");
    display.setCursor(54, 9);  // Position for second
    display.print(nowSec);
    // Display current date (month, day, year)
    display.setCursor(0, 18);  // Position for D:
    display.print("D:");
    display.setCursor(18, 18);  // Position for month
    display.print(nowMon);
    display.setCursor(30, 18);  // Position for slash
    display.print("/");
    display.setCursor(40, 18);  // Position for day
    display.print(nowDay);
    display.setCursor(54, 18);  // Position for slash
    display.print("/");
    display.setCursor(64, 18);  // Position for year
    display.print(nowYr);
    // Display battery voltage, only update if it has changed
    display.setCursor(0, 27);  // Position for "Batt (V):"
    display.println("Batt(V):");
    display.setCursor(54, 27);  // Position for voltage value
    display.print(averagedVoltage);
    // Display temperature
    display.setCursor(0, 36);  // Position for "Temp(C):"
    display.println("Temp(C):");
    display.setCursor(54, 36);  // Position for temp value
    display.print(nowTemperature);
    // Display pressure and depth
    display.setCursor(0, 45);  // Position for "Depth(m):"
    display.println("Depth(m):");
    display.setCursor(54, 45);  // Position for depth value
    display.print(nowDepth);
    // Display light
    display.setCursor(0, 54);  // Position for "Light:"
    display.println("Light:");
    display.setCursor(54, 54);  // Position for light value
    display.print(lightLevel);
    // Display the content on the screen
    display.display();
  }
}

void sampleParamMenuDisplay() {  // Text to display in the sample param menu
  // Clear the display buffer
  display.clearDisplay();
  // Set text size and color
  display.setTextSize(1);
  // Display title
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(" <SAMPLE PARAMS> ");
  // Display and highlight "Sample volume:" if selected
  if (pos == 1) {
    display.setTextColor(BLACK, WHITE);  // Highlight
  } else {
    display.setTextColor(WHITE, BLACK);  // Normal
  }
  display.setCursor(0, 8);
  display.println("Sample volume (mL):");
  if (pos == 2) {
    display.setTextColor(BLACK, WHITE);  // Highlight
  } else {
    display.setTextColor(WHITE, BLACK);  // Normal
  }
  display.setCursor(0, 18);
  display.println(pumpAVolume);
  // Display and highlight "Preserve volume:" if selected
  if (pos == 3) {
    display.setTextColor(BLACK, WHITE);  // Highlight
  } else {
    display.setTextColor(WHITE, BLACK);  // Normal
  }
  display.setCursor(0, 30);
  display.println("Preserve volume (mL):");
  if (pos == 4) {
    display.setTextColor(BLACK, WHITE);  // Highlight
  } else {
    display.setTextColor(WHITE, BLACK);  // Normal
  }
  display.setCursor(0, 40);
  display.println(pumpBVolume);
  if (pos == 5) {
    display.setTextColor(BLACK, WHITE);  // Highlight
  } else {
    display.setTextColor(WHITE, BLACK);  // Normal
  }
  display.setCursor(0, 52);
  display.println("Save settings");
  numberCorrect();  // Correct all displayed numbers
  // Display the content on the screen
  display.display();
}

void sampleTimesMenuDisplay() {  // Text to display in the sample times menu
  // display.display();  // Refresh the display with initial content
  // delay(500);        // Pause for 2 seconds
  // Clear the display buffer
  display.clearDisplay();
  // Set text color to white
  display.setTextColor(SSD1306_WHITE);
  // Set text size
  display.setTextSize(1);
  // Set the text cursor to top-left corner (0, 0)
  display.setCursor(0, 0);
  int displayCount = 0;  // Counter to keep track of displayed items
  // Print test message
  display.println("   <SAMPLE TIMES>   ");
  // Display sample times
  for (int i = scrollOffset; displayCount < itemsPerPage && i < maxSamples; i++, displayCount++) {
    display.setCursor(0, 16 + displayCount * 8);  // Calculate cursor position based on displayCount
    // Highlighting the row number
    if (selectedComponent == COMPONENT_ROW_NUMBER && i == pos) {
      display.setTextColor(BLACK, WHITE);  // Highlight row number
    } else {
      display.setTextColor(WHITE, BLACK);
    }
    // Display row number
    display.print(i + 1);  // Print row number
    display.print(". ");
    // Reset text color for the time components before displaying
    display.setTextColor(WHITE, BLACK);
    // Display sample time components with conditional highlighting
    // Month
    if (selectedComponent == COMPONENT_MONTH && i == pos) {
      display.setTextColor(BLACK, WHITE);
    } else {
      display.setTextColor(WHITE, BLACK);
    }
    display.print(sampleTimes[i].substring(0, 2));
    display.print("/");
    // Day
    if (selectedComponent == COMPONENT_DAY && i == pos) {
      display.setTextColor(BLACK, WHITE);
    } else {
      display.setTextColor(WHITE, BLACK);
    }
    display.print(sampleTimes[i].substring(3, 5));
    display.print("/");
    // Year
    if (selectedComponent == COMPONENT_YEAR && i == pos) {
      display.setTextColor(BLACK, WHITE);
    } else {
      display.setTextColor(WHITE, BLACK);
    }
    display.print(sampleTimes[i].substring(6, 8));
    display.print(" ");
    // Hour
    if (selectedComponent == COMPONENT_HOUR && i == pos) {
      display.setTextColor(BLACK, WHITE);
    } else {
      display.setTextColor(WHITE, BLACK);
    }
    display.print(sampleTimes[i].substring(9, 11));
    display.print(":");
    // Minute
    if (selectedComponent == COMPONENT_MINUTE && i == pos) {
      display.setTextColor(BLACK, WHITE);
    } else {
      display.setTextColor(WHITE, BLACK);
    }
    display.print(sampleTimes[i].substring(12, 14));
    // Reset text color to default after each row
    display.setTextColor(WHITE, BLACK);
  }
  // Display additional options below the sample times
  if (displayCount < itemsPerPage) {
    // Save Times option
    display.setCursor(0, 16 + displayCount * 8);
    if (selectedComponent == COMPONENT_ROW_NUMBER && pos == maxSamples) {
      display.setTextColor(BLACK, WHITE);  // Highlight "Save Times"
    } else {
      display.setTextColor(WHITE, BLACK);
    }
    display.println("SAVE TIMES");
    displayCount++;
    // Initiate option
    if (displayCount < itemsPerPage) {
      display.setCursor(0, 16 + displayCount * 8);
      if (selectedComponent == COMPONENT_ROW_NUMBER && pos == maxSamples + 1) {
        display.setTextColor(BLACK, WHITE);  // Highlight "Initiate"
      } else {
        display.setTextColor(WHITE, BLACK);
      }
      display.println("INITIATE");
    }
  }
  numberCorrect();  // Correct all displayed numbers
  // Display the content on the screen
  display.display();
}

void valveMenuDisplay() {  // Text to display in the settings menu
  unsigned long currentMillis = millis();  // Only update the display at set intervals
  if (currentMillis - lastUpdate >= interval) {
    lastUpdate = currentMillis;// Clear the display buffer
    
    display.clearDisplay();
    // Set text size and color
    display.setTextSize(1);

    // Display title
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(" <Valve Pos Menu> ");

    // Display and highlight "Current Pos:" if selected
    if (pos == 1) {
      display.setTextColor(BLACK, WHITE);  // Highlight
    } else {
      display.setTextColor(WHITE, BLACK);  // Normal
    }
    display.setCursor(0, 8);
    display.println("Current Pos:");
    display.setCursor(100, 8);
    display.println(currentValvePositionIndex);

    // Display and highlight "New Pos:" if selected
    if (pos == 2) {
      display.setTextColor(BLACK, WHITE);  // Highlight
    } else {
      display.setTextColor(WHITE, BLACK);  // Normal
    }
    display.setCursor(0, 16);
    display.println("New Pos:");
    if (pos == 21) {
      display.setTextColor(BLACK, WHITE);  // Highlight
    } else {
      display.setTextColor(WHITE, BLACK);  // Normal
    }
    display.setCursor(100, 16);
    display.println(selectedValvePositionIndex);

    // Display and highlight "Set Position" if selected
    if (pos == 3) {
      display.setTextColor(BLACK, WHITE);  // Highlight
    } else {
      display.setTextColor(WHITE, BLACK);  // Normal
    }
    display.setCursor(0, 24);
    display.println("Set Position");

    // Display and highlight "Zero valve" if selected
    if (pos == 4) {
      display.setTextColor(BLACK, WHITE);  // Highlight
    } else {
      display.setTextColor(WHITE, BLACK);  // Normal
    }
    display.setCursor(0, 32);
    display.println("Zero valve");

    // Display and highlight "Run Motor A" if selected
    if (pos == 5) {
      display.setTextColor(BLACK, WHITE);  // Highlight
    } else {
      display.setTextColor(WHITE, BLACK);  // Normal
    }
    display.setCursor(0, 40);
    display.println("Run Motor A");

    // Display and highlight "Run Motor B" if selected
    if (pos == 6) {
      display.setTextColor(BLACK, WHITE);  // Highlight
    } else {
      display.setTextColor(WHITE, BLACK);  // Normal
    }
    display.setCursor(0, 48);
    display.println("Run Motor B");

    // Highlight "Open valve" only if selected
    if (pos == 7) {
      display.setTextColor(BLACK, WHITE);  // Highlight
    } else {
      display.setTextColor(WHITE, BLACK);  // Normal
    }
    display.setCursor(0, 56);
    display.println("Open valve");
    // Always draw eye symbol (non-highlighted)
    display.setCursor(70, 56);
    display.setTextColor(WHITE, BLACK);
    display.print(digitalRead(threeWayPin) == HIGH ? "o" : "-");
    // Display the content on the screen
    display.display();
  }
}

void zeroValveMenuDisplay() {  // Text to display in the settings menu
  // Clear the display buffer
  display.clearDisplay();
  // Set text size and color
  display.setTextSize(1);
  // Display title
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(" <Valve Offset Menu> ");
  // Display and highlight "Only do during first setup." if selected
  if (pos == 1) {
    display.setTextColor(BLACK, WHITE);  // Highlight
  } else {
    display.setTextColor(WHITE, BLACK);  // Normal
  }
  display.setCursor(0, 10);
  // Display and highlight "Zero valve" if selected
  display.println("Zero valve");
  // Display and highlight "Manual Adjust" if selected
  if (pos == 2) {
    display.setTextColor(BLACK, WHITE);  // Highlight
  } else {
    display.setTextColor(WHITE, BLACK);  // Normal
  }
  display.setCursor(0, 22);
  display.println("Manual adjust");
  // Display the content on the screen
  display.display();
}

void cleanMenuDisplay() {  // Text to display in the settings menu
  // Clear the display buffer
  display.clearDisplay();
  // Set text size and color
  display.setTextSize(1);
  // Display title
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(" <Clean System Menu> ");
  // Display and highlight "Only do during first setup." if selected
  if (pos == 1) {
    display.setTextColor(BLACK, WHITE);  // Highlight
  } else {
    display.setTextColor(WHITE, BLACK);  // Normal
  }
  display.setCursor(0, 12);
  // Display and highlight "Clean system" if selected
  display.println("Clean System");
  if (pos == 2) {
    display.setTextColor(BLACK, WHITE);  // Highlight
  } else {
    display.setTextColor(WHITE, BLACK);  // Normal
  }
  display.setCursor(0, 24);
  // Display and highlight "Clean system" if selected
  display.println(cleanVolume);
  numberCorrect();  // Correct all displayed numbers
  // Display the content on the screen
  display.display();
}

void calibrationMenuDisplay() {
  display.clearDisplay();  // Clear the display buffer
  display.setTextSize(1);  // Set text size
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(" <Calibration Menu> ");

  // Highlight "Run Calibration A" if selected
  if (pos == 1) {
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE, BLACK);
  }
  display.setCursor(0, 8);
  display.println("Run Calibration A");

  // Highlight "Volume A:" if selected
  if (pos == 2) {
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE, BLACK);
  }
  display.setCursor(0, 20);
  display.println("Volume A:");

  // Highlight calibration value separately if selected
  if (pos == 21) {
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE, BLACK);
  }
  display.setCursor(70, 20);
  display.println(calibrationAVolume);

  // Highlight "Run Calibration B" if selected
  if (pos == 3) {
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE, BLACK);
  }
  display.setCursor(0, 32);
  display.println("Run Calibration B");

  // Highlight "Volume B:" if selected
  if (pos == 4) {
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE, BLACK);
  }
  display.setCursor(0, 44);
  display.println("Volume B:");

  // Highlight calibration value separately if selected
  if (pos == 41) {
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE, BLACK);
  }
  display.setCursor(70, 44);
  display.println(calibrationBVolume);

  // Highlight "Enter Settings" if selected
  if (pos == 5) {
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE, BLACK);
  }
  display.setCursor(0, 56);
  display.println("Save Settings");
  numberCorrect();    // Correct all displayed numbers
  display.display();  // Update the display
}

void primeMenuDisplay() {
  display.clearDisplay();  // Clear the display buffer
  display.setTextSize(1);  // Set text size
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(" <Prime Menu> ");
  // Highlight "Run Water" if selected
  if (pos == 1) {
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE, BLACK);
  }
  display.setCursor(0, 10);
  display.println("Run Blue Pump (A):");
  // Highlight "CW" if selected
  if (pos == 2) {
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE, BLACK);
  }
  display.setCursor(0, 20);
  display.println("CW");
  // Highlight "CCW" if selected
  if (pos == 21) {
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE, BLACK);
  }
  display.setCursor(24, 20);
  display.println("CCW");
  // Highlight "Run Pump B" if selected
  if (pos == 3) {
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE, BLACK);
  }
  display.setCursor(0, 30);
  display.println("Run Orange Pump (B)");
  // Highlight "CW" if selected
  if (pos == 4) {
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE, BLACK);
  }
  display.setCursor(0, 40);
  display.println("CW");
  // Highlight "CCW" if selected
  if (pos == 41) {
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE, BLACK);
  }
  display.setCursor(20, 40);
  display.println("CCW");
  // Highlight "Run System Prime" if selected
  if (pos == 5) {
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE, BLACK);
  }
  display.setCursor(0, 50);
  display.println("Prime Full System");
  display.display();  // Update the display
}

void timeSetMenuDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(" <Time Set> ");

  // TIME LINE
  display.setCursor(0, 10);
  if (pos == 1) display.setTextColor(BLACK, WHITE); else display.setTextColor(WHITE, BLACK);
  display.print("T:");
  if (pos == 11) display.setTextColor(BLACK, WHITE); else display.setTextColor(WHITE, BLACK);
  display.printf("%02d", nowHr);
  display.setTextColor(WHITE, BLACK);
  display.print(":");
  if (pos == 12) display.setTextColor(BLACK, WHITE); else display.setTextColor(WHITE, BLACK);
  display.printf("%02d", nowMin);
  display.setTextColor(WHITE, BLACK);
  display.print(":");
  if (pos == 13) display.setTextColor(BLACK, WHITE); else display.setTextColor(WHITE, BLACK);
  display.printf("%02d", nowSec);

  // DATE LINE (Month/Day/Year)
  display.setCursor(0, 24);
  if (pos == 2) display.setTextColor(BLACK, WHITE); else display.setTextColor(WHITE, BLACK);
  display.print("D:");
    if (pos == 21) display.setTextColor(BLACK, WHITE); else display.setTextColor(WHITE, BLACK);
  display.printf("%02d", nowMon);
  display.setTextColor(WHITE, BLACK);
  display.print("/");
  if (pos == 22) display.setTextColor(BLACK, WHITE); else display.setTextColor(WHITE, BLACK);
  display.printf("%02d", nowDay);
  display.setTextColor(WHITE, BLACK);
  display.print("/");
  if (pos == 23) display.setTextColor(BLACK, WHITE); else display.setTextColor(WHITE, BLACK);
  display.printf("%02d", nowYr % 100);  // 2-digit year

  // SAVE OPTION
  display.setCursor(0, 38);
  if (pos == 3) display.setTextColor(BLACK, WHITE); else display.setTextColor(WHITE, BLACK);
  display.print("Save Set Time");
  display.setTextColor(WHITE, BLACK);
  numberCorrect();
  display.display();
}

void testPumpVolumesMenuDisplay() {  // Text to display in the settings menu
  // Clear the display buffer
  display.clearDisplay();
  // Set text size and color
  display.setTextSize(1);
  // Display title
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(" <Test Pump Volumes> ");
  if (pos == 1) {
    display.setTextColor(BLACK, WHITE);  // Highlight
  } else {
    display.setTextColor(WHITE, BLACK);  // Normal
  }
  display.setCursor(0, 12);
  // Display and highlight "Sample" if selected
  display.println("Run Sample Valve 3");
  if (pos == 2) {
    display.setTextColor(BLACK, WHITE);  // Highlight
  } else {
    display.setTextColor(WHITE, BLACK);  // Normal
  }
  display.setCursor(0, 24);
  // Display and highlight "Preserve" if selected
  display.println("Run Preserve Valve 3");
    if (pos == 3) {
    display.setTextColor(BLACK, WHITE);  // Highlight
  } else {
    display.setTextColor(WHITE, BLACK);  // Normal
  }
  display.setCursor(0, 36);
  // Display and highlight "HCl" if selected
  display.println("Run HCl");
  numberCorrect();  // Correct all displayed numbers
  // Display the content on the screen
  display.display();
}

void calibrationECMenuDisplay() {
  getSalinity();
  // Check if it's time to update the EC value (every second)
  unsigned long currentMillis = millis();
  if (currentMillis - lastECUpdateTime >= ecUpdateInterval) {
    lastECUpdateTime = currentMillis;  // Update the last update time
    // Update the EC value (or any other real-time sensor update code you have)
  }
  // Don't clear the display if calibration is in progress
  if (!isCalibrating) {
    display.clearDisplay();  // Clear only if not calibrating
  }
  // Always display the title and EC value
  display.setCursor(0, 0);
  display.print("<Salinity Calib>");
  display.setCursor(0, 10);
  display.print("EC Value: ");
  display.print(ecValue);  // Update real-time sensor value

  // Handle submenus based on calibration step
  switch (calibrationStep) {
    case 0:  // Start Calibration Menu
      if (pos == 2) {
        display.setTextColor(BLACK, WHITE);
      } else {
        display.setTextColor(WHITE, BLACK);
      }  // If position 0 is selected, highlight text in that position
      display.setCursor(0, 20);
      display.print("Start Calibration");
      break;

    case 1:  // Dry Calibration
      //if (pos==0){display.setTextColor(BLACK,WHITE);} else{display.setTextColor(WHITE,BLACK);}// If position 0 is selected, highlight text in that position
      display.setCursor(0, 20);
      display.print("Dry Calibration");
      //if (pos==0){display.setTextColor(BLACK,WHITE);} else{display.setTextColor(WHITE,BLACK);}// If position 0 is selected, highlight text in that position
      display.setCursor(0, 30);  // Move to next line for options
      display.print("Start");
      //if (pos==0){display.setTextColor(BLACK,WHITE);} else{display.setTextColor(WHITE,BLACK);}// If position 0 is selected, highlight text in that position
      display.setCursor(0, 40);  // Move to next line
      display.print("Cancel");
      break;

    case 2:  // Low Point Calibration
      display.setCursor(0, 20);
      display.print("Low Point Calib.");
      display.setCursor(0, 30);  // Move to next line for options
      display.print("Start");
      display.setCursor(0, 40);  // Move to next line
      display.print("Cancel");
      break;

    case 3:  // High Point Calibration
      display.setCursor(0, 20);
      display.print("High Point Calib.");
      display.setCursor(0, 30);  // Move to next line for options
      display.print("Start");
      display.setCursor(0, 40);  // Move to next line
      display.print("Cancel");
      break;
  }

  // Show "Wait..." message if calibration is in progress
  if (isCalibrating) {
    display.setCursor(0, 20);  // This will overwrite the previous option in the submenu
    display.print("Wait...");
  }

  // Display the content on the screen
  display.display();
}


///////////////////////////////////////////////////////////////////
////////////////////////Navigation Menus///////////////////////////
///////////////////////////////////////////////////////////////////

void statusMenuNavigation() {
  if (IrReceiver.decode()) {  // If there is an IR signal, return true and store signal in "results"
    //Serial.println(IrReceiver.decodedIRData.decodedRawData);  //Uncomment this line to see what codes each IR button is generating to adjust code IDs in IR definitions
    Serial.println("Pos: " + String(pos));
    Serial.println("Menu: " + String(activeMenu));
    switch (IrReceiver.decodedIRData.decodedRawData) {  // What was the signal received from the IR remote?
      case LEFTIR:
        activeMenu = 9;
        pos = 0;
        display.clearDisplay();  // Clear the display when switching menus
        Serial.println("Navigating to GUI Menu 8");
        break;

      case RIGHTIR:
        activeMenu = 1;
        pos = 0;
        display.clearDisplay();  // Clear the display when switching menus
        Serial.println("Navigating to GUI Menu 1");
        break;
    }
    IrReceiver.resume();  // Continue receiving looking for IR signals
  }
}

void sampleParamMenuNavigation() {
  if (IrReceiver.decode()) {  // If there is an IR signal
    uint32_t rawData = IrReceiver.decodedIRData.decodedRawData;
    // Handle the signal based on the position and raw data received
    switch (pos) {
      case 0:  // Position is 0 (Highlight "Sample Params")
        if (rawData == LEFTIR) {
          activeMenu = 0;
          pos = 0;
          display.clearDisplay();  // Clear the display when switching menus
          Serial.println("Navigating to GUI Menu 0");
        } else if (rawData == RIGHTIR) {
          activeMenu = 2;
          pos = -1;
          display.clearDisplay();  // Clear the display when switching menus
          Serial.println("Navigating to GUI Menu 2");
        } else if (rawData == DOWNIR) {
          pos = 1;  // Move to "Sample volume"
        }
        break;
      case 1:  // Position is 1 (Highlight "Sample volume")
        if (rawData == LEFTIR) {
          pos = 0;  // Move to "Sample param Menu"
        } else if (rawData == RIGHTIR) {
          pos = 2;  // Move to "sample volume variable"
        } else if (rawData == UPIR) {
          pos = 0;  // Move to "sample param Menu"
        } else if (rawData == DOWNIR) {
          pos = 2;  // Move to "New Pos"
        }
        break;
      case 2:  // Position is 2 (Highlight "sample volume variable")
        if (rawData == LEFTIR) {
          pumpAVolume = pumpAVolume - pumpAVolumeChangeMl;  // Adjust sample volume down
        } else if (rawData == RIGHTIR) {
          pumpAVolume = pumpAVolume + pumpAVolumeChangeMl;  // Adjust sample volume up
        } else if (rawData == UPIR) {
          pos = 1;  // Move to "sample volume"
        } else if (rawData == DOWNIR) {
          pos = 3;  // Move to "preserve volume variable"
        }
        break;
      case 3:  // Position is 3 (Highlight "Preserve volume")
        if (rawData == LEFTIR) {
          pos = 2;  // Move to "sample volume variable"
        } else if (rawData == RIGHTIR) {
          pos = 4;  // Move to "preserve volume variable"
        } else if (rawData == UPIR) {
          pos = 2;  // Move to "sample volume"
        } else if (rawData == DOWNIR) {
          pos = 4;  // Move to "save settings"
        }
        break;
      case 4:  // Position is 4 (Highlight "preserve volume variable")
        if (rawData == LEFTIR) {
          pumpBVolume = pumpBVolume - pumpBVolumeChangeMl;  // Adjust sample volume down
        } else if (rawData == RIGHTIR) {
          pumpBVolume = pumpBVolume + pumpBVolumeChangeMl;  // Adjust sample volume up
        } else if (rawData == UPIR) {
          pos = 3;  // Move to "sample volume"
        } else if (rawData == DOWNIR) {
          pos = 5;  // Move to "save settings"
        }
        break;
      case 5:  // Position is 5 (Highlight "Save settings")
        if (rawData == LEFTIR) {
          pos = 4;  // Move to "preserve volume variable"
        } else if (rawData == RIGHTIR) {
          pos = 5;  // Stay on "save settings"
        } else if (rawData == UPIR) {
          pos = 4;  // Move to "preserve volume variable"
        } else if (rawData == DOWNIR) {
          pos = 5;  // Stay on "save settings"
        } else if (rawData == ENTERIR) {
          saveSampleSettings();  // Save settings
        }
        break;
    }
    IrReceiver.resume();  // Continue receiving IR signals
  }
}

void sampleTimesMenuNavigation() {
  if (IrReceiver.decode()) {  // If there is an IR signal
    Serial.println("Pos: " + String(pos));
    Serial.println("Menu: " + String(activeMenu));
    Serial.println("Scroll Offset: " + String(scrollOffset));
    switch (IrReceiver.decodedIRData.decodedRawData) {
      case LEFTIR:
        if (pos == maxRows) {
          // On last row, do nothing on LEFT
        } else {
          if (selectedComponent == COMPONENT_TITLE) {
            activeMenu = 1;          // Return to status menu
            pos = 0;                 // Reset position to 0
            display.clearDisplay();  // Clear the display when switching menus
            Serial.println("Navigating to GUI Menu 1");
          } else if (selectedComponent > COMPONENT_ROW_NUMBER) {
            selectedComponent--;  // Move to the previous component
          }
        }
        break;
      case RIGHTIR:
        if (pos == maxRows) {
          // On last row, do nothing on RIGHT
        } else {
          if (selectedComponent == COMPONENT_TITLE) {
            activeMenu = 3;          // Return to status menu
            pos = 0;                 // Reset position to 0
            display.clearDisplay();  // Clear the display when switching menus
            Serial.println("Navigating to GUI Menu 3");
          } else if (selectedComponent < COMPONENT_MINUTE) {
            selectedComponent++;  // Move to the next component
          }
        }
        break;
      case UPIR:
        if (selectedComponent == COMPONENT_ROW_NUMBER) {
          pos--;  // Move up through the rows
          if (pos < 0) {  // When navigating up past row 0, go to title
            pos = -1;  // Prevent going negative
            selectedComponent = COMPONENT_TITLE;  // Switch to the title
          } else if (pos < scrollOffset && pos > -1) {
            scrollOffset--;  // Adjust scrollOffset if moving above visible range
            if (scrollOffset < 0) {
              scrollOffset = 0;
            }
          }
        } else if (selectedComponent == COMPONENT_TITLE) {
          pos = -1;  // Reset position to ensure consistency
        } else {
          adjustSelectedComponent(1);  // Increment selected component value
          updateSampleTime();          // Update sampleTimes array after editing
        }
        break;
      case DOWNIR:
        if (selectedComponent == COMPONENT_TITLE) {
          pos = 0;  // Move down through rows
          selectedComponent = COMPONENT_ROW_NUMBER;  // Move to row selection
        } else if (selectedComponent == COMPONENT_ROW_NUMBER) {
          if (pos < maxRows) {
            pos++;  // Move down through the rows
            if (pos >= scrollOffset + itemsPerPage) {
              scrollOffset++;  // Adjust scrollOffset if moving below visible range
            }
          }
          if (pos > maxRows) {
            pos = maxRows;  // Limit pos to last selectable item
          }
        } else {
          adjustSelectedComponent(-1);  // Decrement selected component value
          updateSampleTime();           // Update sampleTimes array after editing
        }
        break;
      case ENTERIR:
        if (selectedComponent == COMPONENT_ROW_NUMBER) {
          if (pos == maxRows - 1) {
            saveSampleTimes();  // "Save Times" selected
            readSampleTimes();  // Read and print sample times
          } else if (pos == maxRows) {
            returnMainLoop();  // "Initiate" selected
          }
        }
        break;
    }
    IrReceiver.resume();  // Continue receiving IR signals
  }
}


void valveMenuNavigation() {
  if (IrReceiver.decode()) {  // If there is an IR signal
    uint32_t rawData = IrReceiver.decodedIRData.decodedRawData;
    // Handle the signal based on the position and raw data received
    switch (pos) {
      case 0:  // Position is 0 (Highlight "Valve Position Menu")
        if (rawData == LEFTIR) {
          activeMenu = 2;
          pos = -1;
          display.clearDisplay();  // Clear the display when switching menus
          Serial.println("Navigating to GUI Menu 2");
        } else if (rawData == RIGHTIR) {
          activeMenu = 4;
          pos = 0;
          display.clearDisplay();  // Clear the display when switching menus
          Serial.println("Navigating to GUI Menu 4");
        } else if (rawData == DOWNIR) {
          pos = 1;  // Move to "Current Pos"
        }
        break;
      case 1:  // Position is 1 (Highlight "Current Pos")
        if (rawData == LEFTIR) {
          pos = 0;  // Move to "Valve Position Menu"
        } else if (rawData == RIGHTIR) {
          pos = 2;  // Move to "New Pos"
        } else if (rawData == UPIR) {
          pos = 0;  // Move to "Valve Position Menu"
        } else if (rawData == DOWNIR) {
          pos = 2;  // Move to "New Pos"
        }
        break;
      case 2:  // Position is 2 (Highlight "New Pos")
        if (rawData == LEFTIR) {
          pos = 1;  // Move to "Current Pos"
        } else if (rawData == RIGHTIR) {
          pos = 21;  // Move to "New Pos Entry"
        } else if (rawData == UPIR) {
          pos = 1;  // Move to "Current Pos"
        } else if (rawData == DOWNIR) {
          pos = 3;  // Move to "Set Position"
        }
        break;
      case 21:  // Position is 21 (Highlight "Set Position")
        if (rawData == LEFTIR) {
          pos = 2;  // Move to "New Pos"
        } else if (rawData == RIGHTIR) {
          pos = 3;  // Move to "Set Position"
        } else if (rawData == UPIR) {
          valvePosCorrect(1);  // Adjust valve position up
        } else if (rawData == DOWNIR) {
          valvePosCorrect(-1);  // Adjust valve position down
        }
        break;
      case 3:  // Position is 3 (Highlight "Set Position")
        if (rawData == LEFTIR) {
          pos = 21;  // Move to "New Pos Entry"
        } else if (rawData == RIGHTIR) {
          pos = 3;  // Stay on "Set Position"
        } else if (rawData == UPIR) {
          pos = 2;  // Move to "New Pos"
        } else if (rawData == DOWNIR) {
          pos = 4;  // Move to "Zero Valve"
        } else if (rawData == ENTERIR) {
          adjustValvePosition();  // Perform the valve position adjustment
        }
        break;
      case 4:  // Position is 4 (Highlight "Zero Valve")
        if (rawData == LEFTIR) {
          pos = 3;  // Move to "Set Position"
        } else if (rawData == RIGHTIR) {
          pos = 5;  // Stay on "Zero Valve"
        } else if (rawData == UPIR) {
          pos = 3;  // Move to "Set Position"
        } else if (rawData == DOWNIR) {
          pos = 5;  // Stay on "Zero Valve"
        } else if (rawData == ENTERIR) {
          resetZeroPosition();  // Rotate the valve to zero position using photogate
        }
        break;
      case 5:  // Position is 5 (Highlight "Run Motor A")
        if (rawData == LEFTIR) {
          pos = 4;  // Move to "Zero Valve"
        } else if (rawData == RIGHTIR) {
          pos = 6;  // Stay on "Run Motor"
        } else if (rawData == UPIR) {
          pos = 4;  // Move to "Zero Valve"
        } else if (rawData == DOWNIR) {
          pos = 6;  // Stay on "Run Motor"
        } else if (rawData == ENTERIR) {
          runPumpA();  // Run the motor
        }
        break;
      case 6:  // Position is 6 (Highlight "Run Motor B")
        if (rawData == LEFTIR) {
          pos = 5;  // Move to "Run Motor A"
        } else if (rawData == RIGHTIR) {
          pos = 7;  // Stay on "Open Valve"
        } else if (rawData == UPIR) {
          pos = 5;  // Move to "Run Motor A"
        } else if (rawData == DOWNIR) {
          pos = 7;  // Stay on "Open Valve"
        } else if (rawData == ENTERIR) {
          runPumpB();  // Run the motor
        }
        break;
      case 7:  // Position is 7 (Highlight "Open Valve")
        if (rawData == LEFTIR) {
          pos = 6;  // Move to "Run Motor B"
        } else if (rawData == RIGHTIR) {
          pos = 7;  // Stay on "Open Valve"
        } else if (rawData == UPIR) {
          pos = 6;  // Move to "Run Motor B"
        } else if (rawData == DOWNIR) {
          pos = 7;  // Stay on "Test sampling"
        } else if (rawData == ENTERIR) {
          testValve(); //Open 3 way valve
        }
        break;
    }
    IrReceiver.resume();  // Continue receiving IR signals
  }
}

void zeroValveMenuNavigation() {
  if (IrReceiver.decode()) {  // If there is an IR signal
    uint32_t rawData = IrReceiver.decodedIRData.decodedRawData;
    // Handle the signal based on the position and raw data received
    switch (pos) {
      case 0:  // Position is 0 (Highlight "Valve Offset Menu")
        if (rawData == LEFTIR) {
          activeMenu = 6;
          pos = 0;                 // Reset position to 0
          display.clearDisplay();  // Clear the display when switching menus
          Serial.println("Navigating to GUI Menu 6");
        } else if (rawData == RIGHTIR) {
          activeMenu = 8;
          pos = 0;
          display.clearDisplay();  // Clear the display when switching menus
          Serial.println("Navigating to GUI Menu 8");
        } else if (rawData == DOWNIR) {
          pos = 1;  // Move to "Zero valve"
        }
        break;
      case 1:  // Position is 1 (Highlight Zero valve)
        if (rawData == LEFTIR) {
          pos = 0;  // Move to "Offset Valve Menu"
        } else if (rawData == RIGHTIR) {
          pos = 2;  // Move to "Manual adjust"
        } else if (rawData == UPIR) {
          pos = 0;  // Move to "Zero Valve"
        } else if (rawData == DOWNIR) {
          pos = 2;  // Move to "Find pos. 1"
        } else if (rawData == ENTERIR) {
          resetZeroPosition();  // Rotate the valve to zero position using photogate
        }
        break;
      case 2:  // Position is 2 (Highlight "Manual adjust")
        if (rawData == RIGHTIR) {
          moveValveOnePlus();
        } else if (rawData == LEFTIR) {
          moveValveOneMinus();
        } else if (rawData == UPIR) {
          pos = 1;  // Move to warning
        } else if (rawData == DOWNIR) {
          pos = 2;  // Stay on "manual adjust"
        }
        break;
    }
    IrReceiver.resume();  // Continue receiving IR signals
  }
}

void cleanMenuNavigation() {
  if (IrReceiver.decode()) {  // If there is an IR signal
    uint32_t rawData = IrReceiver.decodedIRData.decodedRawData;
    // Handle the signal based on the position and raw data received
    switch (pos) {
      case 0:  // Position is 0 (Highlight "Clean System Menu")
        if (rawData == LEFTIR) {
          activeMenu = 7;
          pos = 0;                 // Reset position to 0
          display.clearDisplay();  // Clear the display when switching menus
          Serial.println("Navigating to GUI Menu 7");
        } else if (rawData == RIGHTIR) {
          activeMenu = 9;
          pos = 0;
          display.clearDisplay();  // Clear the display when switching menus
          Serial.println("Navigating to GUI Menu 9");
        } else if (rawData == DOWNIR) {
          pos = 1;  // Move to "Clean system"
        }
        break;
      case 1:  // Position is 1 (Highlight Clean system)
        if (rawData == LEFTIR) {
          pos = 0;  // Move to "Clean System Menu"
        } else if (rawData == RIGHTIR) {
          pos = 2;  // Move to "Clean system menu"
        } else if (rawData == UPIR) {
          pos = 0;  // Move to "Clean system menu"
        } else if (rawData == DOWNIR) {
          pos = 2;  // Move to "Clean system menu"
        } else if (rawData == ENTERIR) {
          runCleanSystem();  // Clean system by priming all tubing with liquid
        }
        break;
      case 2:  // Adjust Clean Volume
        if (rawData == UPIR) {
          pos = 1;
        } else if (rawData == DOWNIR) {
          pos = 2;
        } else if (rawData == RIGHTIR) {
          cleanVolume += cleanVolumeChangeMl;
        } else if (rawData == LEFTIR) {
          cleanVolume -= cleanVolumeChangeMl;
        }
        break;
    }
    IrReceiver.resume();  // Continue receiving IR signals
  }
}

void testPumpVolumesMenuNavigation() {
  if (IrReceiver.decode()) {  // If there is an IR signal
    uint32_t rawData = IrReceiver.decodedIRData.decodedRawData;
    // Handle the signal based on the position and raw data received
    switch (pos) {
      case 0:  // Position is 0 (Highlight "Clean System Menu")
        if (rawData == LEFTIR) {
          activeMenu = 8;
          pos = 0;                 // Reset position to 0
          display.clearDisplay();  // Clear the display when switching menus
          Serial.println("Navigating to GUI Menu 8");
        } else if (rawData == RIGHTIR) {
          activeMenu = 0;
          pos = 0;
          display.clearDisplay();  // Clear the display when switching menus
          Serial.println("Navigating to GUI Menu 0");
        } else if (rawData == DOWNIR) {
          pos = 1;  // Move to "Sample"
        }
        break;
      case 1:  // Position is 1 (Highlight Run Sample)
        if (rawData == LEFTIR) {
          pos = 0;  // Move to "Test Sample Volume Menu"
        } else if (rawData == RIGHTIR) {
          pos = 2;  // Move to "Test Sample Volume Menu"
        } else if (rawData == UPIR) {
          pos = 0;  // Move to "Test Sample Volume Menu"
        } else if (rawData == DOWNIR) {
          pos = 2;  // Move to "Test Sample Volume Menu"
        } else if (rawData == ENTERIR) {
          testSampling();// Run sample as test
        }
        break;
      case 2:  // Run Preservative test
        if (rawData == LEFTIR) {
          pos = 1;  // Move to "Test Sample Volume Menu"
        } else if (rawData == RIGHTIR) {
          pos = 3;  // Move to "Test Sample Volume Menu"
        } else if (rawData == UPIR) {
          pos = 1;  // Move to "Test Sample Volume Menu"
        } else if (rawData == DOWNIR) {
          pos = 3;  // Move to "Test Sample Volume Menu"
        } else if (rawData == ENTERIR) {
          testPreserve();// Run Preserve as test
        }
        break;
      case 3:  // Run HCL test
        if (rawData == LEFTIR) {
          pos = 2;  // Move to "Test Sample Volume Menu"
        } else if (rawData == RIGHTIR) {
          pos = 3;  // Move to "Test Sample Volume Menu"
        } else if (rawData == UPIR) {
          pos = 2;  // Move to "Test Sample Volume Menu"
        } else if (rawData == DOWNIR) {
          pos = 3;  // Move to "Test Sample Volume Menu"
        } else if (rawData == ENTERIR) {
          testHCL();  // Run HCL as test
        }
        break;
    }
    IrReceiver.resume();  // Continue receiving IR signals
  }
}

void calibrationMenuNavigation() {
  if (IrReceiver.decode()) {  // If there is an IR signal
    uint32_t rawData = IrReceiver.decodedIRData.decodedRawData;
    switch (pos) {
      case 0:  // Position 0
        if (rawData == LEFTIR) {
          activeMenu = 4;
          pos = 0;
          display.clearDisplay();
          Serial.println("Navigating to GUI Menu 4");
        } else if (rawData == RIGHTIR) {
          activeMenu = 6;
          pos = 0;
          display.clearDisplay();
          Serial.println("Navigating to GUI Menu 6");
        } else if (rawData == DOWNIR) {
          pos = 1;
        }
        break;
      case 1:  // Position 1 (Calibration A option)
        if (rawData == LEFTIR || rawData == RIGHTIR) {
          pos = 2;
        } else if (rawData == UPIR) {
          pos = 0;
        } else if (rawData == DOWNIR) {
          pos = 2;
        } else if (rawData == ENTERIR) {
          runACalibration();
        }
        break;
      case 2:  // Position 2 (Calibration A adjustment)
        if (rawData == LEFTIR) {
          pos = 1;
        } else if (rawData == RIGHTIR) {
          pos = 21;
        } else if (rawData == UPIR) {
          pos = 1;
        } else if (rawData == DOWNIR) {
          pos = 3;
        }
        break;
      case 21:  // Adjust Calibration A Volume
        if (rawData == LEFTIR) {
          pos = 2;
        } else if (rawData == RIGHTIR) {
          pos = 3;
        } else if (rawData == UPIR) {
          calibrationAVolume += calibrationVolumeChangeMl;
        } else if (rawData == DOWNIR) {
          calibrationAVolume -= calibrationVolumeChangeMl;
        }
        break;
      case 3:  // Position 3 (Calibration B option)
        if (rawData == LEFTIR || rawData == RIGHTIR) {
          pos = 2;
        } else if (rawData == UPIR) {
          pos = 2;
        } else if (rawData == DOWNIR) {
          pos = 4;
        } else if (rawData == ENTERIR) {
          runBCalibration();
        }
        break;
      case 4:  // Position 2 (Calibration adjustment)
        if (rawData == LEFTIR) {
          pos = 3;
        } else if (rawData == RIGHTIR) {
          pos = 41;
        } else if (rawData == UPIR) {
          pos = 3;
        } else if (rawData == DOWNIR) {
          pos = 5;
        }
        break;
      case 41:  // Adjust Calibration Volume
        if (rawData == LEFTIR) {
          pos = 4;
        } else if (rawData == RIGHTIR) {
          pos = 5;
        } else if (rawData == UPIR) {
          calibrationBVolume += calibrationVolumeChangeMl;
        } else if (rawData == DOWNIR) {
          calibrationBVolume -= calibrationVolumeChangeMl;
        }
        break;
      case 5:  // Save Calibration
        if (rawData == LEFTIR) {
          pos = 4;
        } else if (rawData == RIGHTIR || rawData == DOWNIR) {
          pos = 5;  // Stay
        } else if (rawData == UPIR) {
          pos = 4;
        } else if (rawData == ENTERIR) {
          saveSampleSettings();
        }
        break;
    }
    IrReceiver.resume();
  }
}

void primeMenuNavigation() {
  if (IrReceiver.decode()) {  // If there is an IR signal
    uint32_t rawData = IrReceiver.decodedIRData.decodedRawData;
    switch (pos) {
      case 0:  // Position 0
        if (rawData == LEFTIR) {
          activeMenu = 3;
          pos = 0;
          display.clearDisplay();
          Serial.println("Navigating to GUI Menu 3");
        } else if (rawData == RIGHTIR) {
          activeMenu = 5;
          pos = 0;
          display.clearDisplay();
          Serial.println("Navigating to GUI Menu 5");
        } else if (rawData == DOWNIR) {
          pos = 1;
        }
        break;
      case 1:  // Prime Pump A
        if (rawData == UPIR) {
          pos = 0;
        } else if (rawData == DOWNIR) {
          pos = 2;
        }
        break;
      case 2:  // Run Pump A CW
        if (rawData == LEFTIR) {
          pos = 1;
        } else if (rawData == RIGHTIR) {
          pos = 21;
        } else if (rawData == UPIR) {
          pos = 1;
        } else if (rawData == DOWNIR) {
          pos = 3;
        } else if (rawData == ENTERIR) {
          pumpADirection = 0;  //0 means CW
          runPumpA();
        }
        break;
      case 21:  // Run Pump A CCW
        if (rawData == LEFTIR) {
          pos = 2;
        } else if (rawData == RIGHTIR) {
          pos = 3;
        } else if (rawData == ENTERIR) {
          pumpADirection = 1;  //1 means CCW
          runPumpA();
        }
        break;
      case 3:  // Prime Pump B
        if (rawData == UPIR) {
          pos = 2;
        } else if (rawData == DOWNIR) {
          pos = 4;
        }
        break;
      case 4:  // Run Pump B CW
        if (rawData == LEFTIR) {
          pos = 3;
        } else if (rawData == RIGHTIR) {
          pos = 41;
        } else if (rawData == UPIR) {
          pos = 3;
        } else if (rawData == DOWNIR) {
          pos = 5;
        } else if (rawData == ENTERIR) {
          pumpBDirection = 0;  //0 means CW
          runPumpB();
        }
        break;
      case 41:  // Run Pump B CCW
        if (rawData == LEFTIR) {
          pos = 4;
        } else if (rawData == RIGHTIR) {
          pos = 5;
        } else if (rawData == ENTERIR) {
          pumpBDirection = 1;  //1 means CCW
          runPumpB();
        }
        break;
      case 5:  // Run System Prime Function
        if (rawData == LEFTIR) {
          pos = 41;
        } else if (rawData == RIGHTIR) {
          pos = 5;
        } else if (rawData == UPIR) {
          pos = 4;
        } else if (rawData == ENTERIR) {
          pumpBDirection = 0;  //0 means CW
          runSystemPrime();
        }
        break;
    }
    IrReceiver.resume();
  }
}

void timeSetMenuNavigation() {
  if (IrReceiver.decode()) {  // If there is an IR signal
    uint32_t rawData = IrReceiver.decodedIRData.decodedRawData;
    switch (pos) {
      case 0:  // Position 0
        if (rawData == LEFTIR) {
          activeMenu = 5;
          pos = 0;
          display.clearDisplay();
          Serial.println("Navigating to GUI Menu 6");
        } else if (rawData == RIGHTIR) {
          activeMenu = 7;
          pos = 0;
          display.clearDisplay();
          Serial.println("Navigating to GUI Menu 7");
        } else if (rawData == DOWNIR) {
          pos = 1;
        }
        break;
      case 1:  // Set time
        if (rawData == UPIR) {
          pos = 0;
        } else if (rawData == DOWNIR) {
          pos = 2;
        } else if (rawData == LEFTIR) {
          pos = 0;
        } else if (rawData == RIGHTIR) {
          pos = 11;
        }
        break;
      case 11:  // Set Hour
        if (rawData == UPIR) {
          nowHr++;
        } else if (rawData == DOWNIR) {
          nowHr--;
        } else if (rawData == LEFTIR) {
          pos = 1;
        } else if (rawData == RIGHTIR) {
          pos = 12;
        }
        break;
      case 12:  // Set Minute
        if (rawData == UPIR) {
          nowMin++;
        } else if (rawData == DOWNIR) {
          nowMin--;
        } else if (rawData == LEFTIR) {
          pos = 11;
        } else if (rawData == RIGHTIR) {
          pos = 13;
        }
        break;
      case 13:  // Set Sec
        if (rawData == UPIR) {
          nowSec++;
        } else if (rawData == DOWNIR) {
          nowSec--;
        } else if (rawData == LEFTIR) {
          pos = 12;
        } else if (rawData == RIGHTIR) {
          pos = 2;
        }
        break;
      case 2:  // Set Date
        if (rawData == UPIR) {
          pos = 1;
        } else if (rawData == DOWNIR) {
          pos = 3;
        } else if (rawData == LEFTIR) {
          pos = 13;
        } else if (rawData == RIGHTIR) {
          pos = 21;
        }
        break;
      case 21:  // Set Month
        if (rawData == UPIR) {
          nowMon++;
        } else if (rawData == DOWNIR) {
          nowMon--;
        } else if (rawData == LEFTIR) {
          pos = 2;
        } else if (rawData == RIGHTIR) {
          pos = 22;
        }
        break;
      case 22:  // Set Day
        if (rawData == UPIR) {
          nowDay++;
        } else if (rawData == DOWNIR) {
          nowDay--;
        } else if (rawData == LEFTIR) {
          pos = 21;
        } else if (rawData == RIGHTIR) {
          pos = 23;
        }
        break;
      case 23:  // Set Year
        if (rawData == UPIR) {
          nowYr++;
        } else if (rawData == DOWNIR) {
          nowYr--;
        } else if (rawData == LEFTIR) {
          pos = 22;
        } else if (rawData == RIGHTIR) {
          pos = 3;
        }
        break;
      case 3:  // Save Time Settings
        if (rawData == UPIR) {
          pos = 2;
        } else if (rawData == DOWNIR) {
          pos = 3;
        } else if (rawData == LEFTIR) {
          pos = 2;
        } else if (rawData == RIGHTIR) {
          pos = 3;
        } else if (rawData == ENTERIR) {
          calcUpdateSecTime();                     // this gives you time_t updateSecTime
          Teensy3Clock.set(updateSecTime);         // sets hardware clock
          setTime(updateSecTime);                  // sets software time (TimeLib)
          display.clearDisplay();
          display.setTextSize(2);
          display.setTextColor(WHITE, BLACK);
          display.setCursor(0, 0);
          display.println(" <SAVED>");
          display.display();
          delay(1000);  //Calculate the updated time in seconds, set the clock to that second time, display "<SAVED>"   ;
        }
        break;
    }
    IrReceiver.resume();  // Continue receiving looking for IR signals
  }
}

void calibrationECMenuNavigation() {
  if (IrReceiver.decode()) {  // If there is an IR signal
    uint32_t rawData = IrReceiver.decodedIRData.decodedRawData;
    switch (calibrationStep) {
      case 0:  // Start Calibration Menu
        if (rawData == LEFTIR) {
          activeMenu = 7;
          pos = 0;                 // Reset pos to 0 when navigating
          display.clearDisplay();  // Clear the display when switching menus
          Serial.println("Navigating to GUI Menu 7");
        } else if (rawData == RIGHTIR) {
          activeMenu = 0;
          pos = 0;                 // Reset pos to 0 when navigating
          display.clearDisplay();  // Clear the display when switching menus
          Serial.println("Navigating to GUI Menu 0");
        } else if (rawData == DOWNIR) {
          pos = 2;  // Move to Dry Calibration menu
        }
        break;

      case 1:  // Dry Calibration
        if (rawData == LEFTIR) {
          pos = 0;  // Move to first line
        } else if (rawData == UPIR) {
          pos = 0;  // Move to first line
        } else if (rawData == ENTERIR) {
          if (pos == 2) {
            Serial.println("Starting Dry Calibration");
            runDryCalibration();
            calibrationStep = 2;     // Move to Low Point Calibration
            pos = 0;                 // Reset pos to 0
            display.clearDisplay();  // Clear the display when transitioning
          } else if (pos == 3) {
            Serial.println("Cancelling Calibration");
            calibrationStep = 0;     // Go back to Start Calibration
            pos = 0;                 // Reset pos to 0
            display.clearDisplay();  // Clear the display
          }
        }
        break;

      case 2:  // Low Point Calibration
        if (rawData == LEFTIR || rawData == UPIR) {
          pos = 1;  // Go back to Dry Calibration
        } else if (rawData == ENTERIR) {
          if (pos == 0) {
            Serial.println("Starting Low Point Calibration");
            runLowCalibration();
            calibrationStep = 3;     // Move to High Point Calibration
            pos = 0;                 // Reset pos to 0
            display.clearDisplay();  // Clear the display
          } else if (pos == 1) {
            Serial.println("Cancelling Calibration");
            calibrationStep = 0;     // Go back to Start Calibration
            pos = 0;                 // Reset pos to 0
            display.clearDisplay();  // Clear the display
          }
        } else if (rawData == DOWNIR) {
          pos = 3;  // Move to High Point Calibration
        }
        break;

      case 3:  // High Point Calibration
        if (rawData == LEFTIR || rawData == UPIR) {
          pos = 2;  // Go back to Low Point Calibration
        } else if (rawData == ENTERIR) {
          if (pos == 0) {
            Serial.println("Starting High Point Calibration");
            runHighCalibration();
            calibrationStep = 0;     // Calibration complete, go back to Start Calibration
            pos = 0;                 // Reset pos to 0
            display.clearDisplay();  // Clear the display
          } else if (pos == 1) {
            Serial.println("Cancelling Calibration");
            calibrationStep = 0;     // Go back to Start Calibration
            pos = 0;                 // Reset pos to 0
            display.clearDisplay();  // Clear the display
          }
        }
        break;
    }

    IrReceiver.resume();  // Continue receiving IR signals
  }
}


// //<-------------------------------------------------------------------------------------------------------------------------------------------------------------->
// //<----------------------------------------------------------------------FUNCTIONS------------------------------------------------------------------------------->
// //<-------------------------------------------------------------------------------------------------------------------------------------------------------------->

time_t getTeensy3Time() {     // Time variable getTeensy3Time
  return Teensy3Clock.get();  // Get the time from the Teensy clock
}

void getNow() {
  nowSec = second();        // Name seconds nowSec
  nowMin = minute();        // Name minutes nowMin
  nowHr = hour();           // Name hour nowHr
  nowDay = day();           // Name day nowDay
  nowMon = month();         // Name month nowMon
  nowYr = (year() - 2000);  // Name years since 2000 nowYr
}

//Valves
void valvePosCorrect(int direction) {
  // Adjust the selected valve index with direction
  selectedValvePositionIndex += direction;
  // Clamp to ensure the intermediate index is between 1 and 16
  if (selectedValvePositionIndex < 1)
    selectedValvePositionIndex = 16;
  if (selectedValvePositionIndex > 16)
    selectedValvePositionIndex = 1;
}

// Helper to read encoder atomically
const int ENCODER_DIRECTION = -1; // set to 1 or -1 depending on wiring

long readEncoder() {
  long val;
  noInterrupts();
  val = encoderCount;
  interrupts();
  return ENCODER_DIRECTION * val;
}

void step_to_target(long target) {
  long currentPos = readEncoder();
  Serial.println("\n=== STEP_TO_TARGET DEBUG ===");
  Serial.println("Current position: " + String(currentPos));
  Serial.println("Target position: " + String(target));
  if (currentPos == target) {
    Serial.println("Already at target position!");
    return;
  }
  bool moveForward = (target > currentPos);
  digitalWrite(stepperDir, moveForward ? HIGH : LOW);
  digitalWrite(stepperMode2, LOW);
  digitalWrite(stepperMode1, LOW);
  digitalWrite(stepperMode0, HIGH);
  digitalWrite(stepperOn, HIGH);
  int stepCount = 0;
  while (true) {
    long pos = readEncoder();
    if (moveForward) {
      if (pos >= target) break;
    } else {
      if (pos <= target) break;
    }
    Serial.println("EncoderCount: " + String(pos));
    digitalWrite(stepperStep, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(stepperStep, LOW);
    delayMicroseconds(STEP_DELAY_US);
    stepCount++;
    if (stepCount > 5000) {
      Serial.println("TIMEOUT!");
      break;
    }
  }
  long finalPos = readEncoder();
  Serial.println("Final encoder position: " + String(finalPos));
  digitalWrite(stepperOn, LOW);
  delay(100);
}

void adjustValvePosition() {
  Serial.println("Current Valve Position Index: " + String(currentValvePositionIndex));
  Serial.println("Selected Valve Position Index: " + String(selectedValvePositionIndex));
  if (selectedValvePositionIndex == currentValvePositionIndex) {
    Serial.println("Already at target valve position.");
    return;
  }
  long currentPos = readEncoder();
  long targetPos = valveTargets[selectedValvePositionIndex - 1];
  Serial.println("Current encoder position: " + String(currentPos));
  Serial.println("Target encoder position: " + String(targetPos));
  step_to_target(targetPos);
  Serial.println("Saving new valve position to EEPROM...");
  EEPROM.update(eepromValvePosAddress, selectedValvePositionIndex);
  delay(100);
  int verifyValvePosition;
  EEPROM.get(eepromValvePosAddress, verifyValvePosition);
  Serial.println("Verifying EEPROM Write...");
  if (verifyValvePosition == selectedValvePositionIndex) {
    Serial.println("SUCCESS: EEPROM updated correctly. New Valve Position: " + String(verifyValvePosition));
  } else {
    Serial.println("ERROR: EEPROM write failed! Read: " + String(verifyValvePosition) + ", Expected: " + String(selectedValvePositionIndex));
  }
  currentValvePositionIndex = selectedValvePositionIndex;
  Serial.println("Valve position adjustment complete.");
}

void sampleValvePosition() {
  Serial.println("Current Valve Position Index: " + String(currentValvePositionIndex));
  Serial.println("Selected Valve Position Index: " + String(valveSelection));
  if (valveSelection == currentValvePositionIndex) {
    Serial.println("Already at target valve position.");
    return;
  }
  long currentPos = readEncoder();
  long targetPos = valveTargets[valveSelection - 1];
  Serial.println("Current encoder position: " + String(currentPos));
  Serial.println("Target encoder position: " + String(targetPos));
  step_to_target(targetPos);
  Serial.println("Saving new valve position to EEPROM...");
  EEPROM.update(eepromValvePosAddress, valveSelection);
  delay(100);
  int verifyValvePosition;
  EEPROM.get(eepromValvePosAddress, verifyValvePosition);
  Serial.println("Verifying EEPROM Write...");
  if (verifyValvePosition == valveSelection) {
    Serial.println("SUCCESS: EEPROM updated correctly. New Valve Position: " + String(verifyValvePosition));
  } else {
    Serial.println("ERROR: EEPROM write failed! Read: " + String(verifyValvePosition) + ", Expected: " + String(valveSelection));
  }
  currentValvePositionIndex = selectedValvePositionIndex;
  Serial.println("Valve position adjustment complete.");
}

void moveValveOnePlus() {
  Serial.println("Moving valve one encoder count forward...");
  long currentPos = readEncoder();
  long targetPos = currentPos + 1;
  Serial.println("Current encoder position: " + String(currentPos));
  Serial.println("Target encoder position: " + String(targetPos));
  step_to_target(targetPos);
  Serial.println("Forward movement complete.");
}

void moveValveOneMinus() {
  Serial.println("Moving valve one encoder count backward...");
  long currentPos = readEncoder();
  long targetPos = currentPos - 1;
  Serial.println("Current encoder position: " + String(currentPos));
  Serial.println("Target encoder position: " + String(targetPos));
  step_to_target(targetPos);
  Serial.println("Backward movement complete.");
}


void resetZeroPosition() {
  Serial.println("Resetting zero valve position.");
  digitalWrite(stepperReset, HIGH);
  digitalWrite(stepperOn, HIGH);
  delay(50);
  digitalWrite(stepperDir, LOW);  // Move towards home position
  // Set microstepping
  digitalWrite(stepperMode2, LOW);
  digitalWrite(stepperMode1, LOW);
  digitalWrite(stepperMode0, HIGH);
  int signalCount = 0;
  int lastState = digitalRead(photoHomePin);
  Serial.print("Photo Sensor Initial State: ");
  Serial.println(lastState);
  while (signalCount < 7) {  // Stop on the 7th HIGH signal
    // Step the motor
    digitalWrite(stepperStep, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(stepperStep, LOW);
    delayMicroseconds(STEP_DELAY_US);
    //Read current state of the photogate sensor
    int currentState = digitalRead(photoHomePin);
    if (currentState == 1) {
      signalCount++;
      Serial.print("Photogate trigger #");
      Serial.println(signalCount);
    }
  }
  Serial.println("7th photogate trigger reached. Stopping motor.");
  // Reset raw encoder count to zero
  noInterrupts();
  encoderCount = 0;
  interrupts();
  // Power down stepper
  digitalWrite(stepperMode2, LOW);
  digitalWrite(stepperMode1, LOW);
  digitalWrite(stepperMode0, LOW);
  digitalWrite(stepperOn, LOW);
  // Update current valve position
  currentValvePositionIndex = 1;  // Valve 1 is now zero position
  Serial.println("Zero position reset complete.");
}

void zeroValveNow() {
  Serial.println("Starting zeroing process and recording steps.");
  digitalWrite(stepperReset, HIGH);
  digitalWrite(stepperOn, HIGH);
  delay(50);
  digitalWrite(stepperDir, LOW);  // Move towards home position
  // Set microstepping
  digitalWrite(stepperMode2, LOW);
  digitalWrite(stepperMode1, LOW);
  digitalWrite(stepperMode0, HIGH);
  int signalCount = 0;
  int lastState = digitalRead(photoHomePin);
  Serial.print("Photo Sensor Initial State: ");
  Serial.println(lastState);
  while (signalCount < 7) {  // Stop on the 7th HIGH signal
    // Step the motor
    digitalWrite(stepperStep, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(stepperStep, LOW);
    delayMicroseconds(STEP_DELAY_US);
     // Read current state of the photogate sensor
    int currentState = digitalRead(photoHomePin);
    if (currentState == 1) {
      signalCount++;
      Serial.print("Photogate trigger #");
      Serial.println(signalCount);
    }
  }
  Serial.println("7th photogate trigger reached. Stopping motor.");
  // Reset raw encoder count to zero
  noInterrupts();
  encoderCount = 0;
  interrupts();
  // Update current valve position (valve 1 is zero)
  currentValvePositionIndex = 1;
  // Power down stepper
  digitalWrite(stepperMode2, LOW);
  digitalWrite(stepperMode1, LOW);
  digitalWrite(stepperMode0, LOW);
  digitalWrite(stepperOn, LOW);
  Serial.println("Zeroing complete.");
}

//3 Way Valve
void openValve() {
  digitalWrite(threeWayPin, HIGH);
  Serial.println("Valve open.");
}

void closeValve() {
  digitalWrite(threeWayPin, LOW);
  Serial.println("Valve closed.");
}

void testValve() {
  if (!threeWayValveOpen) {
    digitalWrite(threeWayPin, HIGH);
    Serial.println("Valve open.");
    IrReceiver.resume();
    threeWayValveOpen = true;
    waitingForThreeWayClose = true;
  }
  if (waitingForThreeWayClose && IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.decodedRawData == ENTERIR) {
      digitalWrite(threeWayPin, LOW);
      Serial.println("Valve closed.");
      waitingForThreeWayClose = false;
      threeWayValveOpen = false;
    }
    IrReceiver.resume();
  }
}

//Pressure
void getPressureTemp() {
  // Set sensor model and fluid density
  sensor.setModel(MS5837::MS5837_30BA);  // Set the sensor model (Blue Robotics Bar30)
  sensor.setFluidDensity(1029);          // Set fluid density, for freshwater 997, for seawater 1029 (kg/m^3)
  // Allow the sensor to stabilize and perform the read
  sensor.read();
  // Read sensor values: pressure, temperature, and depth
  nowPressure = sensor.pressure();        // Pressure in bar, adjust for units if needed
  nowTemperature = sensor.temperature();  // Temperature in Celsius
  nowDepth = sensor.depth();              // Depth in meters
  Serial.println("Pressure : " + String(nowPressure));
  Serial.println("Temperature : " + String(nowTemperature));
  Serial.println("Depth : " + String(nowDepth));
}

//Light
void getLight() {
  // Read the analog value from the phototransistor
  lightLevel = analogRead(lightPin);
  Serial.println("Light level: " + String(lightLevel));
}

//Salinity
void getSalinity() {
  unsigned long currentMillis = millis();
  // Only call this every second
  if (currentMillis - lastECUpdateTime >= ecUpdateInterval) {
    lastECUpdateTime = currentMillis;   // Update the last update time
    digitalWrite(ecOffPin, HIGH);       // Turn on the EC sensor
    int currentECValue = getECValue();  // Get EC value from the sensor
    if (currentECValue != -1) {         // Valid EC value received
      ecValue = currentECValue;         // Update global EC value
      Serial.print("EC Value: ");
      Serial.println(ecValue);
    } else {
      Serial.println("Error reading EC value.");
    }
  }
}

int getECValue() {
  myserial.print("R");   // Send the command to request EC data
  myserial.print('\r');  // Add <CR> to the command
  unsigned long start = millis();
  sensorstring = "";  // Clear the sensor string
  sensor_string_complete = false;
  // Wait for the response or timeout
  while (!sensor_string_complete && millis() - start < 2000) {
    if (myserial.available() > 0) {
      char inchar = (char)myserial.read();
      sensorstring += inchar;
      if (inchar == '\r') {
        sensor_string_complete = true;
      }
    }
  }
  if (!sensor_string_complete) {
    return -1;  // Timeout or no response
  }
  char sensorstring_array[30];
  char* EC;
  sensorstring.toCharArray(sensorstring_array, 30);
  EC = strtok(sensorstring_array, ",");  // Parse EC value
  if (EC != nullptr) {
    return atoi(EC);  // Convert EC value to int using atoi(), if the sensor provides decimal values, switch to atof()
  } else {
    return -1;  // Parsing failed
  }
}

void runDryCalibration() {
  // Perform dry calibration
  delay(3000);  // Simulate calibration duration
  isCalibrating = false;
  calibrationStep = 2;  // Move to Low Point Calibration
}

void runLowCalibration() {
  // Perform low point calibration
  delay(3000);  // Simulate calibration duration
  isCalibrating = false;
  calibrationStep = 3;  // Move to High Point Calibration
}

void runHighCalibration() {
  // Perform high point calibration
  delay(3000);  // Simulate calibration duration
  isCalibrating = false;
  calibrationStep = 0;  // Return to Start Calibration
  display.setCursor(0, 5);
  display.print("Calibration complete.");
  delay(3000);
}

//Voltage
void getVoltage() {
  unsigned long currentMillis = millis();  // Get the current time
  // Check if enough time has passed since the last update
  if (currentMillis - lastVoltageUpdateTime >= voltageUpdateInterval) {
    lastVoltageUpdateTime = currentMillis;  // Update the last update time
    int rawVoltage = analogRead(voltageReadPin);
    float batteryVoltage = ((rawVoltage * (vRef / 4095.00)) / (resistorB / (resistorA + resistorB)));
    // Update the sample array and calculate the average
    sum -= voltageSamples[sampleIndex];            // Remove oldest sample
    voltageSamples[sampleIndex] = batteryVoltage;  // Add new sample
    sum += voltageSamples[sampleIndex];            // Add new value to sum
    sampleIndex = (sampleIndex + 1) % numSamples;  // Move to the next index
    averagedVoltage = sum / numSamples;            // Calculate the average
    Serial.println("Average voltage: " + String(averagedVoltage));
  }
}

void nowVoltage() {
  int rawVoltage = analogRead(voltageReadPin);
  delay(100);                                                                                  // Delay to allow processing of analog pin data
  batteryVoltage = ((rawVoltage * (vRef / 4095.00)) / (resistorB / (resistorA + resistorB)));  // Update the global variable
  Serial.println("Battery voltage: " + String(batteryVoltage));                                // Print correct variable
}

//Pumps
void runACalibration() {  //Calibration step uses encoder count and timed volume collection to determine a volume per rotation calibration for sampling volumes
  digitalWrite(pumpsOn, LOW);
  delay(500);
  digitalWrite(pumpADirPin, HIGH);
  digitalWrite(pumpAEncPowerPin, HIGH);  //Power on encoder pin
  pumpAEnc.write(0);                     //reset encoder pin
  calibrationACount = 0.0;               //reset pump A encoder pin count to 0
  digitalWrite(pumpARunPin, HIGH);
  delay(30000);                                          // Power on Pump A for 30 seconds
  calibrationACount = abs(pumpAEnc.read());  //Measure the count of the encoder after 30 seconds of running pump
  digitalWrite(pumpAEncPowerPin, LOW);                   //turn off Pump A encoder pin
  digitalWrite(pumpARunPin, LOW);                        // turn off Pump A
  saveSampleSettings();                                  //Record the encoder count to sd card
}

void runBCalibration() {  //Calibration step uses encoder count and timed volume collection to determine a volume per rotation calibration for sampling volumes
  digitalWrite(pumpsOn, LOW);
  delay(500);
  digitalWrite(pumpBDirPin, HIGH);
  digitalWrite(pumpBEncPowerPin, HIGH);  //Power on encoder pin
  pumpBEnc.write(0);                     //reset encoder pin
  calibrationBCount = 0.0;               //reset pump A encoder pin count to 0
  digitalWrite(pumpBRunPin, HIGH);
  delay(30000);                                          // Power on Pump A for 30 seconds
  calibrationBCount = abs(pumpBEnc.read());  //Measure the count of the encoder after 30 seconds of running pump and divide by gear ratio to get rotations of pump
  digitalWrite(pumpBEncPowerPin, LOW);                   //turn off Pump A encoder pin
  digitalWrite(pumpBRunPin, LOW);                        // turn off Pump A
  saveSampleSettings();                                  //Record the encoder count to sd card
}

void runPumpA() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0, 0);
  display.println("PUMP A!");  // Display PUMP A
  display.display();           // Update the OLED display
  delay(100);
  waitforit = 0;
  digitalWrite(pumpsOn, LOW);
  delay(500);
  Serial.println("Starting pump A.");
  digitalWrite(pumpAEncPowerPin, HIGH);
  pumpAEnc.write(0);
  // Debugging line to monitor encoder readings
  Serial.print("Encoder count: ");
  Serial.println(pumpAEnc.read());
  if (pumpADirection == 0) {
    digitalWrite(pumpADirPin, HIGH);
    Serial.println("CW");
  } else {
    digitalWrite(pumpADirPin, LOW);
    Serial.println("CCW");
  }
  digitalWrite(pumpARunPin, HIGH);
  IrReceiver.resume();  // Continue receiving looking for IR signals
  while (waitforit == 0) {
    if (IrReceiver.decode()) {  // If there is an IR signal, return true and store signal in "results"
      if (IrReceiver.decodedIRData.decodedRawData == ENTERIR) {
        waitforit = 1;
        digitalWrite(pumpARunPin, LOW);
        Serial.println("Final Pump A encoder reading:");
        Serial.println(pumpAEnc.read());
        digitalWrite(pumpAEncPowerPin, LOW);
        digitalWrite(pumpsOn, HIGH);
      }
      IrReceiver.resume();  // Continue receiving looking for IR signals
    }
  }
}

void runPumpB() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0, 0);
  display.println("PUMP B!");  // Display PUMP B
  display.display();           // Update the OLED display
  delay(100);
  waitforit = 0;
  digitalWrite(pumpsOn, LOW);
  delay(500);
  Serial.println("Starting pump B.");
  digitalWrite(pumpBEncPowerPin, HIGH);
  pumpBEnc.write(0);
  // Debugging line to monitor encoder readings
  Serial.print("Encoder count: ");
  Serial.println(abs(pumpBEnc.read()));
  if (pumpBDirection == 0) {
    digitalWrite(pumpBDirPin, HIGH);
    Serial.println("CW");
  } else {
    digitalWrite(pumpBDirPin, LOW);
    Serial.println("CCW");
  }
  digitalWrite(pumpBRunPin, HIGH);
  IrReceiver.resume();  // Continue receiving looking for IR signals
  while (waitforit == 0) {
    if (IrReceiver.decode()) {  // If there is an IR signal, return true and store signal in "results"
      if (IrReceiver.decodedIRData.decodedRawData == ENTERIR) {
        waitforit = 1;
        digitalWrite(pumpBRunPin, LOW);
        Serial.println("Final Pump B encoder reading:");
        Serial.println(abs(pumpBEnc.read()));
        digitalWrite(pumpBEncPowerPin, LOW);
        digitalWrite(pumpsOn, HIGH);
      }
      IrReceiver.resume();  // Continue receiving looking for IR signals
    }
  }
}

void runSystemPrime(){
  bool interrupted = false;  // Flag to check if IR interrupt happened
  for (int pos = 3; pos <= 16; pos++) {
    resetZeroPosition(); // <-- Re-zero before each valve run
    valveSelection = pos;  // Set the target valve
    sampleValvePosition(); // Move to the valve
    // Run pump A for 30 seconds (30000 ms)
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(0, 0);
    display.println("PUMP A");
    display.println("Valve " + String(pos));
    display.display();
    Serial.println("Running pump A at valve position: " + String(pos));
    digitalWrite(pumpsOn, LOW);
    delay(1000);
    digitalWrite(pumpADirPin, HIGH); // CW
    digitalWrite(pumpARunPin, HIGH);   // Start the pump
    unsigned long startTime = millis();
    int secondsElapsed = 0;
    IrReceiver.resume(); // Start listening
    while (millis() - startTime < 10000) {
      int currentSeconds = (millis() - startTime) / 1000;
      if (currentSeconds != secondsElapsed) {
        secondsElapsed = currentSeconds;
        Serial.println("Pump running: " + String(secondsElapsed) + " sec");
        // Clear just the number area or entire display
        display.fillRect(0, 36, 128, 16, BLACK); // Clear previous time (optional)
        display.setCursor(0, 36);
        display.println(secondsElapsed);
        display.display(); // Refresh screen to show the updated value
      }
      // Check IR input
      if (IrReceiver.decode()) {
        if (IrReceiver.decodedIRData.decodedRawData == ENTERIR) {
          Serial.println("IR interrupt detected. Stopping pump early.");
          interrupted = true; // Set the interrupt flag
          break; // Break out of the loop        
          }
        IrReceiver.resume(); // Continue listening
      }
    }
    digitalWrite(pumpARunPin, LOW);    // Stop the pump
    digitalWrite(pumpsOn, HIGH);
    Serial.println("Pump A done at valve position: " + String(pos));
    delay(1000); // Optional delay between valves
    // If interrupted, break out of the for loop and go back to primeMenu
    if (interrupted) {
      Serial.println("Returning to Prime Menu...");
      return; // Return to the menu or control loop after interruption
    }
  }
}

void runCleanSystem(){
  bool interrupted = false;  // Flag to check if IR interrupt happened
  for (int pos = 1; pos <= 16; pos++) {
    resetZeroPosition(); // <-- Re-zero before each valve run
    valveSelection = pos;  // Set the target valve
    sampleValvePosition(); // Move to the valve
    // Run pump A for 30 seconds (30000 ms)
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(0, 0);
    display.println("PUMP A");
    display.println("Valve " + String(pos));
    display.display();
    Serial.println("Running pump A at valve position: " + String(pos));
    digitalWrite(pumpsOn, LOW);
    delay(1000);
    digitalWrite(pumpADirPin, HIGH); // CW
    digitalWrite(pumpARunPin, HIGH);   // Start the pump
    unsigned long startTime = millis();
    int secondsElapsed = 0;
    IrReceiver.resume(); // Start listening
    //while (millis() - startTime < (cleanVolume*1000)) {
    while (millis() - startTime < (unsigned long)(cleanVolume*1000)){
      int currentSeconds = (millis() - startTime) / 1000;
      if (currentSeconds != secondsElapsed) {
        secondsElapsed = currentSeconds;
        Serial.println("Pump running: " + String(secondsElapsed) + " sec");
        // Clear just the number area or entire display
        display.fillRect(0, 36, 128, 16, BLACK); // Clear previous time (optional)
        display.setCursor(0, 36);
        display.println(secondsElapsed);
        display.display(); // Refresh screen to show the updated value
      }
      // Check IR input
      if (IrReceiver.decode()) {
        if (IrReceiver.decodedIRData.decodedRawData == ENTERIR) {
          Serial.println("IR interrupt detected. Stopping pump early.");
          interrupted = true; // Set the interrupt flag
          break; // Break out of the loop        
          }
        IrReceiver.resume(); // Continue listening
      }
    }
    digitalWrite(pumpARunPin, LOW);    // Stop the pump
    digitalWrite(pumpsOn, HIGH);
    Serial.println("Pump A done at valve position: " + String(pos));
    delay(1000); // Optional delay between valves
    // If interrupted, break out of the for loop and go back to primeMenu
    if (interrupted) {
      Serial.println("Returning to Prime Menu...");
      return; // Return to the menu or control loop after interruption
    }
  }
}

void samplePumpA() {
  digitalWrite(pumpsOn, LOW);
  delay(500);
  Serial.println("Starting pump A.");
  digitalWrite(pumpAEncPowerPin, HIGH);
  pumpAEnc.write(0);  // Reset encoder reading
  // Calculate pump run time
  pumpARunTime = round(pumpAVolumeTemp * (calibrationACount/calibrationAVolume));
  // Debugging line to monitor encoder readings
  Serial.print("Encoder goal: " + String(pumpARunTime));
  Serial.print("Encoder count: ");
  Serial.println(abs(pumpAEnc.read()));
  // Set pump direction
  if (pumpADirection == 0) {
    digitalWrite(pumpADirPin, HIGH);
    Serial.println("CW");
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(0, 0);
    display.println(messageTemp);
    display.println("V" + String(valveSelection));
    display.display();
    delay(100);
  } else {
    digitalWrite(pumpADirPin, LOW);
    Serial.println("CCW");
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(0, 0);
    display.println("HCl Clean");
    display.display();
    delay(100);
  }
  digitalWrite(pumpARunPin, HIGH);  // Start the pump
  // Keep running the pump until the encoder count reaches pumpARunTime
  while (abs(pumpAEnc.read()) < pumpARunTime){
    Serial.print("Encoder count: ");
    Serial.println(abs(pumpAEnc.read())/(calibrationACount / calibrationAVolume));
    delay(50);  // Small delay to allow encoder readings to update
  }
  // Stop the pump when the encoder count is reached
  digitalWrite(pumpARunPin, LOW);
  Serial.println("Final Pump A encoder reading:");
  Serial.println(abs(pumpAEnc.read())/(calibrationACount / calibrationAVolume));
  digitalWrite(pumpAEncPowerPin, LOW);
  digitalWrite(pumpsOn, HIGH);
}

void samplePumpB() {
  digitalWrite(pumpsOn, LOW);
  delay(500);
  Serial.println("Starting pump B.");
  digitalWrite(pumpBEncPowerPin, HIGH);
  pumpBEnc.write(0);  // Reset encoder reading
  // Calculate pump run time
  pumpBRunTime = round(pumpBVolume * (calibrationBCount/calibrationBVolume));
  // Debugging line to monitor encoder readings
  Serial.print("Encoder goal: " + String(pumpBRunTime));
  Serial.print("Encoder count: ");
  Serial.println(abs(pumpBEnc.read()));
  // Set pump direction
  pumpBDirection = 0;  //Pump B preservative pump will always be CW
  digitalWrite(pumpBDirPin, HIGH);
  Serial.println("CW");
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0, 0);
  display.println("Preserving");
  display.println("V" + String(valveSelection));
  display.display();
  delay(100);
  digitalWrite(pumpsOn, LOW);
  delay(500);
  digitalWrite(pumpBRunPin, HIGH);  // Start the pump
  // Keep running the pump until the encoder count reaches pumpARunTime
  while (abs(pumpBEnc.read()) < pumpBRunTime){
    Serial.print("Encoder count: ");
    Serial.println(abs(pumpBEnc.read())/(calibrationBCount / calibrationBVolume));
    delay(50);  // Small delay to allow encoder readings to update
  }
  // Stop the pump when the encoder count is reached
  digitalWrite(pumpBRunPin, LOW);
  Serial.println("Final Pump B encoder reading:");
  Serial.println(abs(pumpBEnc.read())/(calibrationBCount/calibrationBVolume));
  digitalWrite(pumpBEncPowerPin, LOW);
  digitalWrite(pumpsOn, HIGH);
}

//Time
// Parse SD card date time rows into date-time objects
time_t parseDateTime(const String& dateTimeStr) {
  // Split the string by commas
  int commaIndex1 = dateTimeStr.indexOf(',');
  int commaIndex2 = dateTimeStr.indexOf(',', commaIndex1 + 1);
  int commaIndex3 = dateTimeStr.indexOf(',', commaIndex2 + 1);
  int commaIndex4 = dateTimeStr.indexOf(',', commaIndex3 + 1);
  int month = dateTimeStr.substring(0, commaIndex1).toInt();
  int day = dateTimeStr.substring(commaIndex1 + 1, commaIndex2).toInt();
  int fullYear = dateTimeStr.substring(commaIndex2 + 1, commaIndex3).toInt();
  int hour = dateTimeStr.substring(commaIndex3 + 1, commaIndex4).toInt();
  int minute = dateTimeStr.substring(commaIndex4 + 1).toInt();  // No second comma
  // Print parsed values to ensure they are correct
  Serial.print("Parsed values - Year: ");
  Serial.print(fullYear);
  Serial.print(", Month: ");
  Serial.print(month);
  Serial.print(", Day: ");
  Serial.print(day);
  Serial.print(", Hour: ");
  Serial.print(hour);
  Serial.print(", Minute: ");
  Serial.println(minute);
  tmElements_t myElements;
  myElements.Year = fullYear - 1970;  // Adjust to the Unix epoch base year
  myElements.Month = month;
  myElements.Day = day;
  myElements.Hour = hour;
  myElements.Minute = minute;
  myElements.Second = 0;
  time_t parsedTime = makeTime(myElements);
  // Print the parsed time in UNIX timestamp format
  Serial.print("Parsed time_t: ");
  Serial.println(parsedTime);
  return parsedTime;
}

// Function to find the next wake-up time and set sampleValve
time_t findNextWakeUpTimeFromSD() {
  readSampleSettings(); //Load existing sample settings
  time_t nextTime = 0;           // Initialize with 0 (no valid time found)
  bool foundFutureTime = false;  // Flag to check if a future time was found
  int rowNumber = 0;             // Track the current row number
  // Open the file
  File dataFile = SD.open("sampleTimesSd.txt");
  if (dataFile) {
    Serial.println("Trying to open SD card file");
    while (dataFile.available()) {
      rowNumber++;  // Track which row we're on
      String line = dataFile.readStringUntil('\n');
      line.trim();  // Remove extra spaces/newlines
      Serial.print("Reading line: ");
      Serial.println(line);
      // Parse the date-time from the line
      time_t parsedTime = parseDateTime(line);
      Serial.print("Parsed time from line: ");
      Serial.println(parsedTime);
      // Get the current time
      time_t currentTime = now();
      Serial.print("Current time: ");
      Serial.println(currentTime);
      // Compare with current time
      if (parsedTime > currentTime) {
        Serial.println("Parsed time is in the future.");
        foundFutureTime = true;  // We found at least one future time
        // Update nextTime if it's the first future time or the closest one
        if (nextTime == 0 || parsedTime < nextTime) {
          nextTime = parsedTime;        // Set the new next wake-up time
          sampleValve = rowNumber + 2;  // Set sampleValve based on row number
          Serial.print("Updated nextTime: ");
          Serial.println(nextTime);
          Serial.print("Updated sampleValve: ");
          Serial.println(sampleValve);
        }
      } else {
        Serial.println("Parsed time is in the past or equal to now.");
      }
    }
    dataFile.close();
  } else {
    Serial.println("Failed to open sampleTimesSd.txt");
  }
  // If no future times were found, set sampleValve to 255
  if (!foundFutureTime) {
    nextTime = NO_MORE_SAMPLES;  // Indicate no future samples
    sampleValve = 255;           // Set sampleValve to 255
    Serial.println("No more samples found. Setting sampleValve to 255.");
  }
  // Print final values
  Serial.print("Final Next Wake-Up Time: ");
  Serial.println(nextTime);
  Serial.print("Final sampleValve: ");
  Serial.println(sampleValve);
  saveSampleSettings();
  return nextTime;  // Return the next wake-up time
}

// Adjust the value of the selected component (month, day, etc.) based on the selected row
void adjustSelectedComponent(int change) {
  String currentSampleTime = sampleTimes[pos];               // Get the currently selected sample time
  int month = currentSampleTime.substring(0, 2).toInt();     // Extract the current month
  int day = currentSampleTime.substring(3, 5).toInt();       // Extract the current day
  int year = currentSampleTime.substring(6, 8).toInt();      // Extract the current year
  int hour = currentSampleTime.substring(9, 11).toInt();     // Extract the current hour
  int minute = currentSampleTime.substring(12, 14).toInt();  // Extract the current minute
  switch (selectedComponent) {
    case COMPONENT_MONTH:
      month += change;
      if (month > 12) {
        month = 1;  // Wrap around to 1 if it goes beyond 12
      } else if (month < 1) {
        month = 12;  // Wrap around to 12 if it goes below 1
      }
      break;
    case COMPONENT_DAY:
      day += change;
      // Adjust days for months and leap years if needed
      if (month == 2) {                                                    // February (simplified, no leap year handling here)
        day = constrain(day, 1, 28);                                       // Max 28 days in February
      } else if (month == 4 || month == 6 || month == 9 || month == 11) {  // Months with 30 days
        day = constrain(day, 1, 30);                                       // Max 30 days
      } else {                                                             // All other months (31 days)
        day = constrain(day, 1, 31);                                       // Max 31 days
      }
      break;
    case COMPONENT_YEAR:
      year += change;
      break;
    case COMPONENT_HOUR:
      hour += change;
      if (hour > 23) {
        hour = 0;  // Wrap around to 0 if it goes beyond 23
      } else if (hour < 0) {
        hour = 23;  // Wrap around to 23 if it goes below 0
      }
      break;
    case COMPONENT_MINUTE:
      minute += change;
      if (minute > 59) {
        minute = 0;  // Wrap around to 0 if it goes beyond 59
      } else if (minute < 0) {
        minute = 59;  // Wrap around to 59 if it goes below 0
      }
      break;
  }
  // After adjusting, format the new sample time and save it back to the array
  sampleTimes[pos] = String(month < 10 ? "0" : "") + String(month) + "/" + String(day < 10 ? "0" : "") + String(day) + "/" + String(year < 10 ? "0" : "") + String(year) + " " + String(hour < 10 ? "0" : "") + String(hour) + ":" + String(minute < 10 ? "0" : "") + String(minute);
}

// Function to update the sampleTimes array after editing
void updateSampleTime() {
  // Get the currently selected sample time
  String currentSampleTime = sampleTimes[pos];
  // Extract current components
  int month = currentSampleTime.substring(0, 2).toInt();
  int day = currentSampleTime.substring(3, 5).toInt();
  int year = currentSampleTime.substring(6, 8).toInt();
  int hour = currentSampleTime.substring(9, 11).toInt();
  int minute = currentSampleTime.substring(12, 14).toInt();
  // Format the new sample time using the current components
  String newTime = String(month < 10 ? "0" : "") + String(month) + "/";
  newTime += String(day < 10 ? "0" : "") + String(day) + "/";
  newTime += String(year < 10 ? "0" : "") + String(year) + " ";
  newTime += String(hour < 10 ? "0" : "") + String(hour) + ":";
  newTime += String(minute < 10 ? "0" : "") + String(minute);
  // Update the sampleTimes array
  sampleTimes[pos] = newTime;
}

void numberCorrect() {
  //Settings correct
  if (pumpAVolume < 10) { pumpAVolume = 2000; }                                                                                                                  // If sample volume is less than 10, make it 2000
  if (pumpAVolume > 2000) { pumpAVolume = 10; }                                                                                                                  // If sample volume is greater than 2000, make it 10
  if (pumpBVolume < 1) { pumpBVolume = 20; }                                                                                                                     // If preservative volume is less than 1, make it 20
  if (pumpBVolume > 20) { pumpBVolume = 1; } 
  if (cleanVolume < 1) { cleanVolume = 50; }                                                                                                                     // If preservative volume is less than 1, make it 20
  if (cleanVolume > 50) { cleanVolume = 1; }                                                                                                                     // If sample volume is greater than 20, make it 1
  if (calibrationAVolume < 1) { calibrationAVolume = 75; }                                                                                                       // If sample calib volume is less than 1, make it 75
  if (calibrationAVolume > 75) { calibrationAVolume = 1; }                                                                                                       // If sample calib volume is greater than 75, make it 1
  if (calibrationBVolume < 1) { calibrationBVolume = 75; }                                                                                                       // If preservative calib volume is less than 1, make it 75
  if (calibrationBVolume > 75) { calibrationBVolume = 1; }                                                                                                       // If preservative calib volume is greater than 75, make it 1
                                                                                                                                                                 //Date and Time Correct
  if (((nowYr != 28) && (nowYr != 32) && (nowYr != 36) && (nowYr != 40) && (nowYr != 44)) && (nowMon == 2) && (nowDay > 28)) { nowDay = 1; }                     // Sets max day in Feb for non leap years up to 2044, and loops to 1 if you exceed
  if (((nowYr == 28) || (nowYr == 32) || (nowYr == 36) || (nowYr == 40) || (nowYr == 44)) && (nowMon == 2) && (nowDay > 29)) { nowDay = 1; }                     // Sets max day in Feb for leap years up to 2044, and loops to 1 if you exceed
  if (((nowMon == 4) || (nowMon == 6) || (nowMon == 9) || (nowMon == 11)) && (nowDay > 30)) { nowDay = 1; }                                                      // Sets max days for April, June, September, November, and loops to 1 if you exceed
  if (((nowMon == 1) || (nowMon == 3) || (nowMon == 5) || (nowMon == 7) || (nowMon == 8) || (nowMon == 10) || (nowMon == 12)) && (nowDay > 31)) { nowDay = 1; }  // Sets max days for January, March, May, July, August, October, December, and loops to 1 if you exceed
  if (((nowYr != 28) && (nowYr != 32) && (nowYr != 36) && (nowYr != 40) && (nowYr != 44)) && (nowMon == 2) && (nowDay < 1)) { nowDay = 28; }                     // Sets max day in Feb for non leap years up to 2044, and loops to max if you go lower than 1
  if (((nowYr == 28) || (nowYr == 32) || (nowYr == 36) || (nowYr == 40) || (nowYr == 44)) && (nowMon == 2) && (nowDay < 1)) { nowDay = 29; }                     // Sets max day in Feb for leap years up to 2044, and loops to max if you go lower than 1
  if (((nowMon == 4) || (nowMon == 6) || (nowMon == 9) || (nowMon == 11)) && (nowDay < 1)) { nowDay = 30; }                                                      // Sets max days for April, June, September, November, and loops to max if you go lower than 1
  if (((nowMon == 1) || (nowMon == 3) || (nowMon == 5) || (nowMon == 7) || (nowMon == 8) || (nowMon == 10) || (nowMon == 12)) && (nowDay < 1)) { nowDay = 31; }  // Sets max days for January, March, May, July, August, October, and loops to max if you go lower than 1
  if (nowYr < 0) { nowYr = 0; }                                                                                                                                  // Sets minimum year to 0 or 2000
  if (nowMon > 12) { nowMon = 1; }                                                                                                                               // If max month (December) is exceeded, loop to January
  if (nowMon < 1) { nowMon = 12; }                                                                                                                               // If min month (January) is exceeded, loop to December
  if (nowHr > 23) { nowHr = 0; }                                                                                                                                 // If hr is greater than 23, set to 0
  if (nowHr < 0) { nowHr = 23; }                                                                                                                                 // If hr is less than 0, set to 23
  if (nowMin > 59) { nowMin = 0; }                                                                                                                               // If min is greater than 59, set to 0
  if (nowMin < 0) { nowMin = 59; }                                                                                                                               // If min is less than 0, set to 59
  if (nowSec > 59) { nowSec = 0; }                                                                                                                               // If sec is greater than 59, set to 0
  if (nowSec < 0) { nowSec = 59; }                                                                                                                               // If sec is less than 0, set to 59
}  //Sample Times Correct
//   if (((COMPONENT_YEAR != 28)&&(COMPONENT_YEAR != 32)&&(COMPONENT_YEAR != 36)&&(COMPONENT_YEAR != 40)&&(COMPONENT_YEAR != 44))&&(COMPONENT_MONTH==2)&&(COMPONENT_DAY > 28)){COMPONENT_DAY=1;}// Sets max day in Feb for non leap years up to 2044, and loops to 1 if you exceed
//   if (((COMPONENT_YEAR == 28)||(COMPONENT_YEAR == 32)||(COMPONENT_YEAR == 36)||(COMPONENT_YEAR == 40)||(COMPONENT_YEAR == 44))&&(COMPONENT_MONTH==2)&&(COMPONENT_DAY > 29)){COMPONENT_DAY=1;}// Sets max day in Feb for leap years up to 2044, and loops to 1 if you exceed
//   if (((COMPONENT_MONTH == 4)||(COMPONENT_MONTH == 6)||(COMPONENT_MONTH == 9)||(COMPONENT_MONTH == 11))&&(COMPONENT_DAY > 30)){COMPONENT_DAY=1;}// Sets max days for April, June, September, November, and loops to 1 if you exceed
//   if (((COMPONENT_MONTH == 1)||(COMPONENT_MONTH == 3)||(COMPONENT_MONTH == 5)||(COMPONENT_MONTH == 7)||(COMPONENT_MONTH == 8)||(COMPONENT_MONTH == 10)||(COMPONENT_MONTH == 12))&&(COMPONENT_DAY > 31)){COMPONENT_DAY=1;}// Sets max days for January, March, May, July, August, October, December, and loops to 1 if you exceed
//   if (((COMPONENT_YEAR != 28)&&(COMPONENT_YEAR != 32)&&(COMPONENT_YEAR != 36)&&(COMPONENT_YEAR != 40)&&(COMPONENT_YEAR != 44))&&(COMPONENT_MONTH==2)&&(COMPONENT_DAY < 1)){COMPONENT_DAY=28;}// Sets max day in Feb for non leap years up to 2044, and loops to max if you go lower than 1
//   if (((COMPONENT_YEAR == 28)||(COMPONENT_YEAR == 32)||(COMPONENT_YEAR == 36)||(COMPONENT_YEAR == 40)||(COMPONENT_YEAR == 44))&&(COMPONENT_MONTH==2)&&(COMPONENT_DAY < 1)){COMPONENT_DAY=29;}// Sets max day in Feb for leap years up to 2044, and loops to max if you go lower than 1
//   if (((COMPONENT_MONTH == 4)||(COMPONENT_MONTH == 6)||(COMPONENT_MONTH == 9)||(COMPONENT_MONTH == 11))&&(COMPONENT_DAY < 1)){COMPONENT_DAY=30;}// Sets max days for April, June, September, November, and loops to max if you go lower than 1
//   if (((COMPONENT_MONTH == 1)||(COMPONENT_MONTH == 3)||(COMPONENT_MONTH == 5)||(COMPONENT_MONTH == 7)||(COMPONENT_MONTH == 8)||(COMPONENT_MONTH == 10)||(COMPONENT_MONTH == 12))&&(COMPONENT_DAY < 1)){COMPONENT_DAY=31;}// Sets max days for January, March, May, July, August, October, and loops to max if you go lower than 1
//   if (COMPONENT_YEAR < 0){COMPONENT_YEAR=0;}// Sets minimum year to 0 or 2000
//   if (COMPONENT_MONTH > 12){COMPONENT_MONTH=1;}// If max month (December) is exceeded, loop to January
//   if (COMPONENT_MONTH < 1){COMPONENT_MONTH=12;}// If min month (January) is exceeded, loop to December
//   if (COMPONENT_HOUR>23){COMPONENT_HOUR=0;}// If hr is greater than 23, set to 0
//   if (COMPONENT_HOUR<0){COMPONENT_HOUR=23;}// If hr is less than 0, set to 23
//   if (COMPONENT_MINUTE>59){COMPONENT_MINUTE=0;}// If min is greater than 59, set to 0
//   if (COMPONENT_MINUTE<0){COMPONENT_MINUTE=59;}// If min is less than 0, set to 59
// }

void calcUpdateSecTime() {
  tmElements_t updateNowTm;               // This is a time elements variable named updateNowTm with each of the different elements as below
  updateNowTm.Hour = nowHr;               // Makes the hour of updateNowTm equal to nowHr
  updateNowTm.Minute = nowMin;            // Makes the minute of updateNowTm equal to nowMin
  updateNowTm.Second = nowSec;            // Makes the second of updateNowTm equal to nowSec
  updateNowTm.Day = nowDay;               // Makes the day of updateNowTm equal to nowDay
  updateNowTm.Month = nowMon;             // Makes the month of updateNowTm equal to nowMon
  updateNowTm.Year = nowYr + 30;          // Makes the year of updateNowTm equal to nowYr + 30 (because time elements deal with seconds since 1970 and nowYr is from 2000)
  updateSecTime = makeTime(updateNowTm);  // Compile all time elements into seconds since 1970 and name it updateSecTime
}

//SD Card
void saveSampleTimes() {
  delay(500);
  // Remove existing file if it exists
  if (SD.exists("sampleTimesSd.txt")) {
    SD.remove("sampleTimesSd.txt");
    Serial.println("sampleTimesSd.txt deleted.");
  }
  // Create a new file for writing
  File sampleFile = SD.open("sampleTimesSd.txt", FILE_WRITE);
  if (!sampleFile) {
    Serial.println("Error creating sampleTimesSd.txt for writing");
    return;
  }
  // Write each formatted time to the file
  for (int i = 0; i < maxSamples; i++) {
    int month, day, year, hour, minute;
    // Check the input string format
    Serial.println("Input sample time: " + sampleTimes[i]);
    int parsedCount = sscanf(sampleTimes[i].c_str(), "%d/%d/%d %d:%d", &month, &day, &year, &hour, &minute);
    // Debug: Check how many items were parsed
    Serial.print("Parsed count: ");
    Serial.println(parsedCount);
    Serial.print("Month: ");
    Serial.println(month);
    Serial.print("Day: ");
    Serial.println(day);
    Serial.print("Year: ");
    Serial.println(year);
    Serial.print("Hour: ");
    Serial.println(hour);
    Serial.print("Minute: ");
    Serial.println(minute);
    // Convert year to four-digit format
    if (year < 100) {
      year += 2000;  // Assuming the year is in the range of 2000-2099
    }
    // Get the formatted time for saving
    String formattedTime = formatSampleTimeForSaving(month, day, year, hour, minute);
    // Debug: Print the formatted time before writing to the file
    Serial.println("Formatted time: " + formattedTime);
    // Write the correctly formatted time to the file
    sampleFile.println(formattedTime);
    delay(100);
  }
  sampleFile.close();
  Serial.println("Sample times saved successfully.");
  // Display confirmation
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0, 0);
  display.println(" <SAVED>");
  display.display();
  delay(1000);
}

void readSampleTimes() {
  File dataFile = SD.open("sampleTimesSd.txt");
  if (dataFile) {
    int index = 0;
    while (dataFile.available() && index < maxSamples) {
      String dateTime = dataFile.readStringUntil('\n');  // Read the line from SD card
      dateTime.trim();                                   // Trim any whitespace/newline

      // Split the date-time string by commas
      int month, day, year, hour, minute;
      int commaIndex1 = dateTime.indexOf(',');
      int commaIndex2 = dateTime.indexOf(',', commaIndex1 + 1);
      int commaIndex3 = dateTime.indexOf(',', commaIndex2 + 1);
      int commaIndex4 = dateTime.indexOf(',', commaIndex3 + 1);
      if (commaIndex1 > 0 && commaIndex2 > 0 && commaIndex3 > 0 && commaIndex4 > 0) {
        month = dateTime.substring(0, commaIndex1).toInt();
        day = dateTime.substring(commaIndex1 + 1, commaIndex2).toInt();
        year = dateTime.substring(commaIndex2 + 1, commaIndex3).toInt();
        hour = dateTime.substring(commaIndex3 + 1, commaIndex4).toInt();
        minute = dateTime.substring(commaIndex4 + 1).toInt();
      } else {
        Serial.println("Error parsing dateTime: " + dateTime);
        continue;  // Skip this line if parsing fails
      }
      // Format it as MM/DD/YY HH:MM
      String formattedTime = String(month < 10 ? "0" : "") + String(month) + "/";
      formattedTime += String(day < 10 ? "0" : "") + String(day) + "/";
      formattedTime += String(year).substring(2);  // Get last two digits of the year
      formattedTime += " " + String(hour < 10 ? "0" : "") + String(hour) + ":";
      formattedTime += String(minute < 10 ? "0" : "") + String(minute);
      sampleTimes[index] = formattedTime;  // Store the formatted time
      index++;
    }
    dataFile.close();
  } else {
    Serial.println("Error opening sampleTimesSd.txt");
  }
  for (int i = 0; i < maxSamples; i++) {
    Serial.println(sampleTimes[i]);  // Print each sample time to check the content
  }
}

void saveSampleSettings() {
  delay(500);
  // Remove existing file if it exists
  if (SD.exists("settingsParamSd.txt")) {
    SD.remove("settingsParamSd.txt");
    Serial.println("settingsParamSd.txt deleted.");
  }
  // Create a new file for writing
  File settingsParam = SD.open("settingsParamSd.txt", FILE_WRITE);
  if (!settingsParam) {
    Serial.println("Error creating settingsParamSd.txt for writing");
    return;
  }
  // Create and write parameter string
  String settingsParamString = String(pumpAVolume) + "," + String(pumpBVolume) + "," + String(calibrationAVolume) + "," + String(calibrationBVolume) + "," + String(calibrationACount) + "," + String(calibrationBCount) + "," + String(sampleValve);

  Serial.println("Writing sample settings: " + settingsParamString);
  settingsParam.println(settingsParamString);
  settingsParam.close();
  Serial.println("Sample settings saved successfully.");
  Serial.println("Trying oled...");
  if (oledEnabled) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(0, 0);
    display.println(" <SAVED>");
    display.display();
    delay(1000);
  }
}

void readSampleSettings() {
  File settingsParam = SD.open("settingsParamSd.txt", FILE_READ);
  if (!settingsParam) {
    Serial.println("Error opening settingsParamSd.txt");
    return;
  }
  String settingsParamLine = settingsParam.readStringUntil('\n');
  settingsParamLine.trim();  // Remove whitespace
  settingsParam.close();
  Serial.println("Read sample settings: " + settingsParamLine);
  int values[SETTINGS_PARAM];
  int lastIndex = 0, nextIndex = 0;
  for (int i = 0; i < SETTINGS_PARAM; i++) {
    nextIndex = settingsParamLine.indexOf(',', lastIndex);
    if (nextIndex == -1) nextIndex = settingsParamLine.length();
    values[i] = settingsParamLine.substring(lastIndex, nextIndex).toInt();
    lastIndex = nextIndex + 1;
  }
  pumpAVolume = values[0];
  pumpBVolume = values[1];
  calibrationAVolume = values[2];
  calibrationBVolume = values[3];
  calibrationACount = values[4];
  calibrationBCount = values[5];
  sampleValve = values[6];
}

void logData() {                                 // Called to log  data
  getNow();                                      // Read current time
  nowVoltage();                                  // Read the current voltage
  getPressureTemp();                             // Read the current pressure and temp and depth
  getLight();                                    // Read the current light level
  dataLog = SD.open("dataLog.TXT", FILE_WRITE);  // Open dataLog.TXT file
  delay(100);
  if (!dataLog) { Serial.println("open failed"); }                                                                                                                                                                                                                                                              // If open dataLog.txt file doesn't open, write an error to the serial for debugging
  String dataLogString = String(nowHr) + ":" + String(nowMin) + ":" + String(nowSec) + "," + String(nowDay) + "/" + String(nowMon) + "/" + String(nowYr + 2000) + "," + String(batteryVoltage) + "," + String(nowPressure) + "," + String(nowTemperature) + "," + String(nowDepth) + "," + String(lightLevel);  // Create a data logging string
  dataLog.println(dataLogString);
  delay(50);  // Write the datalog string to the dataLog.txt file
  Serial.print("Data log string is: ");
  Serial.println(dataLogString);  //Troubleshoot
  dataLog.flush();                // Sync the dataLog.txt file
  delay(500);
}

String formatSampleTimeForSaving(int month, int day, int year, int hour, int minute) {
  // Format as "MM,DD,YYYY,HH,MM"
  String formattedTime = String(month) + "," + String(day) + "," + String(year) + "," + String(hour) + "," + String(minute);
  return formattedTime;
}

//OLED
void setupOLED() {
  Serial.println("Trying to turn on OLED...");
  oledEnabled = true;
  // pinMode(18, INPUT_PULLUP); // SDA (Teensy 4.1 default I2C pins)
  // pinMode(19, INPUT_PULLUP); // SCL
  // Initialize I2C with 100 kHz clock speed
  // Wire.setClock(100000); // Set I2C clock speed to 100 kHz
  pinMode(oledPowerPin, OUTPUT);
  digitalWrite(oledPowerPin, HIGH);
  delay(3000);
  // Initialize I2C with custom SDA and SCL pins
  //Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  display.clearDisplay();  // Clear display after initialization
  display.display();       // Show initial text
  Serial.println("OLED should be working...");
}

//Sampling
void enterSamplingMode() {
  Serial.println("Starting OLED from sampling mode.");
  setupOLED();
  Serial.println("Sampling within 10 minutes!");
  display.clearDisplay();              // Clear the display
  display.setTextSize(1);              // Set the text size to 1
  display.setTextColor(WHITE, BLACK);  // Set the color of the text to normal
  display.setCursor(0, 0);
  display.println("Sampling in");
  display.println("less than");
  display.println("10 min!");
  display.display();  // Update the display
  Serial.println("Current Time: " + String(currentTime));
  Serial.println("Stored Wake-Up Time: " + String(storedWakeUpTime));
  while (currentTime < storedWakeUpTime) {
    currentTime = now();  // Update the existing currentTime variable
    delay(1000);          // Adjust the delay as needed (e.g., 100 ms)
  }
  display.clearDisplay(); // Clear the display
  display.setTextSize(1); // Set the text size to 1
  display.setTextColor(WHITE, BLACK); // Set the color of the text to normal
  display.setCursor(0, 0);
  display.println("Time to"); // Display message
  display.println("sample"); // Display message
  display.println("babyyyy!"); // Display message
  display.display(); // Update the display
  closeValve(); //Make sure 3 way valve is closed (resting state for water flow)
  logData(); //Log all sensor data and time
  resetZeroPosition(); // Zero valve to ensure good positioning
  valveSelection = 1; //Set to valve 1 for flow thru
  sampleValvePosition(); //Rotate stepper to waste valve position
  delay(100);
  pumpAVolumeTemp = 100;  //Set to 100mL, maybe change
  messageTemp = "Pre-sample";
  samplePumpA(); //Run pump A to purge system with ambient water
  delay(100);
  valveSelection = sampleValve;  //Move to sample valve
  sampleValvePosition(); //Rotate stepper to sample valve position
  delay(100);
  pumpAVolumeTemp = pumpAVolume;  //Set to pump A volume
  messageTemp = "Sampling";
  samplePumpA(); //Run pump A to sample water
  delay(100);
  openValve(); // Open 3 way for preservative
  delay(100);
  samplePumpB(); // Run preservative pump
  delay(100);
  closeValve(); // Close 3 way valve (back to normal flow)
  valveSelection = 2; //Set to valve 2 for HCl cleaning
  sampleValvePosition(); //Rotate stepper to sample valve position
  delay(100);
  pumpADirection = 1; // Set pump direction to CCW for cleaning
  pumpAVolumeTemp = 7; //Set to 7mL, maybe change
  samplePumpA(); //Run pump A CCW to clean system with HCl
  delay(100);
  logData(); //Log second data point mostly for tracking system use and current draw
  //Best to leave at HCl for duration of cleaning process
  finishSamplingMode();
}



void testSampling(){
  resetZeroPosition(); // Zero valve to ensure good positioning
  valveSelection = 3;  //Move to sample valve
  pumpADirection = 0; // Set pump direction to CW for sampling
  sampleValvePosition(); //Rotate stepper to sample valve position
  delay(100);
  closeValve(); //Make sure 3 way valve is closed (resting state for water flow)
  delay(100);
  pumpAVolumeTemp = pumpAVolume;  //Set to pump A volume
  messageTemp = "Sampling";
  samplePumpA(); //Run pump A to sample water
}

void testPreserve(){
  resetZeroPosition(); // Zero valve to ensure good positioning
  valveSelection = 3;  //Move to sample valve
  sampleValvePosition(); //Rotate stepper to sample valve position
  delay(100);
  openValve(); // Open 3 way for preservative
  delay(100);
  samplePumpB(); // Run preservative pump
}

void testHCL(){
  resetZeroPosition(); // Zero valve to ensure good positioning
  valveSelection = 2; //Set to valve 2 for HCl cleaning
  sampleValvePosition(); //Rotate stepper to sample valve position
  delay(100);
  closeValve(); // Close 3 way valve (back to normal flow)
  delay(100);
  pumpADirection = 1; // Set pump direction to CCW for cleaning
  pumpAVolumeTemp = 7; //Set to 7mL, maybe change
  samplePumpA(); //Run pump A CCW to clean system with HCl
}

void finishSamplingMode() {
  Serial.println("Teensy is going back to sleep.");
  time_t nextWakeUpTime = findNextWakeUpTimeFromSD(); // Get the next wake-up time from the SD card
  EEPROM.put(eepromWakeUpAddress, nextWakeUpTime);
  Serial.println("Next Wake-Up Time updated in EEPROM: " + String(nextWakeUpTime));
  sleepyTime();
}

void returnMainLoop() {
  Serial.println("Teensy is going back to sleep.");
  saveSampleTimes();
  delay(1000);
  time_t nextWakeUpTime = findNextWakeUpTimeFromSD();  // Get the next wake-up time from the SD card
  EEPROM.put(eepromWakeUpAddress, nextWakeUpTime);
  Serial.println("Next Wake-Up Time updated in EEPROM: " + String(nextWakeUpTime));
  // Save reed baseline for persistence
  EEPROM.put(eepromReedBaselineAddress, reedBaseline);
  Serial.println("Reed baseline updated in EEPROM: " + String(reedBaseline));
  sleepyTime();
}

void sleepyTime() {
  Serial.println("Sleepytime.");
  delay(500);                       //give it a sec
  digitalWrite(oledPowerPin, LOW);  // screen off
  oledEnabled = false;
  digitalWrite(timerDonePin, LOW);  // Send done signal to timer
  delay(500);                       // Send signal for a short period
  digitalWrite(timerDonePin, HIGH);
  delay(500);
  Serial.println("Everyone's off");
  digitalWrite(powerOnPin, LOW);  // Power off to avoid mistakenly powering up
}

//Attempt at coding encoder logic
void encoderISR() {
  uint8_t newState = (digitalRead(stepperEncAPin) << 1) | digitalRead(stepperEncBPin);
  uint8_t index = (lastState << 2) | newState;
  encoderCount += quadTable[index];
  lastState = newState;
}

void initReedBaseline() {
    uint8_t stored;
    EEPROM.get(eepromReedBaselineAddress, stored);
    // fallback if EEPROM uninitialized (255)
    Serial.println("Current EEPROM Reed State:");
    Serial.println(stored);
    if (stored != 0 && stored != 1) {
        reedBaseline = digitalRead(reedSwitchPin) ? 1 : 0;
        Serial.println("Current Reed Baseline:");
        Serial.println(reedBaseline);
        EEPROM.put(eepromReedBaselineAddress, reedBaseline);
    } else {
        reedBaseline = stored;
        Serial.println("Changed Reed Baseline:");
        Serial.println(reedBaseline);
    }
    Serial.println("Reed baseline initialized: " + String(reedBaseline));
}
