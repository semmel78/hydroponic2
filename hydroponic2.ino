/*
  Hydroponics Controller

  The Hydroponics Controller is used to schedule required tasks
  for operating a Dutch Bucket hydroponic system.

  The circuit:
    list the components attached to each input
    list the components attached to each output

  Created 01.07.2016
  Stefan Demmer (semmel)

  Modified day month year
  By author's name

  this is a test
  this is a new test


  http://

*/

#define SDCARD 1
//#define LCD    1
#define DALLAS 1
#define ADRUINO_MEGA 1

#include <Time.h>
#include <Wire.h>
#include <DS1307RTC.h>  // a basic DS1307 library that returns time as a time_t
#include <EEPROM.h>
#include <RBD_SerialManager.h>
#include <SPI.h>
#include <SdFat.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Bounce2.h>
#include "hydroponic2.h"



RBD::SerialManager serial_manager;
#ifdef SDCARD
SdFat sd;
SdFile myFile;
char logFileName[15];
byte sdEnabled = true;
#endif

#ifdef LCD
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display (0x3F ???)
const int BACKLIGHT_ON_TIME = 5;    // switch LCD back light on for XXX seconds
byte backlightIsOn = BACKLIGHT_ON_TIME;
#endif

#ifdef DALLAS
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(PIN_ONEWIRE);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
// arrays to hold device addresses28 FF AD B5 64 14 1 35
DeviceAddress  tempCabinet;
DeviceAddress  tempOutside;
DeviceAddress  tempReservoir;
#endif



// Instantiate a Bounce objects for push buttons
Bounce debounceBackLight = Bounce();
Bounce debounceErrorReset = Bounce();
Bounce debounceSdEject = Bounce();

float cabinetTemperatureValue;
float outsideTemperatureValue;
float reservoirTemperatureValue;


// Array to hold information about watering cycles
timer_t irrigationAlarms[MAX_WATER_TIMERS];
uint8_t stateWatering = STATE_IDLE;

unsigned int flowSensorCounter = 0;
unsigned int flowSensorMinPerSecond = 0;

boolean doManualRefill = false;
char msgBuffer[40];
unsigned long timer1s;
byte sdLogIntervall = 0;


void setup() {
  timeStatus_t myTimeStatus;
  // Start serial manager for serial command processing
  serial_manager.start();
  serial_manager.setDelimiter(' ');

#ifdef SDCARD
  if (!sd.begin(PIN_CHIP_SELECT, SPI_HALF_SPEED)) {
    sd.initErrorPrint();
    sdEnabled = false;
  }
  setLogFileName();
#endif

  // sync with RTC
  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  setSyncInterval(120);

  if (timeStatus() != timeSet) {
    setTime(0, 0, 0, 1, 1, 2016);
    delay(20);
    logEvent_P(PSTR("Unable to sync with the RTC."), MSG_ERROR);
    logEvent_P(PSTR("Setting time manually."), MSG_INFO);
  } else {
    logEvent_P(PSTR("RTC has set the system time."), MSG_INFO);
  }

  // Configure pins
  pinMode(PIN_IRRIGATION, OUTPUT);
  pinMode(PIN_NUTRIENT_1, OUTPUT);
  pinMode(PIN_NUTRIENT_2, OUTPUT);
  pinMode(PIN_REFILL_VALVE, OUTPUT);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  digitalWrite(PIN_IRRIGATION, LOW);
  digitalWrite(PIN_NUTRIENT_1, LOW);
  digitalWrite(PIN_NUTRIENT_2, LOW);
  digitalWrite(PIN_REFILL_VALVE, LOW);

  pinMode(PIN_HW_RESET, OUTPUT);
  digitalWrite(PIN_HW_RESET, HIGH);

  pinMode(PIN_RESERVOIR_EMPTY, INPUT_PULLUP);
  pinMode(PIN_RESERVOIR_VERY_EMPTY, INPUT_PULLUP);
  pinMode(PIN_RESERVOIR_FULL, INPUT_PULLUP);
  pinMode(PIN_RESERVOIR_VERY_FULL, INPUT_PULLUP);

  // Cunfigure inputs for various push buttons
  pinMode(PIN_BACKLIGHT, INPUT_PULLUP);
  pinMode(PIN_ERRORRESET, INPUT_PULLUP);
  pinMode(PIN_SDEJECT, INPUT_PULLUP);

  // After setting up the button, setup the Bounce instance :
  debounceBackLight.attach(PIN_BACKLIGHT);
  debounceBackLight.interval(10); // interval in ms
  debounceErrorReset.attach(PIN_ERRORRESET);
  debounceErrorReset.interval(10); // interval in ms
  debounceSdEject.attach(PIN_SDEJECT);
  debounceSdEject.interval(10); // interval in ms

  // Configure input for water flow sensor and attache interrupt
  pinMode(PIN_REFILL_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_REFILL_SENSOR), interruptRoutineFlowSensorCounter, RISING);
  logEvent_P(PSTR("Setting pin modes done."), MSG_INFO);

  // read configuration from EEPROM
  sdLogIntervall = EEPROM.read(EEPROM_LOG_INTERVAL);
  if (sdLogIntervall < 1 or sdLogIntervall > 30) {
    sdLogIntervall = 5;
  }
  flowSensorMinPerSecond = EEPROMReadInt(EEPROM_FLOW_SENSOR_MIN_PER_SECOND);
  if (flowSensorMinPerSecond < 1 or flowSensorMinPerSecond > 65000) {
    flowSensorMinPerSecond = 20;
  }


  readOwArdessesFromEeprom(tempCabinet, EEPROM_TEMP_CABINET);
  readOwArdessesFromEeprom(tempOutside, EEPROM_TEMP_OUTSIDE);
  readOwArdessesFromEeprom(tempReservoir, EEPROM_TEMP_RESERVOIR);

#ifdef DALLAS
  // Start up the library for the Temperature Sensors
  sensors.begin();
  sensors.setWaitForConversion(false);

  // locate devices on the bus
  sprintf_P(msgBuffer, PSTR("Temperature sensors found: %d."), sensors.getDeviceCount());
  logEvent(msgBuffer, MSG_INFO);

  sprintf_P(msgBuffer, PSTR("Parasite power: %d."), sensors.isParasitePowerMode());
  logEvent(msgBuffer, MSG_INFO);

  if (sensors.isConnected(tempCabinet)) {
    logEvent_P(PSTR("OneWire Cabinet Sensor connected."), MSG_INFO);
    sensors.setResolution(tempCabinet, TEMPERATURE_PRECISION);
  }
  else {
    logEvent_P(PSTR("OneWire Cabinet Sensor not found."), MSG_ERROR);
  }

  if (sensors.isConnected(tempOutside)) {
    logEvent_P(PSTR("OneWire Outside Sensor connected."), MSG_INFO);
    sensors.setResolution(tempOutside, TEMPERATURE_PRECISION);
  }
  else {
    logEvent_P(PSTR("OneWire Outside Sensor not found."), MSG_ERROR);
  }

  if (sensors.isConnected(tempReservoir)) {
    logEvent_P(PSTR("OneWire Reservoir Sensor connected."), MSG_INFO);
    sensors.setResolution(tempReservoir, TEMPERATURE_PRECISION);
  }
  else {
    logEvent_P(PSTR("OneWire Reservoir Sensor not found."), MSG_ERROR);
  }

  sensors.requestTemperatures();

  logEvent_P(PSTR("OneWire initialised."), MSG_INFO);
#endif

#ifdef LCD
  lcd.init();                      // initialize the lcd
  lcd.clear();
  logEvent_P(PSTR("LCD initialised."), MSG_INFO);
#endif

  readWaterTimers();
  logEvent_P(PSTR("Setup finished."), MSG_INFO);
}

void loop() {
  static float refilledWater = 0;
  static int fertilizerPumpRunTime1;
  static int fertilizerPumpRunTime2;
  static unsigned long fertilizerPumpStartTime;
  static unsigned int nowHM = 0;
  static unsigned int nextActionTime = 0;
  static unsigned long flowSensorTime = 0;
  static byte flowSensorErrorCounter = 0;
  static unsigned long lcdUpdateTime;
  static byte lastMinute;

  // Serial Command Interface
  CheckSerialManager();


  // blink pin 13
  digitalWrite(13, !digitalRead(13));

  // Get current time with 1 minute resolution
  nowHM = hour() * 60 + minute();

  switch (stateWatering) {
    case STATE_IDLE:
      for (int i = 0; i < MAX_WATER_TIMERS; i++) {
        if (nowHM >= irrigationAlarms[i].startTime && nowHM <= irrigationAlarms[i].endTime && irrigationAlarms[i].duration > 0) { // irrigation time slot
          digitalWrite(PIN_IRRIGATION, HIGH);
          nextActionTime = irrigationAlarms[i].endTime;
          stateWatering = STATE_IRRIGATE;

          logEvent_P(PSTR("Irrigation pump ON."), MSG_INFO);
        }
      }
      // if reservoir is empty or very empty, directly jump to refill state
      if (digitalRead(PIN_RESERVOIR_EMPTY) == HIGH || digitalRead(PIN_RESERVOIR_VERY_EMPTY) == HIGH || doManualRefill) {
        digitalWrite(PIN_REFILL_VALVE, HIGH);
        if (doManualRefill) {
          doManualRefill = false;
          //logEvent_P(PSTR("Manual refill triggered! Refill valve ON."), MSG_WARNING);
          logEvent_P(PSTR("Man ref ON."), MSG_WARNING);
        }
        else {
          logEvent_P(PSTR("Reservoir empty! Refill valve ON."), MSG_INFO);
        }
        nextActionTime = nowHM + MAX_REFILL_TIME; // maximum time for refilling!!!
        stateWatering = STATE_REFILL;
        flowSensorCounter = 0;      //Set flowSensorCounter to 0 ready for calculations
        flowSensorTime = millis();
        interrupts();            //Enables interrupts
      }
      break;
    case STATE_IRRIGATE:
      if (nowHM >= nextActionTime || digitalRead(PIN_RESERVOIR_VERY_EMPTY) == HIGH) { // Switch off after time or if reservoir very empty
        digitalWrite(PIN_IRRIGATION, LOW); // Switch pump off
        if (digitalRead(PIN_RESERVOIR_VERY_EMPTY) == HIGH) {
          logEvent_P(PSTR("Irrigation pump OFF. Reservoir very empty."), MSG_WARNING);
        }
        else {
          logEvent_P(PSTR("Irrigation pump OFF."), MSG_INFO);
        }
        nextActionTime = nowHM + (int)EEPROM.read(EEPROM_DRAIN_TIME);
        stateWatering = STATE_DRAIN;
      }
      break;
    case STATE_DRAIN:
      if (nowHM >= nextActionTime) { // after drain time is over
        if (digitalRead(PIN_RESERVOIR_FULL) == HIGH || digitalRead(PIN_RESERVOIR_VERY_FULL) == HIGH) { // if reservoir is still full, no refill required
          logEvent_P(PSTR("No refill required."), MSG_INFO);
          stateWatering = STATE_IDLE;
        }
        else {
          digitalWrite(PIN_REFILL_VALVE, HIGH);
          logEvent_P(PSTR("Refill valve ON."), MSG_INFO);

          nextActionTime = nowHM + MAX_REFILL_TIME; // maximum time for refilling!!!
          flowSensorCounter = 0;                   // Set flowSensorCounter to 0 ready for calculations
          flowSensorTime = millis();
          interrupts();                                    // Enables interrupts
          stateWatering = STATE_REFILL;
        }
      }
      break;
    case STATE_REFILL:
      flowSensorCounter = random(25, 100);
      if (nowHM >= nextActionTime) {
        digitalWrite(PIN_REFILL_VALVE, LOW);
        logEvent_P(PSTR("Refill took too long."), MSG_ERROR);

        stateWatering = STATE_ERROR;
        // it seems that water flow is too low or not existing (has someone turned off the main valve?)
      }
      if (digitalRead(PIN_RESERVOIR_FULL) == HIGH || digitalRead(PIN_RESERVOIR_VERY_FULL) == HIGH) {
        digitalWrite(PIN_REFILL_VALVE, LOW); // stop valve
        logEvent_P(PSTR("Refill valve OFF."), MSG_INFO);

        // --> evaluate amount of water filled to reservoir
        noInterrupts(); // Deactivate interrupt
        refilledWater = refilledWater + (float)flowSensorCounter * ML_PER_TICK;  // calculate final amount of water which was refilled to reservoir
        sprintf_P(msgBuffer, PSTR("%dml refilled."), (int)(refilledWater + .5));
        logEvent(msgBuffer, MSG_INFO);

        fertilizerPumpRunTime1 = (int)(EEPROM.read(EEPROM_NUTRIENT_TIMER)     * refilledWater / 100 * EEPROM.read(EEPROM_NUTRIENT_CALIBRATION));
        fertilizerPumpRunTime2 = (int)(EEPROM.read(EEPROM_NUTRIENT_TIMER + 1) * refilledWater / 100 * EEPROM.read(EEPROM_NUTRIENT_CALIBRATION));
        fertilizerPumpStartTime = millis();
        digitalWrite(PIN_NUTRIENT_1, HIGH);
        digitalWrite(PIN_NUTRIENT_2, HIGH);
        logEvent_P(PSTR("Nutrient pumps 1 and 2 ON."), MSG_INFO);
        sprintf_P(msgBuffer, PSTR("Nutrient 1: %d ms."), fertilizerPumpRunTime1);
        logEvent(msgBuffer, MSG_INFO);
        sprintf_P(msgBuffer, PSTR("Nutrient 2: %d ms."), fertilizerPumpRunTime2);
        logEvent(msgBuffer, MSG_INFO);

        stateWatering = STATE_FERTILIZE;
      }
      if (millis() - flowSensorTime > 1000) {
        noInterrupts();
        refilledWater = refilledWater + (float)flowSensorCounter * ML_PER_TICK;  // calculate intermediate amount of water which was refilled to reservoir
        sprintf_P(msgBuffer, PSTR("Act: %d  - Cum: %d"), flowSensorCounter, (int)(refilledWater + .5));
        logEvent(msgBuffer, MSG_INFO);
        if (flowSensorCounter < 20) { // No water flow since 1 second
          flowSensorErrorCounter++;   // Try some times
          logEvent_P(PSTR("No flow."), MSG_ERROR);
        }
        flowSensorCounter = 0; // Reset flowSensorCounter
        flowSensorTime = millis();
        interrupts(); // Enable interrupt again
      }
      if (flowSensorErrorCounter > 5) { // Refill faild goto ERROR
        flowSensorErrorCounter = 0;
        flowSensorCounter = 0;
        logEvent_P(PSTR("No water flow detected during refill."), MSG_ERROR);
        stateWatering = STATE_ERROR;
        break;
      }
      break;
    case STATE_FERTILIZE:
      if (millis() - fertilizerPumpStartTime >= fertilizerPumpRunTime1 && fertilizerPumpRunTime1 > 0) {
        // stop pump 1
        digitalWrite(PIN_NUTRIENT_1, LOW);
        logEvent_P(PSTR("Nutrient pump 1 OFF."), MSG_INFO);
        fertilizerPumpRunTime1 = 0;
      }
      if (millis() - fertilizerPumpStartTime >= fertilizerPumpRunTime2 && fertilizerPumpRunTime2 > 0) {
        // stop pump 2
        digitalWrite(PIN_NUTRIENT_2, LOW);
        logEvent_P(PSTR("Nutrient pump 2 OFF."), MSG_INFO);
        fertilizerPumpRunTime2 = 0;
      }
      if (millis() - fertilizerPumpStartTime >= fertilizerPumpRunTime1 && millis() - fertilizerPumpStartTime >= fertilizerPumpRunTime2) {
        refilledWater = 0;

        logEvent_P(PSTR("Refill and adding nutrients finished."), MSG_INFO);
        stateWatering = STATE_IDLE;
      }
      break;
    case STATE_ERROR:
      //send sms
      logEvent_P(PSTR("I am in state ERROR."), MSG_ERROR);
      break;
    default:
      // something went wrong
      logEvent_P(PSTR("State machine in undefined state! Resetting to idle mode."), MSG_WARNING);
      stateWatering = STATE_IDLE;
      break;
  }

  // execute every second
  if (millis() - timer1s >= 1000) {
    timer1s = millis();

    cabinetTemperatureValue = sensors.getTempC(tempCabinet);
    outsideTemperatureValue = sensors.getTempC(tempOutside);
    reservoirTemperatureValue = sensors.getTempC(tempReservoir);
    sensors.requestTemperatures();
#ifdef LCD
    // Switch LCD backlight off after BACKLIGHT_ON_TIME seconds
    if (backlightIsOn > 0) {
      backlightIsOn--;
    }
    else {
      lcd.noBacklight();
    }
#endif
  }

  // Execute every RTC minute
  if (minute() != lastMinute && second() == 0) {
    lastMinute = minute();

    // Update log file name
    if (hour() == 0 && minute() == 0 && second() == 0) {
      logEvent_P(PSTR("Day over, creating new log file."), MSG_INFO);
      setLogFileName();
    }

    if (minute() % sdLogIntervall == 0) {
      // do sensor logging
      msgBuffer[0] = '\0'; // Nullstring setzen

      itoa(digitalRead(PIN_RESERVOIR_VERY_FULL), &msgBuffer[strlen(msgBuffer)], 10);
      strcat(msgBuffer, "\t");
      itoa(digitalRead(PIN_RESERVOIR_FULL), &msgBuffer[strlen(msgBuffer)], 10);
      strcat(msgBuffer, "\t");
      itoa(digitalRead(PIN_RESERVOIR_EMPTY), &msgBuffer[strlen(msgBuffer)], 10);
      strcat(msgBuffer, "\t");
      itoa(digitalRead(PIN_RESERVOIR_VERY_EMPTY), &msgBuffer[strlen(msgBuffer)], 10);
      strcat(msgBuffer, "\t");
      itoa(1, &msgBuffer[strlen(msgBuffer)], 10);
      strcat(msgBuffer, "\t");
      itoa(1, &msgBuffer[strlen(msgBuffer)], 10);
      strcat(msgBuffer, "\t");
      dtostrf(cabinetTemperatureValue, 0, 1, &msgBuffer[strlen(msgBuffer)]);
      strcat(msgBuffer, "\t");
      dtostrf(outsideTemperatureValue, 0, 1, &msgBuffer[strlen(msgBuffer)]);
      strcat(msgBuffer, "\t");
      dtostrf(reservoirTemperatureValue, 0, 1, &msgBuffer[strlen(msgBuffer)]);
      strcat(msgBuffer, "\t");
      strcat(msgBuffer, "600");
      strcat(msgBuffer, "\t");
      dtostrf(6.4F, 0, 1, &msgBuffer[strlen(msgBuffer)]);
      logEvent(msgBuffer, MSG_SENSOR);
    }
  }

#ifdef LCD
  //if button pressed, activate backlight for BACKLIGHT_ON_TIME s
  if (digitalRead(PIN_BACKLIGHT) == LOW) {
    backlightIsOn = BACKLIGHT_ON_TIME;
    lcd.backlight();
  }
#endif

  // every 200ms update LCD
  if (millis() - lcdUpdateTime >= 200) {
    //lcd.setCursor(x,y);
    // 01:01:01  01.01.2016
    // STATE   xxxxxxxxxxxx
    // 20,5°C 20,5°C 20,5°C
    // 1  1  1  1

    //sprintf(lcdText.row1, "%02d:%02d:%02d  %02d.%02d.%04d", hour(), minute(), second(), day(), month(), year());
  }
}

void setLogFileName() {
#ifdef SDCARD
  sprintf(logFileName, "%04d%02d%02d.log", year(), month(), day());
#endif
}

int nowHM() {
  return hour() * 60 + minute();
}

void readWaterTimers() {
  // Read watering timers from EEPROM
  for (int i = 0; i < MAX_WATER_TIMERS; i++) {
    byte myHour     =  EEPROM.read(EEPROM_WATER_TIMER + i * 3 );
    byte myMinute   =  EEPROM.read(EEPROM_WATER_TIMER + i * 3 + 1);
    byte myDuration =  EEPROM.read(EEPROM_WATER_TIMER + i * 3 + 2);
    if (myHour >= 0 && myHour <= 23 &&
        myMinute >= 0 && myMinute <= 59 &&
        myDuration > 0 && myDuration <= 255) {
      irrigationAlarms[i].startTime = myHour * 60 + myMinute;
      irrigationAlarms[i].endTime = myHour * 60 + myMinute + myDuration;
      irrigationAlarms[i].duration = myDuration;
    }
    else {
      irrigationAlarms[i].startTime = 99 * 60 + 99;
      irrigationAlarms[i].endTime = 99 * 60 + 99;
      irrigationAlarms[i].duration = 0;
    }
    irrigationAlarms[i].eepromStorageId = (byte)i;
  }
  sortWaterTimers(); // sort the array of irrigation alarms in RAM
}


void CheckSerialManager() {
  if (serial_manager.onReceive()) {
    if (serial_manager.isCmd("help") || serial_manager.isCmd("HELP")) {
      if (serial_manager.getParam().length() > 0) {
        String value = serial_manager.getParam();
        Serial.println("");
        if (value == "cat" || value == "CAT") {
          printHelpCat();
          return;
        }
        if (value == "cfgflow" || value == "CFGFLOW") {
          printHelpCfgflow();
          return;
        }
        if (value == "cfgtmp" || value == "CFGTMP") {
          printHelpCfgtmp();
          return;
        }
        if (value == "date" || value == "DATE") {
          printHelpDate();
          return;
        }
        if (value == "datetime" || value == "DATETIME") {
          printHelpDatetime();
          return;
        }
        if (value == "dir" || value == "DIR") {
          Serial.println(F("List files on storage media."));
          return;
        }
        if (value == "drain" || value == "DRAIN") {
          printHelpDrain();
          return;
        }
        if (value == "hwrst" || value == "HWRST") {
          Serial.println(F("Do a real HW reset. Use carefully!"));
          return;
        }
        if (value == "info" || value == "INFO") {
          Serial.println(F("Show current status of controller."));
          return;
        }
        if (value == "list" || value == "LIST") {
          Serial.println(F("List content of EEPROM."));
          return;
        }
        if (value == "logint" || value == "LOGINT") {
          printHelpLoginterval();
          return;
        }
        if (value == "ls" || value == "LS") {
          Serial.println(F("List files on storage media."));
          return;
        }
        if (value == "now" || value == "NOW") {
          Serial.println(F("Print current date and time."));
          return;
        }
        if (value == "nutcal" || value == "NUTCAL") {
          printHelpNutcal();
          return;
        }
        if (value == "nutrient" || value == "NUTRIENT") {
          printHelpNutrient();
          return;
        }
        if (value == "refill" || value == "REFILL") {
          Serial.println(F("Trigger manual refill of reservoir."));
          return;
        }
        if (value == "reset" || value == "RESET") {
          Serial.println(F("Set all parameters to default values."));
          return;
        }
        if (value == "sdeject" || value == "SDEJECT") {
          Serial.println(F("Disable SD card for save removal."));
          return;
        }
        if (value == "sdinit" || value == "SDINIT") {
          Serial.println(F("Re-initialise SD card."));
          return;
        }
        if (value == "time" || value == "TIME") {
          printHelpTime();
          return;
        }
        if (value == "water" || value == "WATER") {
          printHelpWater();
          return;
        }
      }
      else {
        printHelp();
        return;
      }
    }

    // *** TIME **********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("time") || serial_manager.isCmd("TIME")) {
      if (serial_manager.getParam().length() >= 8) {
        String value = serial_manager.getParam();
        setTime(value.substring(0, 2).toInt(), value.substring(3, 5).toInt(), value.substring(6, 8).toInt(), day(), month(), year());
        RTC.set(now());
        // Log Event to SD
        serial_manager.println(F("OK"));
        printDateTime();
        logEvent_P(PSTR("Time set manually."), MSG_INFO);
        value = "";
      }
      else {
        // Print help
        printHelpTime();
        printDateTime();
      }
    }

    // *** NOW ***********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("now") || serial_manager.isCmd("NOW")) {
      printDateTime();
    }

    // *** DATE **********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("date") || serial_manager.isCmd("DATE")) {
      if (serial_manager.getParam().length() >= 10) {
        String value = serial_manager.getParam();
        setTime(hour(), minute(), second(), value.substring(0, 2).toInt(), value.substring(3, 5).toInt(), value.substring(6, 10).toInt());
        RTC.set(now());
        // Log Event to SD
        Serial.println(F("OK"));
        logEvent_P(PSTR("Date set manually."), MSG_INFO);
        printDateTime();
      }
      else {
        // Print help
        printHelpDate();
        printDateTime();
      }
    }

    // *** DATETIME ******************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("datetime") || serial_manager.isCmd("DATETIME")) {
      if (serial_manager.getParam().length() >= 19) {
        String value = serial_manager.getParam();
        setTime(value.substring(11, 13).toInt(), value.substring(14, 16).toInt(), value.substring(17, 19).toInt(), value.substring(0, 2).toInt(), value.substring(3, 5).toInt(), value.substring(6, 10).toInt());
        RTC.set(now());
        // Log Event to SD
        Serial.println(F("OK"));
        printDateTime();
        logEvent_P(PSTR("Date and time set manually."), MSG_INFO);
      }
      else {
        // Print help
        printHelpDatetime();
        printDateTime();
      }
    }

    // *** WATER *********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("water") || serial_manager.isCmd("WATER")) {
      if (serial_manager.getParam().length() > 0) {
        int channel = -1;
        int myHour = -1;
        int myMinute = -1;
        int myDuration = -1;
        boolean valuesChanged = false;

        String value = serial_manager.getParam();
        // water <0...8> <hh>:<mm> <duration in minutes max. 255>
        if (value.length() > 8) {
          channel = value.substring(0, 1).toInt();
          myHour = value.substring(2, 4).toInt();
          myMinute = value.substring(5, 7).toInt();
          myDuration = value.substring(8, value.length()).toInt();
        }
        // water <0...8> del
        else if (value.length() >= 3) {
          channel = value.substring(0, 1).toInt();
          if (value.substring(2, value.length()) == "del") {
            myHour = 0;
            myMinute = 0;
            myDuration = 0;
          }
        }

        // Validate and correct input if required
        if (myHour < 0 || myHour > 23) {
          myHour = 0;
        }
        if (myMinute < 0 || myMinute > 59) {
          myMinute = 0;
        }
        if (myDuration < 0 || myDuration > 255) {
          myDuration = 0;
        }
        if (channel >= 0 || channel < MAX_WATER_TIMERS) {
          if (myHour != EEPROM.read(irrigationAlarms[channel].eepromStorageId * 3)) {
            EEPROM.write(EEPROM_WATER_TIMER + irrigationAlarms[channel].eepromStorageId * 3, myHour);       // Hour
            valuesChanged = true;
          }
          if (myMinute != EEPROM.read(irrigationAlarms[channel].eepromStorageId * 3 + 1)) {
            EEPROM.write(EEPROM_WATER_TIMER + irrigationAlarms[channel].eepromStorageId * 3 + 1, myMinute); // Minute
            valuesChanged = true;
          }
          if (myDuration != EEPROM.read(irrigationAlarms[channel].eepromStorageId * 3 + 2)) {
            EEPROM.write(EEPROM_WATER_TIMER + irrigationAlarms[channel].eepromStorageId * 3 + 2, myDuration); // Duration
            valuesChanged = true;
          }
        }
        if (valuesChanged)  {
          // reread from EEPROM to array in RAM
          readWaterTimers();

          // Log Event
          logEvent_P(PSTR("Water Timer changed."), MSG_INFO);
          logParameters(LOG_WATERTIMERS);
        }
      }
      else {
        // Print help
        printHelpWater();
        logParameters(LOG_WATERTIMERS, SERIAL_ONLY);
      }
    }

    // *** ERASE *********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("erase")) {
      for (int i = 0; i < EEPROM.length(); i++) {
        EEPROM.update(i, 0xFF);
      }
      serial_manager.println("OK");
    }

    // *** LOGINT *********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("logint") || serial_manager.isCmd("LOGINT")) {
      if (serial_manager.getParam().length() > 0) {
        int value = serial_manager.getParam().toInt();
        if (value < 1 || value > 30) {
          value = 5;
        }
        EEPROM.update(EEPROM_LOG_INTERVAL, (byte)value);
        sdLogIntervall = (byte)value;
        logEvent_P(PSTR("New log interval set."), MSG_INFO);
        logParameters(LOG_LOGINTERVAL);
      }

      else {
        // Print help
        printHelpLoginterval();
        logParameters(LOG_LOGINTERVAL, SERIAL_ONLY);
      }
    }

#ifdef SDCARD
    // *** CAT *********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("cat") || serial_manager.isCmd("CAT")) {
      if (serial_manager.getParam().length() > 0) {
        char buf[14];
        serial_manager.getParam().toCharArray(buf, 13);
        //SdFile myOutputFile;
        //file open
        if (!myFile.open(buf, O_READ)) {
          sprintf_P(msgBuffer, PSTR("Failed to open file %s"), buf);
          logEvent(msgBuffer, MSG_WARNING, SERIAL_ONLY);
          return;
        }
        else {
          // read from the file until there's nothing else in it:
          int data;
          sprintf_P(msgBuffer, PSTR("********** Reading file: %s **********"), buf);
          Serial.println(msgBuffer);
          while ((data = myFile.read()) >= 0) {
            Serial.write(data);
          }
          // close the file:
          myFile.close();
          Serial.println(F("********** end file **********"));
        }
      }
      else {
        // Print help
        printHelpCat();
      }
    }

    // *** DIR *********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("ls") || serial_manager.isCmd("dir") || serial_manager.isCmd("LS") || serial_manager.isCmd("DIR")) {
      Serial.println(F("List of files on the SD."));
      sd.ls(LS_R);
    }

    // *** SDEJECT **********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("sdeject") || serial_manager.isCmd("SDEJECT")) {
      sdEnabled = false;
      logEvent_P(PSTR("SD card can now be removed safely."), MSG_WARNING);
      // light LED
    }

    // *** SDINIT **********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("sdinit") || serial_manager.isCmd("SDINIT")) {
      if (!sd.begin(PIN_CHIP_SELECT, SPI_HALF_SPEED)) {
        sd.initErrorPrint();
      }
      else {
        sdEnabled = true;
        setLogFileName();
        logEvent_P(PSTR("SD card has been re-initialised."), MSG_INFO);
        //switch off led
      }
    }
#endif

    // *** LIST **********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("list")) {
      EEPROMSerialDumpTable(10);
    }

    // *** REFILL **********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("refill")) {
      doManualRefill = true;
      logEvent_P(PSTR("Manual refill triggered."), MSG_WARNING);
    }

    // *** DRAIN **********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("drain") || serial_manager.isCmd("DRAIN")) {
      if (serial_manager.getParam().length() > 0) {
        byte draintime = 0;
        String value = serial_manager.getParam();
        draintime = (byte)value.substring(0, value.length()).toInt();
        EEPROM.update(EEPROM_DRAIN_TIME, draintime);
        logEvent_P(PSTR("New drain time set."), MSG_INFO);
      }
      else {
        // Print help
        printHelpDrain();
        logParameters(LOG_DRAINTIME, SERIAL_ONLY);
      }
    }

    // *** CFGFLOW **********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("cfgflow") || serial_manager.isCmd("CFGFLOW")) {
      if (serial_manager.getParam().length() > 0) {
        unsigned int minFlowRate = 0;
        String value = serial_manager.getParam();
        minFlowRate = value.substring(0, value.length()).toInt();
        EEPROMWriteInt(EEPROM_FLOW_SENSOR_MIN_PER_SECOND, minFlowRate);
        logEvent_P(PSTR("Configuration of flow sensor alarm saved."), MSG_INFO);
      }
      else {
        // Print help
        printHelpCfgflow();
        logParameters(LOG_CFGFLOW, SERIAL_ONLY);
      }
    }

    // *** INFO **********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("info") || serial_manager.isCmd("INFO")) {
      Serial.println(F("--------- status begin ---------"));
      logParameters(LOG_ALL, SERIAL_ONLY);
      Serial.println(F("---------- status end ----------"));
    }

    // *** HWRST **********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("hwrst") || serial_manager.isCmd("HWRST")) {
      logEvent_P(PSTR("Manual HW RESET performed."), MSG_WARNING);
      delay(200);
      digitalWrite(PIN_HW_RESET, LOW);
    }

    // *** RESET **********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("reset") || serial_manager.isCmd("RESET")) {
      logEvent_P(PSTR("Resetting controller to default values!"), MSG_WARNING);

      char myChar;
      for (int i = 0; i < 50; i++)
      {
        myChar =  pgm_read_byte_near(EEPROMBackup + i);
        EEPROM.update(i, myChar);
      }
      logEvent_P(PSTR("Automatic HW RESET performed."), MSG_WARNING);
      delay(200);
      digitalWrite(PIN_HW_RESET, LOW);
    }

    // *** NUTRIENT **********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("nutrient") || serial_manager.isCmd("NUTRIENT")) {
      if (serial_manager.getParam().length() > 0) {
        int channel = -1;
        int myDuration = -1;

        String value = serial_manager.getParam();
        channel = value.substring(0, 1).toInt();
        myDuration = value.substring(2, value.length()).toInt();
        if (channel >= 0 && channel <= 1 &&
            myDuration >= 0 && myDuration <= 255) {
          EEPROM.update(EEPROM_NUTRIENT_TIMER + channel, myDuration);
          logParameters(LOG_NUTRIENTS);
        }
      }
      else {
        // Print help
        printHelpNutrient();
        logParameters(LOG_NUTRIENTS, SERIAL_ONLY);
      }
    }

    // *** NUTCAL **********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("nutcal") || serial_manager.isCmd("NUTCAL")) {
      if (serial_manager.getParam().length() > 0) {
        int channel = -1;
        int myValue = -1;

        String value = serial_manager.getParam();
        channel = value.substring(0, 1).toInt();
        myValue = value.substring(2, value.length()).toInt();
        if (channel >= 0 && channel <= 1 &&
            myValue >= 0 && myValue <= 255) {
          EEPROM.update(EEPROM_NUTRIENT_CALIBRATION + channel, myValue);
          logParameters(LOG_NUTRIENTS);
        }
      }
      else {
        // Print help
        printHelpNutcal();
        logParameters(LOG_NUTRIENTS, SERIAL_ONLY);
      }
    }

    // *** CFGTMP **********************************************************************************************************************************************************************************************
    if (serial_manager.isCmd("cfgtmp") || serial_manager.isCmd("CFGTMP")) {
      if (serial_manager.getParam().length() > 0) {
        int channel = -1;
        char buf[50];
        serial_manager.getParam().toCharArray(buf, 50);
        channel = (char)buf[0] - '0'; //value.substring(0, 1).toInt();

        switch (channel) {
          case 0:
            for (int i = 0; i < 8; i++) {
              int n;
              n = (hexToByte(buf[2 + i * 3]) * 16) + hexToByte(buf[2 + i * 3 + 1]);
              EEPROM.update(EEPROM_TEMP_CABINET + i, n);
              tempCabinet[i] = n;
            }
            logEvent_P(PSTR("OW Sensor adress changed for sensor CABINET."), MSG_INFO);
            break;
          case 1:
            for (int i = 0; i < 8; i++) {
              int n;
              n = (hexToByte(buf[2 + i * 3]) * 16) + hexToByte(buf[2 + i * 3 + 1]);
              EEPROM.update(EEPROM_TEMP_OUTSIDE + i, n);
              tempOutside[i] = n;
            }
            logEvent_P(PSTR("OW Sensor adress changed for sensor OUTSIDE."), MSG_INFO);
            break;
          case 2:
            for (int i = 0; i < 8; i++) {
              int n;
              n = (hexToByte(buf[2 + i * 3]) * 16) + hexToByte(buf[2 + i * 3 + 1]);
              EEPROM.update(EEPROM_TEMP_RESERVOIR + i, n);
              tempReservoir[i] = n;
            }
            logEvent_P(PSTR("OW Sensor adress changed for sensor RESERVOIR."), MSG_INFO);
            break;
          default:
            break;
        }
      } else {
        printHelpCfgtmp();
        // print configured sensor adresses + status
        Serial.print(F("Sensor Cabinet   (ID: 0): "));
        printOwAddress(tempCabinet);
        if (sensors.isConnected(tempCabinet)) {
          Serial.print(F(" OK"));
        } else {
          Serial.print(F(" NOT CONNECTED"));
        }
        Serial.println("");

        Serial.print(F("Sensor Outside   (ID: 1): "));
        printOwAddress(tempOutside);
        if (sensors.isConnected(tempOutside)) {
          Serial.print(F(" OK"));
        } else {
          Serial.print(F(" NOT CONNECTED"));
        }
        Serial.println("");

        Serial.print(F("Sensor Reservoir (ID: 2): "));
        printOwAddress(tempReservoir);
        if (sensors.isConnected(tempReservoir)) {
          Serial.print(F(" OK"));
        } else {
          Serial.print(F(" NOT CONNECTED"));
        }
        Serial.println("");
        Serial.println("");
        // search for sensors

        uint8_t address[8];
        uint8_t count = 0;

        if (oneWire.search(address)) {
          if (memcmp(address, tempCabinet, 8) != 0 && memcmp(address, tempOutside, 8) != 0 && memcmp(address, tempReservoir, 8) != 0) { // found sensor has different address than sensors in use
            count++;
            Serial.print("New Sensor              : ");
            printOwAddress(address);
          }
          while (oneWire.search(address));
          if (count == 0) {
            Serial.println(F("NO new temperature sensors found!"));
          } else {
            Serial.print(count, DEC);
            Serial.println(F(" new temperature sensors found!"));
          }
        }
      }
    }
  }
}

byte hexToByte (char c) {
  if ( (c >= '0') && (c <= '9') ) {
    return c - '0';
  }
  if ( (c >= 'A') && (c <= 'F') ) {
    return (c - 'A') + 10;
  }
}

void printOwAddress(uint8_t* address) {
  for (uint8_t i = 0; i < 8; i++)
  {
    if (address[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(address[i], HEX);
    if (i < 7) {
      Serial.print(" ");
    }
  }
}


void printHelp() {
  Serial.println(F("    __  __          __                             _"));
  Serial.println(F("   / / / /_  ______/ /________  ____  ____  ____  (_)_________"));
  Serial.println(F("  / /_/ / / / / __  / ___/ __ \\/ __ \\/ __ \\/ __ \\/ / ___/ ___/"));
  Serial.println(F(" / __  / /_/ / /_/ / /  / /_/ / /_/ / /_/ / / / / / /__(__  ) "));
  Serial.println(F("/_/ /_/\\__, /\\__,_/_/   \\____/ .___/\\____/_/ /_/_/\\___/____/"));
  Serial.println(F("      /____/ ______         /_/__             ____            "));
  Serial.println(F("            / ____/___  ____  / /__________  / / /__  _____   "));
  Serial.println(F("           / /   / __ \\/ __ \\/ __/ ___/ __ \\/ / / _ \\/ ___/"));
  Serial.println(F("          / /___/ /_/ / / / / /_/ /  / /_/ / / /  __/ /"));
  Serial.println(F("          \\____/\\____/_/ /_/\\__/_/   \\____/_/_/\\___/_/"));
  Serial.println("");
  Serial.println( "Compiled: " __DATE__ ", " __TIME__ ", " __VERSION__);
  Serial.println("");
  Serial.println(F("Enter HELP \"command name\" to get detailled information about a command."));
  Serial.println("");
  Serial.println(F("cat        Show content of a file."));
  Serial.println(F("cfgtmp     Configure HW adresses of temperature sensors."));
  Serial.println(F("cfgflow    Configure the minimum flow per second for refilling the reservoir."));
  Serial.println(F("date       Show or set current date."));
  Serial.println(F("datetime   Show or set current date and time."));
  Serial.println(F("dir        List files on storage media."));
  Serial.println(F("drain      Configure time to wait between irrigation and refill of reservoir."));
  Serial.println(F("hwrst      Do a real HW reset. Use carefully!"));
  Serial.println(F("info       Show current status of controller."));
  Serial.println(F("now        Print current date and time."));
  Serial.println(F("list       List content of EEPROM."));
  Serial.println(F("ls         List files on storage media."));
  Serial.println(F("logint     Configure intervall for logging sensor data."));
  Serial.println(F("nutcal     Calibration values for dosing pumps."));
  Serial.println(F("nutrient   Configure amount of nutrient to be added after refill."));
  Serial.println(F("refill     Trigger manual refill of reservoir."));
  Serial.println(F("reset      Set all parameters to default values."));
  Serial.println(F("sdeject    Disable SD card for save removal."));
  Serial.println(F("sdinit     Re-initialise SD card."));
  Serial.println(F("time       Show or set current time."));
  Serial.println(F("water      List or edit configured timers for irrigation."));
}
void printHelpTime() {
  Serial.println(F("Set time of RTC."));
  Serial.println(F("   time"));
  Serial.println(F("   time <hh>:<mm>:<ss>"));
}
void printHelpDate() {
  Serial.println(F("Set date of RTC."));
  Serial.println(F("   date"));
  Serial.println(F("   date <dd>.<mm>.<yyyy>"));
}
void printHelpDatetime() {
  Serial.println(F("Set date and time of RTC."));
  Serial.println(F("   datetime"));
  Serial.println(F("   datetime <dd>.<mm>.<yyyy> <hh>:<mm>:<ss>"));
}
void printHelpWater() {
  Serial.println(F("Create, change or delete a water timer."));
  Serial.println(F("   water"));
  Serial.println(F("   water <0...8> <hh>:<mm> <0... 255 minutes runtime>"));
  Serial.println(F("   water <0...8> del"));
}
void printHelpNutrient() {
  Serial.println(F("Configure amount of fertilizer to be added after refill."));
  Serial.println(F("   nutrient"));
  Serial.println(F("   nutrient <0...1> <0... 255 ml per 100ml fresh water>"));
}
void printHelpNutcal() {
  Serial.println(F("Set calibration value of dosing pump."));
  Serial.println(F("   nutcal"));
  Serial.println(F("   nutcal <0...1> <0... 255 seconds per ml>"));
}
void printHelpLoginterval() {
  Serial.println(F("Set interval for logging data to SD card."));
  Serial.println(F("   loginterval"));
  Serial.println(F("   loginterval <1...30 minutes>"));
}
void printHelpDrain() {
  Serial.println(F("Set drain time to allow water returning to the reservoir after irrigation."));
  Serial.println(F("   drain"));
  Serial.println(F("   drain <0...255 minutes>"));
}
void printHelpCat() {
  Serial.println(F("Display the content of a file. Use ls or dir to list files on the SD card."));
  Serial.println(F("   cat <filename>"));
}
void printHelpCfgtmp() {
  Serial.println(F("Configure temperature sensor hw adresses."));
  Serial.println(F("   cfgtmp"));
  Serial.println(F("   cfgtmp <0...2 which sensor> <FF FF FF FF FF FF FF FF>"));
}
void printHelpCfgflow() {
  Serial.println(F("Configure the minimum flow required for refilling the reservoir."));
  Serial.println(F("   cfgflow"));
  Serial.println(F("   cfgflow <1...65000 ticks per second>"));
}
void printDateTime() {
  char localBuffer[25]; // "Now: 00.00.0000 00:00:00"
  sprintf_P(localBuffer, PSTR("Now: %02d.%02d.%04d %02d:%02d:%02d"), day(), month(), year(), hour(), minute(), second());
  Serial.println(localBuffer);
}

void logEvent_P(const char *data, byte msgType, boolean serialOnly) {
  char localBuffer[60];

  sprintf_P(localBuffer, PSTR("%02d:%02d:%02d\t"), hour(), minute(), second());
  switch (msgType) {
    case MSG_INFO:
    default:
      strcat_P(localBuffer, PSTR("INFO\t"));
      break;
    case MSG_WARNING:
      strcat_P(localBuffer, PSTR("WARNING\t"));
      break;
    case MSG_ERROR:
      strcat_P(localBuffer, PSTR("ERROR\t"));
      break;
    case MSG_SENSOR:
      strcat_P(localBuffer, PSTR("SENSOR\t"));
      break;
  }
  strcat_P(localBuffer, data);

  if (!serialOnly) {
    logToSd(localBuffer);
  }
  Serial.println(localBuffer);
}

void logEvent(const char *data, byte msgType, boolean serialOnly) {
  char localBuffer[60];

  sprintf_P(localBuffer, PSTR("%02d:%02d:%02d\t"), hour(), minute(), second());
  switch (msgType) {
    case MSG_INFO:
    default:
      strcat_P(localBuffer, PSTR("INFO\t"));
      break;
    case MSG_WARNING:
      strcat_P(localBuffer, PSTR("WARNING\t"));
      break;
    case MSG_ERROR:
      strcat_P(localBuffer, PSTR("ERROR\t"));
      break;
    case MSG_SENSOR:
      strcat_P(localBuffer, PSTR("SENSOR\t"));
      break;
  }
  strcat(localBuffer, data);

  if (!serialOnly) {
    logToSd(localBuffer);
  }
  Serial.println(localBuffer);
}

void logParameters(byte whatToLog, boolean serialOnly) {
  char localBuffer[65];// = "Timer 0: Start: 00:00 End: 00:00 Duration: 000 min (eepromID: 0)";
  unsigned int localStartTime;
  unsigned int localEndTime;

  if (whatToLog == LOG_WATERTIMERS || whatToLog == LOG_ALL) {
    Serial.println(F("********** Water timers **********"));
    sprintf_P(localBuffer, PSTR("%02d:%02d:%02d\tINFO\tWater Timers:"), hour(), minute(), second());
    Serial.println(localBuffer);
    for (int i = 0; i < MAX_WATER_TIMERS; i++) {
      localStartTime = irrigationAlarms[i].startTime;
      localEndTime = irrigationAlarms[i].endTime;

      if (localStartTime > 23 * 60 + 59) {
        localStartTime = 0;
      }
      if (localEndTime > 23 * 60 + 59) {
        localEndTime = 0;
      }

      if (!serialOnly) {
        sprintf_P(localBuffer, PSTR("%02d:%02d:%02d\tINFO\t%d\t%02d:%02d\t%02d:%02d\t%d\t%d"), hour(), minute(), second(), i, localStartTime / 60, localStartTime % 60, localEndTime / 60, localEndTime % 60, irrigationAlarms[i].duration, irrigationAlarms[i].eepromStorageId);
        logToSd(localBuffer);
      }
      sprintf_P(localBuffer, PSTR("Timer %d: Start: %02d:%02d End: %02d:%02d Duration: % 3d min (eepromID: %d)"), i, localStartTime / 60, localStartTime % 60, localEndTime / 60, localEndTime % 60, irrigationAlarms[i].duration, irrigationAlarms[i].eepromStorageId);
      Serial.println(localBuffer);
    }
  }
  if (whatToLog == LOG_NUTRIENTS || whatToLog == LOG_ALL) {
    Serial.println(F("************ Nutrients ***********"));
    for (int i = 0; i < 2; i++) {
      sprintf_P(localBuffer, PSTR("%02d:%02d:%02d\tINFO\tNutrient calibration\t%d\t%d\ts/ml"), hour(), minute(), second(), i, EEPROM.read(EEPROM_NUTRIENT_CALIBRATION + i));
      if (!serialOnly) {
        logToSd(localBuffer);
      }
      Serial.println(localBuffer);
    }
    for (int i = 0; i < 2; i++) {
      sprintf_P(localBuffer, PSTR("%02d:%02d:%02d\tINFO\tNutrient dosing\t%d\t%d\tml/100ml"), hour(), minute(), second(), i, EEPROM.read(EEPROM_NUTRIENT_TIMER + i));
      if (!serialOnly) {
        logToSd(localBuffer);
      }
      Serial.println(localBuffer);
    }
  }
  if (whatToLog == LOG_DRAINTIME || whatToLog == LOG_ALL) {
    Serial.println(F("************ Drain time **********"));
    sprintf_P(localBuffer, PSTR("%02d:%02d:%02d\tINFO\tDrain time\t%d\tmin"), hour(), minute(), second(), EEPROM.read(EEPROM_DRAIN_TIME));
    if (!serialOnly) {
      logToSd(localBuffer);
    }
    Serial.println(localBuffer);
  }

  if (whatToLog == LOG_LOGINTERVAL || whatToLog == LOG_ALL) {
    Serial.println(F("*********** Log interval *********"));
    sprintf_P(localBuffer, PSTR("%02d:%02d:%02d\tINFO\tLog interval\t%d\tmin"), hour(), minute(), second(), EEPROM.read(EEPROM_LOG_INTERVAL));
    if (!serialOnly) {
      logToSd(localBuffer);
    }
    Serial.println(localBuffer);
  }
  if (whatToLog == LOG_CFGFLOW || whatToLog == LOG_ALL) {
    Serial.println(F("********** Min flow rate *********"));
    sprintf_P(localBuffer, PSTR("%02d:%02d:%02d\tINFO\tMin flow rate\t%d\tticks per second"), hour(), minute(), second(), EEPROMReadInt(EEPROM_FLOW_SENSOR_MIN_PER_SECOND));
    if (!serialOnly) {
      logToSd(localBuffer);
    }
    Serial.println(localBuffer);
  }
  if (whatToLog == LOG_PINSTATUS || whatToLog == LOG_ALL) {
    Serial.println(F("*************** FSM **************"));
    Serial.print(F("FSM state:            "));
    switch (stateWatering) {
      case STATE_IDLE:
        Serial.println(F("STATE_IDLE"));
        break;
      case STATE_IRRIGATE:
        Serial.println(F("STATE_IRRIGATE"));
        break;
      case STATE_DRAIN:
        Serial.println(F("STATE_DRAIN"));
        break;
      case STATE_REFILL:
        Serial.println(F("STATE_REFILL"));
        break;
      case STATE_FERTILIZE:
        Serial.println(F("STATE_FERTILIZE"));
        break;
      case STATE_ERROR:
        Serial.println(F("STATE_ERROR"));
        break;
      default:
        Serial.print(F("UNDEFINED   "));
        Serial.println(stateWatering);
        break;
    }
    Serial.println(F("********** SD Card State *********"));
    uint32_t cardSize;
    cardSize = sd.card()->cardSize();

    cid_t cid;
    if (!sd.card()->readCID(&cid)) {
      Serial.print(F("readCID failed"));
    }
    else {
      Serial.print(F("Manufacturer ID: "));
      Serial.println(int(cid.mid), HEX);
      Serial.print(F("OEM ID: "));
      Serial.print(cid.oid[0]);
      Serial.println(cid.oid[1]);
      Serial.print(F("Product: "));
      for (uint8_t i = 0; i < 5; i++) {
        Serial.print(cid.pnm[i]);
      }
      Serial.println("");
      Serial.print(F("Version: "));
      Serial.print(int(cid.prv_n));
      Serial.print('.');
      Serial.println(int(cid.prv_m));
      Serial.print(F("Serial number: "));
      Serial.println(cid.psn, HEX);
      Serial.print(F("Manufacturing date: "));
      Serial.print(int(cid.mdt_month));
      Serial.print('/');
      Serial.println((2000 + cid.mdt_year_low + 10 * cid.mdt_year_high));
    }
    Serial.print(F("\nVolume is FAT"));
    Serial.println(int(sd.vol()->fatType()));
    Serial.print(F("Card Size:  "));
    float cs;
    cs = cardSize * 0.000512F;
    Serial.print((int)cs);
    Serial.print(",");
    Serial.print((int)(cs * 1000.0F) % 1000);
    Serial.println(F(" MB (MB = 1,000,000 bytes)"));

    float fs;
    fs = 0.000512 * sd.vol()->freeClusterCount() * sd.vol()->blocksPerCluster();
    Serial.print(F("Free Space: "));
    Serial.print((int)fs);
    Serial.print(",");
    Serial.print((int)(fs * 1000.0F) % 1000);
    Serial.println(F(" MB (MB = 1,000,000 bytes)"));

    Serial.println(F("********** Output States *********"));
    Serial.print(F("Irrigation pump:      "));
    Serial.print(digitalRead(PIN_IRRIGATION));
    Serial.println("");
    Serial.print(F("Nutrient pump 1:      "));
    Serial.print(digitalRead(PIN_NUTRIENT_1));
    Serial.println("");
    Serial.print(F("Nutrient pump 2:      "));
    Serial.print(digitalRead(PIN_NUTRIENT_2));
    Serial.println("");
    Serial.print(F("Refill valve:         "));
    Serial.print(digitalRead(PIN_REFILL_VALVE));
    Serial.println("");

    Serial.println(F("********** Input States **********"));
    Serial.print(F("Reservoir VERY_FULL:  "));
    Serial.print(digitalRead(PIN_RESERVOIR_VERY_FULL));
    Serial.println("");
    Serial.print(F("Reservoir FULL:       "));
    Serial.print(digitalRead(PIN_RESERVOIR_FULL));
    Serial.println("");
    Serial.print(F("Reservoir EMPTY:      "));
    Serial.print(digitalRead(PIN_RESERVOIR_EMPTY));
    Serial.println("");
    Serial.print(F("Reservoir VERY_EMPTY: "));
    Serial.print(digitalRead(PIN_RESERVOIR_VERY_EMPTY));
    Serial.println("");

    Serial.print(F("Nutrient Solution 1:  "));
    Serial.print("EMPTY");
    Serial.println("");
    Serial.print(F("Nutrient Solution 2:  "));
    Serial.print("EMPTY");
    Serial.println("");

    Serial.println(F("********** Temperatures **********"));
    Serial.print(F("Temperature Cabinet ("));
    printOwAddress(tempCabinet);
    Serial.print(F("):     "));
    Serial.print((int)cabinetTemperatureValue);
    Serial.print(",");
    Serial.print((int)(cabinetTemperatureValue * 10.0F) % 10);
    Serial.print(" ");
    Serial.print((char)176);
    Serial.println("C");

    Serial.print(F("Temperature Outside ("));
    printOwAddress(tempOutside);
    Serial.print(F("):     "));
    Serial.print((int)outsideTemperatureValue);
    Serial.print(",");
    Serial.print((int)(outsideTemperatureValue * 10.0F) % 10);
    Serial.print(" ");
    Serial.print((char)176);
    Serial.println("C");

    Serial.print(F("Temperature Outside ("));
    printOwAddress(tempReservoir);
    Serial.print(F("):     "));
    Serial.print((int)reservoirTemperatureValue);
    Serial.print(",");
    Serial.print((int)(reservoirTemperatureValue * 10.0F) % 10);
    Serial.print(" ");
    Serial.print((char)176);
    Serial.println("C");
  }
}

void logToSd(char *localBuffer) {
#ifdef SDCARD
  if (sdEnabled == true) {
    // open the file for write at end like the Native SD library
    if (!myFile.open(logFileName, O_RDWR | O_CREAT | O_AT_END)) {
      sd.errorPrint(F("opening file faild."));
    }
    else {
      // if the file opened okay, write to it:
      myFile.println(localBuffer);

      // close the file:
      myFile.close();
    }
  }
#endif
}

void readOwArdessesFromEeprom(uint8_t* myDeviceAddress, int eepromStartAddress) {
  for (int i = 0; i < 8; i++) {
    myDeviceAddress[i] = EEPROM.read(eepromStartAddress + i);
  }
}

void interruptRoutineFlowSensorCounter() {
  flowSensorCounter++;
}

/* qsort struct comparision function (hour * 60 + minutes field) */
int struct_cmp_timer(const void *a, const void *b)
{
  struct timer_t *ia = (struct timer_t *)a;
  struct timer_t *ib = (struct timer_t *)b;
  return (ia->startTime - ib->startTime);
  /*
     comparison: returns negative if b > a
     and positive if a > b.
  */
}

void sortWaterTimers() {
  size_t structs_len = sizeof(irrigationAlarms) / sizeof(struct timer_t);
  /* sort array using qsort functions */
  qsort(irrigationAlarms, structs_len, sizeof(struct timer_t), struct_cmp_timer);
}

//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value) {
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.update(p_address, lowByte);
  EEPROM.update(p_address + 1, highByte);
}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address) {
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

void EEPROMSerialDumpTable(int bytesPerRow) {
  // address counter
  int i;

  // row bytes counter
  int j;

  // byte read from eeprom
  byte b;

  // temporary buffer for sprintf
  char buf[10];


  // initialize row counter
  j = 0;

  // go from first to last eeprom address
  for (i = 0; i < 100; i++) {

    // if this is the first byte of the row,
    // start row by printing the byte address
    if (j == 0) {
      sprintf(buf, "%03d: ", i);
      Serial.print(buf);
    }

    // read current byte from eeprom
    b = EEPROM.read(i);

    // write byte in hex form
    sprintf(buf, "%02X ", b);

    // increment row counter
    j++;

    // if this is the last byte of the row,
    // reset row counter and use println()
    // to start a new line
    if (j == bytesPerRow) {
      j = 0;
      Serial.println(buf);
    }
    // else just print the hex value with print()
    else {
      Serial.print(buf);
    }
  }
}

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
