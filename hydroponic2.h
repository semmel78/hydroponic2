//#define SERIAL_TX_BUFFER_SIZE 16
//#define SERIAL_RX_BUFFER_SIZE 16

#define STATE_IDLE       0
#define STATE_IRRIGATE   1
#define STATE_DRAIN      2
#define STATE_REFILL     3
#define STATE_FERTILIZE  4
#define STATE_ERROR      5

#define LOG_ALL          0
#define LOG_WATERTIMERS  1
#define LOG_NUTRIENTS    2
#define LOG_DRAINTIME    3
#define LOG_PINSTATUS    4
#define LOG_LOGINTERVAL  5
#define LOG_CFGFLOW      6


#define MSG_INFO         0
#define MSG_WARNING      1
#define MSG_ERROR        2
#define MSG_SENSOR       3

#define TEMPERATURE_PRECISION 12

#define SERIAL_ONLY      true

// Constructors
void CheckSerialManager();

void printHelp();
void printHelpCat();
void printHelpCfgtmp();
void printHelpDate();
void printHelpDatetime();
void printHelpDrain();
void printHelpLoginterval();
void printHelpNutcal();
void printHelpNutrient();
void printHelpTime();
void printHelpWater();


void printDateTime();

void interruptRoutineFlowSensorCounter();
void readOwArdessesFromEeprom(uint8_t* myDeviceAddress, int eepromStartAddress);

void logEvent_P(const char *data, byte msgType = MSG_INFO, boolean serialOnly = false);
void logEvent(const char *data, byte msgType = MSG_INFO, boolean serialOnly = false);
void logParameters(byte whatToLog, boolean serialOnly = false);
void setLogFileName();

void readWaterTimers();
void sortWaterTimers();
int  struct_cmp_timer(const void *a, const void *b);
int freeRam ();

void EEPROMSerialDumpTable(int bytesPerRow);
void EEPROMWriteInt(int p_address, int p_value);
unsigned int EEPROMReadInt(int p_address);

const int EEPROM_WATER_TIMER = 0;
const int EEPROM_NUTRIENT_TIMER = 27;
const int EEPROM_NUTRIENT_CALIBRATION = 29;
const int EEPROM_LOG_INTERVAL = 30;
const int EEPROM_DRAIN_TIME = 31;
const int EEPROM_FLOW_SENSOR_MIN_PER_SECOND = 32;
const int EEPROM_TEMP_CABINET = 40;
const int EEPROM_TEMP_OUTSIDE = 48;
const int EEPROM_TEMP_RESERVOIR = 56;
const int MAX_WATER_TIMERS = 9;

#ifdef ADRUINO_MEGA
const int PIN_IRRIGATION = 3;
const int PIN_NUTRIENT_1 = 4;
const int PIN_NUTRIENT_2 = 5;
const int PIN_REFILL_VALVE = 6;
const int PIN_REFILL_SENSOR = 16;

const int PIN_RESERVOIR_EMPTY = 22;
const int PIN_RESERVOIR_VERY_EMPTY = 23;
const int PIN_RESERVOIR_FULL = 24;
const int PIN_RESERVOIR_VERY_FULL = 25;
const int PIN_HW_RESET = 54; // connect this pin to RST

const int PIN_BACKLIGHT = 26;    // Push button to activate the backlight of the display
const int PIN_ERRORRESET = 27;   // Push button to reset error state to idle
const int PIN_SDEJECT = 28;      // Push button to disable SD card

const int PIN_ONEWIRE = 2;
const int PIN_CHIP_SELECT = 53;
#else
const int PIN_IRRIGATION = 3;
const int PIN_NUTRIENT_1 = 4;
const int PIN_NUTRIENT_2 = 5;
const int PIN_REFILL_VALVE = 6;
const int PIN_REFILL_SENSOR = 16;

const int PIN_RESERVOIR_EMPTY = 14;
const int PIN_RESERVOIR_VERY_EMPTY = 14;
const int PIN_RESERVOIR_FULL = 15;
const int PIN_RESERVOIR_VERY_FULL = 15;
const int PIN_HW_RESET = 17; // connect this pin to RST

const int PIN_BACKLIGHT = 2;    // Push button to activate the backlight of the display
const int PIN_ERRORRESET = 2;   // Push button to reset error state to idle
const int PIN_EJECTSD = 2;      // Push button to disable SD card

const int PIN_ONEWIRE = 2;
const int PIN_CHIP_SELECT = 10;
#endif

const float ML_PER_TICK = 2.5;  // ml per sensor pulse (1000ml / number of ticks per litre)
const int MAX_REFILL_TIME = 10; // Maximum time for refill state in minutes

struct timer_t {
  unsigned int startTime;
  unsigned int endTime;
  byte duration;
  byte eepromStorageId;
};

const PROGMEM char EEPROMBackup[] = {0x06, 0x00, 0x0F, 0x08, 0x00, 0x0F, 0x0A, 0x00, 0x0F, 0x0C,
                                     0x00, 0x0F, 0x0E, 0x00, 0x0F, 0x10, 0x00, 0x0F, 0x12, 0x00,
                                     0x0F, 0x14, 0x00, 0x0F, 0x01, 0x00, 0x0F, 0x05, 0x07, 0x02,
                                     0x02, 0x03, 0x19, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                     0x28, 0xFF, 0xAD, 0xB5, 0x64, 0x14, 0x01, 0x35, 0xFF, 0xFF
                                    };

