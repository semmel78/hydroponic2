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

#define MSG_INFO         0
#define MSG_WARNING      1
#define MSG_ERROR        2
#define MSG_SENSOR       3

#define TEMPERATURE_PRECISION 10

#define SERIAL_ONLY      true

// Constructors
void CheckSerialManager();
void printHelp_time();
void printHelp_date();
void printHelp_datetime();
void printHelp_water();
void printHelp_nutrient();
void printHelp_nutcal();
void printDateTime();
void printHelp_loginterval();
void interruptRoutineFloatSensorCounter();

void logEvent_P(const char *data, byte msgType = MSG_INFO, boolean serialOnly = false);
void logEvent(const char *data, byte msgType = MSG_INFO, boolean serialOnly = false);
void logParameters(byte whatToLog, boolean serialOnly = false);
void logNutrientParameters(boolean serialOnly = false);
void setLogFileName();

void readWaterTimers();
void sortWaterTimers();
int  struct_cmp_timer(const void *a, const void *b);
int freeRam ();
void eeprom_serial_dump_table(int bytesPerRow);

const int EEPROM_WATER_TIMER = 0;
const int EEPROM_NUTRIENT_TIMER = 27;
const int EEPROM_NUTRIENT_CALIBRATION = 29;
const int EEPROM_LOG_INTERVAL = 30;
const int EEPROM_DRAIN_TIME = 31;
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
