#include <RBD_SerialManager.h>

RBD::SerialManager serial_manager;

// the setup routine runs once when you press reset:
void setup() {

  serial_manager.start();
  serial_manager.setDelimiter(' ');

  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  Serial.println("Setup done");
  /*
    printHelp_time();
    printHelp_date();
    printHelp_datetime();
    printHelp_water();
    printHelp_nutrient();
    printHelp_nutcal();
    printHelp_loginterval();

  */
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
  Serial.println(F("cat        Show content of a file.."));
  Serial.println(F("date       Show or set current date."));
  Serial.println(F("datetime   Show or set current date and time."));
  Serial.println(F("dir        List files on storage media."));
  Serial.println(F("drain      Configure time to wait between irrigation and refill of reservoir."));
  Serial.println(F("hwrst      Do a real HW reset. Use carefully!"));
  Serial.println(F("ls         List files on storage media."));
  Serial.println(F("logint     Configure intervall for logging sensor data."));
  Serial.println(F("nutcal     Calibration values for dosing pumps."));
  Serial.println(F("nutrient   Configure amount of nutrient to be added after refill."));
  Serial.println(F("reset      Set all parameters to default values."));
  Serial.println(F("time       Show or set current time."));
  Serial.println(F("water      List or edit configured timers for irrigation."));
}
void printHelp_time() {
  Serial.println(F("Set time of RTC:"));
  Serial.println(F("   time"));
  Serial.println(F("   time <hh>:<mm>:<ss>"));
}
void printHelp_date() {
  Serial.println(F("Set date of RTC:"));
  Serial.println(F("   date"));
  Serial.println(F("   date <dd>.<mm>.<yyyy>"));
}
void printHelp_datetime() {
  Serial.println(F("Set date and time of RTC:"));
  Serial.println(F("   datetime"));
  Serial.println(F("   datetime <dd>.<mm>.<yyyy> <hh>:<mm>:<ss>"));
}
void printHelp_water() {
  Serial.println(F("Create, change or delete a water timer:"));
  Serial.println(F("   water"));
  Serial.println(F("   water <0...8> <hh>:<mm> <0... 255 minutes runtime>"));
  Serial.println(F("   water <0...8> del"));
}
void printHelp_nutrient() {
  Serial.println(F("Configure amount of fertilizer to be added after refill:>"));
  Serial.println(F("   nutrient"));
  Serial.println(F("   nutrient <0...1> <0... 255 ml per 100ml fresh water>"));
}
void printHelp_nutcal() {
  Serial.println(F("Set calibration value of dosing pump:"));
  Serial.println(F("   nutcal"));
  Serial.println(F("   nutcal <0...1> <0... 255 seconds per ml>"));
}
void printHelp_loginterval() {
  Serial.println(F("Set interval for logging data to SD card:"));
  Serial.println(F("   loginterval"));
  Serial.println(F("   loginterval <1...30 minutes>"));
}





// the loop routine runs over and over again forever:
void loop() {
  if (serial_manager.onReceive()) {
    if (serial_manager.isCmd("help") || serial_manager.isCmd("HELP")) {
      if (serial_manager.getParam().length() > 0) {
        String value = serial_manager.getParam();
        Serial.println("");
        if (value == "time" || value == "TIME") {
          printHelp_time();
        }
        if (value == "date" || value == "DATE") {
          printHelp_date();
        }
        if (value == "datetime" || value == "DATETIME") {
          printHelp_datetime();
        }
        if (value == "water" || value == "WATER") {
          printHelp_water();
        }
      }
      else {
        printHelp();
      }
    }
  }
}



