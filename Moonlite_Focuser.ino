#include "AccelStepper.h"
#include "EEPROM.h"
///////////////////////////
// DELAY to disable output driver after last move at 5s interval
// will average out temperature in last 150s.
///////////////////////////
#define                STEPPER_DISABLEDELAY 5000

///////////////////////////
// Serial Interface Signals
///////////////////////////

#define MAXCOMMAND 8
char                   inChar;
char                   cmd[MAXCOMMAND];
char                   param[MAXCOMMAND];
char                   packet[MAXCOMMAND];
int                    idx = 0;
boolean                inputcmd = false;

///////////////////////////
// Motor Control Signals
///////////////////////////

long                   TargetPosition = 0;
long                   CurrentPosition = 0;
long                   stepIncr=1;
boolean                isRunning = false;

///////////////////////////
// manage speed
///////////////////////////
int                    SpeedFactor = 1000;
int                    SpeedFactorRaw = 2;

///////////////////////////
// Timer
///////////////////////////
long                   millisLastMove = 0;                // Last move timer to turn off stepper output

///////////////////////////
// Output/Input pins
///////////////////////////
//#define HALFSTEP 8 // Motor pin definitions
#define SPEED_PIN 9
#define DIRECTION_PIN 8

///////////////////////////
// Backlash, NOT being used
///////////////////////////
int                    Backlash = 0;

///////////////////////////
//Manual move control
///////////////////////////
#define                BUT_READING_RELEASED 0
#define                BUT_READING_PRESSED 1
int                    lastReadingButFW = BUT_READING_RELEASED;               //
int                    lastReadingButBW = BUT_READING_RELEASED;
// Button press timer to increase motor move steps 
// (ie, effective motor speed).
long                   millisButFWPressed = 0;
long                   millisButBWPressed = 0;

///////////////////////////
// EEPROM interface
///////////////////////////
#define                EEPROM_POS_LOC 0
long                   lastSavedPosition = 0;

///////////////////////////
// Temperature
///////////////////////////
#define                TMP36 0
///////////////////////////
// Misc signals
///////////////////////////
// Moonlite compatability mode - 0.5 degree temparture reading accuracy
// Set to false will return 0.125 accuracy
boolean                MoonliteMode = true;

int                    i;


// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48

AccelStepper stepper1(AccelStepper::DRIVER, SPEED_PIN, DIRECTION_PIN);

void setup() {
    Serial.begin(9600);
    stepper1.setMaxSpeed(SpeedFactor);
    stepper1.setAcceleration(50.0);
    stepper1.setSpeed(SpeedFactorRaw);
    // initialize serial command: packet set to 0
    memset(packet, 0, MAXCOMMAND);
    // read saved position from EEPROM
    EEPROM.get(EEPROM_POS_LOC, CurrentPosition);
    stepper1.setCurrentPosition(CurrentPosition);
    lastSavedPosition = CurrentPosition;
}

void loop() {
    if (inputcmd)
    {
      memset(cmd, 0, MAXCOMMAND);
      memset(param, 0, MAXCOMMAND);
      // length of received cmd
      int len = strlen(packet);
      // This is a command on 2 chars
      strncpy(cmd, packet, 2);
      if (len > 2)
      {
        // with parameters
        strncpy(param, packet + 2, len - 2);
      }
      // Reset buffer and flag
      memset(packet, 0, MAXCOMMAND);
      inputcmd = false;
      idx = 0;

      // home the motor, hard-coded, ignore parameters
      // since we only have one motor
      if (!strcasecmp(cmd, "PH")) {
        stepper1.setCurrentPosition(8000);
        stepper1.moveTo(0);
        isRunning = true;
      }
      //---------------------------------------------------
      // Set the new position :SNYYYY# where YYYY is a 4 
      // digit unsigned hex number 
      //---------------------------------------------------
      if (!strcasecmp(cmd, "SN"))
      {
        TargetPosition = hexstr2long(param);       
      }
      //---------------------------------------------------
      // initiate a move
      //---------------------------------------------------
      if (!strcasecmp(cmd, "FG"))
      {
        stepper1.enableOutputs();
        stepper1.moveTo(TargetPosition);
      }
      //---------------------------------------------------
      // get the new motor position (target) set by SN
      //---------------------------------------------------
      if (!strcasecmp(cmd, "GN"))
      {
        char tempString[6];
        sprintf(tempString, "%04X", TargetPosition);
        Serial.print(tempString);
        Serial.print("#");
      }
      //---------------------------------------------------
      // set current motor position:SPYYYY# where YYYY is a 4 
      // digit unsigned hex number
      //---------------------------------------------------
      if (!strcasecmp(cmd, "SP"))
      {
        CurrentPosition = hexstr2long(param);
        stepper1.setCurrentPosition(CurrentPosition);
      }
      //---------------------------------------------------
      // Change increment value
      //---------------------------------------------------
      if (!strcasecmp(cmd,"I+"))
      {
         stepIncr=hexstr2long(param);
      }
      //---------------------------------------------------
      // get the current motor position
      //---------------------------------------------------
      if (!strcasecmp(cmd, "GP")) {
        CurrentPosition = stepper1.currentPosition();
        char tempString[6];
        sprintf(tempString, "%04X", CurrentPosition);
        Serial.print(tempString);
        Serial.print("#");
      }
      //---------------------------------------------------
      // stop a move
      // stepper.stop() stops motor gracefully,
      // as a result motor may continue running for sometime
      // (upto 1000 step at max speed setting), depending 
      // the current speed.
      // if we stop the motor abruptly then somehow 
      // stepper library does not handle current/target position
      // correctly.
      //---------------------------------------------------
      if (!strcasecmp(cmd, "FQ"))
      {
        stepper1.stop();
      }
      //---------------------------------------------------
      // set speed, only acceptable values are 02, 04, 08, 10, 20
      //---------------------------------------------------
      if (!strcasecmp(cmd, "SD"))
      {
        SpeedFactorRaw = hexstr2long(param);
        // SpeedFactor: smaller value means faster
        SpeedFactor = 2000 / SpeedFactorRaw;
        //SpeedFactor = 32 / SpeedFactorRaw;
        stepper1.setMaxSpeed(SpeedFactor);
        //stepper.setMaxSpeed(SpeedFactor * SPEEDMULT);
      }
      //---------------------------------------------------
      // get the current motor speed, only values of 02, 04,
      // 08, 10, 20, which is set by SD
      //---------------------------------------------------
      if (!strcasecmp(cmd, "GD"))
      {
        char tempString[6];
        sprintf(tempString, "%02X", SpeedFactorRaw);
        Serial.print(tempString);
        Serial.print("#");
      }
      //---------------------------------------------------
      // whether half-step is enabled or not, always return "00"
      //---------------------------------------------------
      if (!strcasecmp(cmd, "GH"))
      {
        Serial.print("FF#");
      }
      //---------------------------------------------------
      // motor is moving - 01 if moving, 00 otherwise
      //---------------------------------------------------
      if (!strcasecmp(cmd, "GI"))
      {
        if (isRunning)
        {
          Serial.print("01#");
        }
        else
        {
          Serial.print("00#");
        }
      }
      //---------------------------------------------------
      // firmware value, always return "10"
      //---------------------------------------------------
      if (!strcasecmp(cmd, "GV"))
      {
        Serial.print("10#");
      }
      //---------------------------------------------------
      // set full step mode
      //---------------------------------------------------
      if (!strcasecmp(cmd, "SF"))
      {
        // do nothing
      }
      //---------------------------------------------------
      // set half step mode
      //---------------------------------------------------
      if (!strcasecmp(cmd, "SH"))
      {
        // do nothing
      }
      //---------------------------------------------------
      // reset compatability mode
      //---------------------------------------------------
      if (!strcasecmp(cmd, "YM"))
      {
        MoonliteMode = false;
      }
      //---------------------------------------------------
      // set backlash
      //---------------------------------------------------
      if (!strcasecmp(cmd, "YB"))
      {
        Backlash = hexstr2long(param);
      }
      //---------------------------------------------------
      // get backlash set by YB
      //---------------------------------------------------
      if (!strcasecmp(cmd, "ZB"))
      {
        char tempString[6];
        sprintf(tempString, "%02X", Backlash);
        Serial.print(tempString);
        Serial.print("#");
      }
      //---------------------------------------------------
      // Temperatures
      //---------------------------------------------------
      if (!strcasecmp(cmd,"GT"))
      {
        char tempString[6];
        //se ha comprobado que hay que multiplicar por 2 y pasarle un entero para que las aplicaciones den un valor real
        long temperature = 2 * getTemperature();
        sprintf(tempString, "%04X", temperature);
        Serial.print(tempString);
        Serial.print("#");
        
      }
      //---------------------------------------------------
      // set the temperature coefficient
      //---------------------------------------------------
      if (!strcasecmp(cmd, "SC"))
      {
        // do nothing
      }
      //---------------------------------------------------
      // get the temperature coefficient
      //---------------------------------------------------
      if (!strcasecmp(cmd, "GC"))
      {
        char tempString[6];
        sprintf(tempString, "%02X", 1);
        Serial.print(tempString);
        Serial.print("#");
      }
    }
    if (stepper1.distanceToGo() != 0)
    {
      isRunning = true;
      millisLastMove = millis();
      stepper1.run();
    }
   else
    {
      isRunning = false;
  
      // To check if motor has stopped for long time.
      if ((millis() - millisLastMove) > STEPPER_DISABLEDELAY)
      {
        millisLastMove = millis();
  
        // turn off driver to save power.
        stepper1.disableOutputs();
  
        // Save current location in EEPROM
        if (lastSavedPosition != CurrentPosition)
        {
          EEPROM.put(EEPROM_POS_LOC, CurrentPosition);
          lastSavedPosition = CurrentPosition;
        }
      }
    }
   
}

//read temperature
float getTemperature(){
  int reading = analogRead(TMP36);
  float voltage = reading * 5.0 / 1024.0;
  float temp = (voltage - 0.5) * 100;
  return (temp);
}

// read the command until the terminating # character
void serialEvent () {
  while (Serial.available() && !inputcmd)
  {
    inChar = Serial.read();
    if (inChar != '#' && inChar != ':')
    {
      packet[idx++] = inChar;
      if (idx >= MAXCOMMAND)
      {
        idx = MAXCOMMAND - 1;
      }
    }
    else
    {
      if (inChar == '#')
      {
        inputcmd=true;
      }
    }
  }
}

// Convert hex number explained as char to long int
long hexstr2long(char *line) {
  long ret = 0;
  ret = strtol(line, NULL, 16);
  return (ret);
}
