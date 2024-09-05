#ifndef ISERIAL_H
#define ISERIAL_H

// install SafeString library from Library manager or from https://www.forward.com.au/pfod/ArduinoProgramming/SafeString/index.html
// #include <SafeString.h>
// the loopTimer, BufferedOutput, SafeStringReader and millisDelay are all included in SafeString library V3+
#include <loopTimer.h>
#include "BufferedOutput.h"
#include "BufferedInput.h"
#include <millisDelay.h>
#include <SafeStringReader.h>
#include <jsonlib.h>

// #include "IStepper.h"

#define ARDUINO_MEGA_ID 1
#define TEKNIC_ID 2
#define ESP32_ID 3
#define BIKE_MEGA_ID 4

#define MAX_CMD_LEN 30
#define MIN_CMD_LEN 2 // includes the codeChr, idChr...does not include newline char
#define DELIM_CHARS " ,\n"

#define STATUSPACKETSIZE 9 // this size of the struct Status, in bytes

// CMDS
enum Cmds : char
{
  HOME_CMD = 'H',
  RELPOS_CMD = 'R', // relative position command, moves accelStepper to absolute position, long value expected after
  ABSPOS_CMD = 'P', // absolute position command, moves accelStepper to absolute position, long value expected after
  STOP_CMD = 'S',
  INFO_CMD = 'I',
  SERVOPOSINFO_CMD = 'N',
  STATUS_CMD = 'T',
  SERIAL_OUTPUT = 'X',
  ACC_SET = 'A',     // float value expected after
  VEL_SET = 'V',     // sets the velocityfloat value expected after
  PARAMS_SET = 'G',  // sets the motor params
  ZERO_SET = 'Z',    // sets the motor position to zero
  DEBUG_MSG = 'D',   // turns debug messages on
  JOG_CMD = 'Q',     // velocity command
  CLEAR_CMD = 'C',   // clear alert bits
  KILL_CMD = 'K',    // disable the motor
  ENABLE_CMD = 'E',  // enabled the motor
  MODE_CMD = 'M',    // change operating mode
  CONNECT_CMD = 'U', // request to connect, reply with ID
  EVENT_CMD = 'Y'    // event and data
};

struct Status
{
  int32_t mode = 0;
  int32_t step = 0;
  unsigned char io = 0;
};

class ISerial
{
private:
  byte readBytes[MAX_CMD_LEN + 1];
  int readIndex;

  float accel = 0.0;
  long temp_long = 0;
  float temp_float = 0.0;

  bool skipToDelimiter = false; // bool variable to hold the skipToDelimiter state

  int led = 13;       // new pin for led
  bool ledOn = false; // keep track of the led state
  millisDelay ledDelay;
  millisDelay printDelay;
  millisDelay sensorDelay;
  millisDelay readPositionDelay;

  // TASK DELAYS  adjust these to improve responsiveness of steppper ouptut
  const unsigned long LED_DELAY_MS = 1000;
  const unsigned long PRINT_DELAY_MS = 100;
  const unsigned long SENSOR_DELAY_MS = 1;
  const unsigned long READ_POSITION_DELAY_MS = 100;

  void serializeStatus(Status *msgPacket, char *data);
  long lastModeUpdate_ms = 0;
  bool autoSendStatus = false;

public:
  ISerial();
  bool debug = false;
  long THIS_DEVICE_ID = ARDUINO_MEGA_ID; //!!!!!!!!!!!!!!modify this value for different types of controllers!!!!!!!!!!!!!!!
  char cmdChr;
  bool isConnected = false;
  char idChr = '0';
  String dataString = "";
  void init();
  bool readIncomingData();
  bool handleBasicSerialCmds();
  bool processConnectCmd();
  void processModeCmd();
  void sendStatus(bool writeNewLine);
  bool taskProcessUserInput();
  void taskPrintData(char *data, int num_of_bytes);
  unsigned char *longToByteArray(long &longInt);
  String getDataString();
  bool parseFloat(SafeString &sStr, float &num);
  bool parseFloat(float &num);
  bool parseLong(SafeString &sStr, long &num);
  bool parseLong(long &num);
  bool parseCharAtIndex(char &c,uint8_t index);
  bool parseUint8AtIndex(uint8_t &num, uint8_t index);
  bool parseInt16AtIndex(int16_t &num, uint8_t index);
  void taskBlinkLed(bool run);
  void debugPrintln(String msg);
  void debugPrint(String msg);
  void writeCmdChrIdChr();
  void writeNewline();
  void writeString(String msg);
  void writeLong(long &longInt);
  void writeCmdWarning(String msg);
  void writeSfReader();
  int getLengthSfReader();
  Status status;
  int event;
  bool setNewMode(long newMode);
  static String checkNotEmptyString(String str, bool &isNotEmpty);
  void setIo(int ioNum, bool val);
  long modeTime();
  void resetModeTime();
  void setAutoSendStatus(bool val);
};

#endif