#include "ISerial.h"
#include "BufferedOutput.h"
#include "BufferedInput.h"
#include <millisDelay.h>
#include <SafeStringReader.h>
#include <jsonlib.h>
#include <loopTimer.h>

createSafeStringReader(sfReader, MAX_CMD_LEN, DELIM_CHARS); // create a SafeString reader with max Cmd Len 15 and delimiters space, comma, Carrage return and Newline
// Example of using BufferedOutput to release bytes when there is space in the Serial Tx buffer, extra buffer size MAX_CMD_LEN
createBufferedOutput(bufferedOut, MAX_CMD_LEN, DROP_UNTIL_EMPTY);
createBufferedInput(bufferedIn, MAX_CMD_LEN);

// instantiate and initialize
ISerial::ISerial()
{
}

// the setup function runs once when you press reset or power the board
void ISerial::init()
{
  Serial.begin(115200);
  Serial.flush();
  // SafeString::setOutput(Serial); //uncomment this to enable error msgs
  bufferedOut.connect(Serial); // connect bufferedOut to Serial object
  bufferedIn.connect(bufferedOut);
  sfReader.connect(bufferedOut);
  sfReader.echoOff();       // echo goes out via bufferedOut
  sfReader.setTimeout(100); // set 100ms == 0.1sec non-blocking timeout

  // start the task delay timers
  ledDelay.start(LED_DELAY_MS);     // start the ledDelay
  printDelay.start(PRINT_DELAY_MS); // start the printDelay
}

// TASK - BLINK LED ////////////////////////////////////////////////////////////////////
void ISerial::taskBlinkLed(bool run)
{
  if (ledDelay.justFinished())
  {                    // check if delay has timed out
    ledDelay.repeat(); // start delay again without drift
    if (!run)
    {
      digitalWrite(led, LOW); // turn led on/off
      ledOn = false;
      return;
    }
    else
    {
      ledOn = !ledOn;                        // toggle the led
      digitalWrite(led, ledOn ? HIGH : LOW); // turn led on/off
    }
  } // else nothing to do this call just return, quickly
}

// TASK - PRINT STATUS //////////////////////////////////////////////////
void ISerial::taskPrintData(char *data, int num_of_bytes)
{
  for (int i = 0; i < num_of_bytes; i++)
  {
    bufferedOut.write(data[i]);
    // runStepper(); // <<<< extra call here
  }
}

// returns true if there is new data to process
bool ISerial::readIncomingData()
{
  if (sfReader.read()) // reads input stream if data available, returns true if delimiter is found
  {                    // non-blocking!!
    // loopTimer.check(bufferedOut); // moved here from loop()
    // sfReader.toUpperCase(); // ignore case
    cmdChr = sfReader.charAt(0);
    idChr = sfReader.charAt(1);
    sfReader.remove(0, 2);
    return true;
  }
  return false;
}

// returns true if connection is success
bool ISerial::processConnectCmd()
{
  if (!parseLong(sfReader, temp_long))
  {
    int search_id = temp_long;
    if (search_id != THIS_DEVICE_ID)
    {
      debugPrintln("Search ID doesn't match this device's ID: " + String(THIS_DEVICE_ID));
    }
    else
    {
      isConnected = true;
      debugPrintln("Search ID matches :)");
    }
    bufferedOut.write(cmdChr);
    bufferedOut.write(idChr);

    // convert long to byte array and write
    unsigned char byteArray[4];
    memcpy(byteArray, &THIS_DEVICE_ID, 4);
    for (int i = 0; i < 4; i++)
    {
      bufferedOut.write(byteArray[i]);
    }
    return true;
  }
  else
  {
    bufferedOut.print(F("WARNING: parseLong error"));
  }
  return false;
}

void ISerial::processModeCmd()
{
  if (!parseLong(sfReader, temp_long))
  {
    setNewMode(temp_long);

    bufferedOut.write(cmdChr);
    bufferedOut.write(idChr);

    // convert long to byte array and write
    unsigned char byteArray[4];
    memcpy(byteArray, &temp_long, 4);
    for (int i = 0; i < 4; i++)
    {
      bufferedOut.write(byteArray[i]);
    }
  }
  else
  {
    bufferedOut.print(F("WARNING: not able to parse long in mode cmd"));
  }
}

void ISerial::sendStatus(bool writeNewLine)
{
  bufferedOut.write(Cmds::STATUS_CMD);
  bufferedOut.write('0');
  // serialize the status data into a data array
  char data[STATUSPACKETSIZE];
  // Status *newMsg = &status;
  serializeStatus(&status, data);

  // send the data array
  taskPrintData(data, STATUSPACKETSIZE);
  if (writeNewLine)
  {
    writeNewline();
  }
}

bool ISerial::handleBasicSerialCmds()
{
  switch (cmdChr)
  {
  case Cmds::CONNECT_CMD:
    processConnectCmd();
    break;
  case Cmds::MODE_CMD:
    processModeCmd();
    break;
  case Cmds::EVENT_CMD:
    // iSerial.writeNewline("WARNING: Event Cmd handling missing (debug)");
    if (!parseLong(sfReader, temp_long))
    {
      event = temp_long;
      debugPrint("new event: ");
      debugPrintln(String(event));
      bufferedOut.write(cmdChr);
      bufferedOut.write(idChr);
    }
    else
    {
      bufferedOut.print(F("WARNING: not able to parse long in event cmd"));
    }
    break;
  case Cmds::DEBUG_MSG: // DEBUG MSG
    // statements

    if (debug)
    {
      debug = false;
    }
    else
    {
      debug = true;
      debugPrintln("debug messages now available");
    }
    bufferedOut.write(cmdChr);
    bufferedOut.write(idChr);
    break;

  case Cmds::STATUS_CMD:
    sendStatus(false);
    break;

  default:
    debugPrintln("not a standard command");
    return true;
    break;
  }

  bufferedOut.write('\n');
  return false;
}

// to get the user's cmds, input commands terminated by space or , or \r or \n or no new characters for 2secs
// set Global variable with input cmd
bool ISerial::taskProcessUserInput()
{
  if (readIncomingData()) // reads input stream if data available, returns true if delimiter is found
  {                       // non-blocking!!
    return handleBasicSerialCmds();
  }
  else
  {
    return false;
  }
}

void ISerial::writeLong(long &longInt)
{
  // convert long to byte array and write
  unsigned char byteArray[4];
  memcpy(byteArray, &longInt, 4);
  for (int i = 0; i < 4; i++)
  {
    bufferedOut.write(byteArray[i]);
  }
}

// write the cmd char and the motor id char only
void ISerial::writeCmdChrIdChr()
{
  bufferedOut.write(cmdChr);
  bufferedOut.write(idChr);
}

// write the newline char: \n
void ISerial::writeNewline()
{
  // bufferedOut.print(msg);
  bufferedOut.write('\n');
}

void ISerial::writeString(String msg)
{
  bufferedOut.print(msg);
}

// handles writing a warning back if cmd was not understood, already prints "WARNING: " for you
void ISerial::writeCmdWarning(String msg)
{
  String warningMsg = "WARNING: " + msg;
  bufferedOut.print(warningMsg);
}

// write the contents of the safe string reader
void ISerial::writeSfReader(){
  bufferedOut.print(sfReader);
}

int ISerial::getLengthSfReader(){
  return sfReader.length();
}
// CONVERT LONG TO BYTE ARRAY //////////////////
unsigned char *ISerial::longToByteArray(long &longInt)
{
  unsigned char byteArray[4];
  byteArray[0] = (int)((longInt >> 24) & 0xFF);
  byteArray[1] = (int)((longInt >> 16) & 0xFF);
  byteArray[2] = (int)((longInt >> 8) & 0XFF);
  byteArray[3] = (int)((longInt & 0XFF));
  return byteArray;
}

String ISerial::getDataString()
{
  String dataStr = "";
  int N = sfReader.length();
  for (int i = 0; i < N; i++)
  {
    dataStr += sfReader.charAt(i);
  }
  ISerial::dataString = dataStr;
  return dataStr;
}

// PARSE FLOAT /////////////////////////////////
// returns true if unable to parse a float
bool ISerial::parseFloat(SafeString &sStr, float &num)
{
  float temp = 0.0;
  if (!sStr.toFloat(temp))
  {              // an empty field is not a valid float
    return true; // not valid skip the rest
  }
  num = temp;
  return false;
}

// PARSE FLOAT /////////////////////////////////
// returns true if unable to parse a float
bool ISerial::parseFloat(float &num)
{
  float temp = 0.0;
  if (!sfReader.toFloat(temp))
  {              // an empty field is not a valid float
    return true; // not valid skip the rest
  }
  num = temp;
  return false;
}

// returns true if unable to parse a Long Int
bool ISerial::parseLong(SafeString &sStr, long &num)
{
  long temp = 0;
  if (!sStr.toLong(temp))
  {              // an empty field is not a valid float
    return true; // not valid skip the rest
  }
  num = temp;
  return false;
}

// returns true if unable to parse a Long Int
bool ISerial::parseLong(long &num)
{
  long temp = 0;
  if (!sfReader.toLong(temp))
  {              // an empty field is not a valid float
    return true; // not valid skip the rest
  }
  num = temp;
  return false;
}

// returns true if unable to parse char at the index
bool ISerial::parseCharAtIndex(char &c, uint8_t index)
{
  if (sfReader.length() <= index)
  {
    return true;
  }
  c = sfReader.charAt(index);
  return false;
}

// parses a uint8_t from char at the given index
bool ISerial::parseUint8AtIndex(uint8_t &num, uint8_t index)
{
  char c;
  if (parseCharAtIndex(c, index))
  {
    return true;
  }
  num = static_cast<uint8_t>(c);
  return false;
}

bool ISerial::parseInt16AtIndex(int16_t &num, uint8_t index)
{
  if (sfReader.length() < index + 2)
  {
    return true;
  }
  char firstByte = sfReader.charAt(index);
  char secondByte = sfReader.charAt(index + 1);
  num = (firstByte << 8) | secondByte;
  return false;
}

// set this to true if you want to automatically send status after mode changes
void ISerial::setAutoSendStatus(bool val)
{
  autoSendStatus = val;
}

// sets a new mode and if it's new it initializes the step number
bool ISerial::setNewMode(long newMode)
{
  if (status.mode != newMode)
  {
    status.mode = newMode;
    status.step = 0;
    event = 0;
    if (autoSendStatus & isConnected)
    {
      sendStatus(true);
    }
    debugPrint("mode change: ");
    debugPrintln(String(status.mode));
    lastModeUpdate_ms = millis();
    return true;
  }
  else
  {
    return false;
  }
}

// returns time since the mode started (ms)
long ISerial::modeTime()
{
  return millis() - lastModeUpdate_ms;
}

void ISerial::resetModeTime()
{
  lastModeUpdate_ms = millis();
}

// converts the Status struct data into byte array to be used for sending over serial port
void ISerial::serializeStatus(Status *msgPacket, char *data)
{
  // sending int32_t vals
  int32_t *q = (int32_t *)data;
  *q = msgPacket->mode;
  q++;
  *q = msgPacket->step;
  q++;

  // sending char vals
  char *p = (char *)q;
  *p = msgPacket->io;
  p++;

  /*
  float *f = (float *)q;
  *f = msgPacket->actualVelocity;
  f++;
  *f = msgPacket->referenceVelocity;
  f++;
  */
}

// sets the bool isNotEmpty to false only if string is empty,
String ISerial::checkNotEmptyString(String str, bool &isNotEmpty)
{
  // debugPrint("val: ");
  // debugPrintln(str);
  if (str.startsWith("{"))
  {
    isNotEmpty = false;
  }
  else
  {
    // ignore
  }
  return str;
}

void ISerial::setIo(int ioNum, bool val)
{
  if (val)
  {
    status.io |= (1 << ioNum);
  }
  else
  {
    status.io &= ~(1 << ioNum);
  }
}

void ISerial::debugPrint(String msg)
{
  if (debug)
  {
    Serial.print(msg);
  }
}

void ISerial::debugPrintln(String msg)
{
  if (debug)
  {
    Serial.println(msg);
  }
}