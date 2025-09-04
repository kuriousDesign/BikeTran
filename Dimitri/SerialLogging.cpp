// SerialLogging.cpp (Rewritten with custom byte-wise ring buffer)
#include "SerialLogging.h"

uint8_t SerialLogging::queue[SerialLogging::BYTE_BUFFER_SIZE];
volatile size_t SerialLogging::head = 0;
volatile size_t SerialLogging::tail = 0;
uint8_t SerialLogging::output_BUFFER[SerialLogging::BUFFER_SIZE];
BufferedOutput SerialLogging::output(sizeof(SerialLogging::output_BUFFER), SerialLogging::output_BUFFER, DROP_IF_FULL);
bool SerialLogging::debug = false;

bool SerialLogging::isFull()
{
  return ((tail + 1) % BYTE_BUFFER_SIZE) == head; // Leave one slot empty to distinguish full/empty
}

bool SerialLogging::isEmpty()
{
  return head == tail;
}

bool SerialLogging::pushByte(uint8_t byte)
{
  if (isFull())
  {
    return false; // Full: Drop
  }
  if (byte == 0)
  {
    queue[tail] = static_cast<uint8_t>(15);
  }
  else
  {
    queue[tail] = byte;
  }
  tail = (tail + 1) % BYTE_BUFFER_SIZE;
  return true;
}

bool SerialLogging::popByte(uint8_t &byte)
{
  if (isEmpty())
  {
    return false; // Empty: Nothing to pop
  }
  byte = queue[head];
  head = (head + 1) % BYTE_BUFFER_SIZE;
  return true;
}

void SerialLogging::init()
{
  Serial.begin(115200);
  output.connect(Serial, 115200);
  head = 0;
  tail = 0; // Reset indices
  setDebug(false); // Default to no debug
}

void SerialLogging::add(const char *fmt, ...)
{
  if (!debug)
  {
    return; // Skip if debug not enabled
  }
  char msg[MAX_MSG_SIZE];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, MAX_MSG_SIZE, fmt, args);
  va_end(args);

  size_t len = strlen(msg);
  noInterrupts(); // Protect the batch push
  bool canAdd = true;
  for (size_t i = 0; i < len; ++i)
  {
    if (!pushByte(static_cast<uint8_t>(msg[i])))
    {
      canAdd = false;
      break; // Drop partial if full (or rollback if needed, but simple drop for now)
    }
  }
  interrupts();
}

void SerialLogging::info(const char *fmt, ...)
{
  if (!debug)
  {
    return; // Skip if debug not enabled
  }
  char msg[MAX_MSG_SIZE];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, MAX_MSG_SIZE, fmt, args);
  va_end(args);

  size_t len = strlen(msg);
  if (len < MAX_MSG_SIZE - 1)
  { // Append newline if space
    msg[len++] = '\n';
    msg[len] = '\0';
  }
  else
  {
    msg[MAX_MSG_SIZE - 1] = '\n';
    len = MAX_MSG_SIZE;
  }

  noInterrupts(); // Protect the batch push
  bool canAdd = true;
  for (size_t i = 0; i < len; ++i)
  {
    if (!pushByte(static_cast<uint8_t>(msg[i])))
    {
      canAdd = false;
      break;
    }
  }
  interrupts();
}

void SerialLogging::warn(const char *fmt, ...)
{
  if (!debug)
  {
    return; // Skip if debug not enabled
  }
  char msg[MAX_MSG_SIZE];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, MAX_MSG_SIZE, fmt, args);
  va_end(args);

  const char *prefix = "WARNING: ";
  size_t prefixLen = strlen(prefix);
  size_t msgLen = strlen(msg);
  if (prefixLen + msgLen < MAX_MSG_SIZE - 1)
  {
    memmove(msg + prefixLen, msg, msgLen + 1);
    memcpy(msg, prefix, prefixLen);
    msgLen += prefixLen;
  }

  if (msgLen < MAX_MSG_SIZE - 1)
  {
    msg[msgLen++] = '\n';
    msg[msgLen] = '\0';
  }
  else
  {
    msg[MAX_MSG_SIZE - 1] = '\n';
    msgLen = MAX_MSG_SIZE;
  }

  noInterrupts();
  bool canAdd = true;
  for (size_t i = 0; i < msgLen; ++i)
  {
    if (!pushByte(static_cast<uint8_t>(msg[i])))
    {
      canAdd = false;
      break;
    }
  }
  interrupts();
}

void SerialLogging::error(const char *fmt, ...)
{
  if (!debug)
  {
    return; // Skip if debug not enabled
  }
  char msg[MAX_MSG_SIZE];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, MAX_MSG_SIZE, fmt, args);
  va_end(args);

  const char *prefix = "ERROR: ";
  size_t prefixLen = strlen(prefix);
  size_t msgLen = strlen(msg);
  if (prefixLen + msgLen < MAX_MSG_SIZE - 1)
  {
    memmove(msg + prefixLen, msg, msgLen + 1);
    memcpy(msg, prefix, prefixLen);
    msgLen += prefixLen;
  }

  if (msgLen < MAX_MSG_SIZE - 1)
  {
    msg[msgLen++] = '\n';
    msg[msgLen] = '\0';
  }

  noInterrupts();
  bool canAdd = true;
  for (size_t i = 0; i < msgLen; ++i)
  {
    if (!pushByte(static_cast<uint8_t>(msg[i])))
    {
      canAdd = false;
      break;
    }
  }
  interrupts();
}

// prepend with data header
void SerialLogging::publishData(byte *data, size_t len, uint8_t id)
{
  if (len > 255)
  {
    SerialLogging::warn("publishData: Data length %u exceeds max 255", len);
    return;
  }

  noInterrupts();

  // Ensure enough buffer space before committing
  size_t required = 1 + 1 + 1 + len + 1 + 1; // STX + LEN + ID + DATA + ETX + NEWLINE
  size_t available = (tail >= head)
                         ? (BYTE_BUFFER_SIZE - (tail - head) - 1)
                         : (head - tail - 1);

  if (required > available)
  {
    interrupts();
    SerialLogging::warn("publishData: Insufficient buffer space");
    return;
  }
  // in need to create a serial protocol that provides a reliable starting byte, followed by the data length and then the data itself
  // and then the data byte chunk and then a terminating byte with and then end line char
  // maybe something like STX (0x02), LEN (1 byte), ID,  DATA (LEN bytes), ETX (0x03), NEWLINE (0x0A)
  //[STX][LEN][ID][DATA...][ETX][\n]

  // Start of packet
  pushByte(STX);
  pushByte(static_cast<uint8_t>(len));
  pushByte(static_cast<uint8_t>(id));

  for (size_t i = 0; i < len; ++i)
  {
    pushByte(static_cast<uint8_t>(data[i]));
    // pushByte(static_cast<uint8_t>());
  }

  pushByte(ETX);     // append ET
  pushByte(NEWLINE); // append newline
  interrupts();
}

void SerialLogging::process()
{
  uint8_t byteVal;
  while (output.availableForWrite() > 0)
  {
    noInterrupts(); // Protect pop
    if (!popByte(byteVal))
    {
      interrupts();
      break; // Empty: Stop
    }
    interrupts();
    output.write(byteVal); // Add byte to BufferedOutput
  }
  output.nextByteOut(); // Release bytes to Serial non-blockingly
}

void SerialLogging::setDebug(bool state)
{
  debug = state;
}