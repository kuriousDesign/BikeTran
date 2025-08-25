#include "SerialLogging.h"
#include <RingBuf.h>

RingBuf* SerialLogging::queue = nullptr;
uint8_t SerialLogging::output_BUFFER[SerialLogging::BUFFER_SIZE];  // Manual definition of static buffer
BufferedOutput SerialLogging::output(sizeof(SerialLogging::output_BUFFER), SerialLogging::output_BUFFER, DROP_IF_FULL);  // Manual init (drop if full; add false for AllOrNothing if needed)
void SerialLogging::init() {
  Serial.begin(115200);  // Or your baud rate
  output.connect(Serial, 115200);

  queue = RingBuf_new(MAX_MSG_SIZE, QUEUE_LENGTH);
  if (queue == nullptr) {
    // Handle allocation failure - reduce memory usage or check available RAM
    Serial.println("ERROR: Failed to allocate logging queue");
    
    // Try with smaller parameters
    queue = RingBuf_new(MAX_MSG_SIZE, QUEUE_LENGTH / 2);
    if (queue == nullptr) {
      Serial.println("ERROR: Even reduced allocation failed");
      while (true) {}  // Halt
    } else {
      Serial.println("WARNING: Using reduced queue size");
    }
  }
}

void SerialLogging::add(const char* fmt, ...) {
  char msg[MAX_MSG_SIZE];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, MAX_MSG_SIZE, fmt, args);  // Format safely (no newline added)
  va_end(args);

  // Add to queue (interrupt-safe per library)
  if (queue->add(queue, msg) == 0) {
    // Queue full: Optionally handle (e.g., drop silently or log error)
  }
}

void SerialLogging::info(const char* fmt, ...) {
  char msg[MAX_MSG_SIZE];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, MAX_MSG_SIZE, fmt, args);  // Format safely
  va_end(args);

  // Append newline if there's space
  size_t len = strlen(msg);
  if (len < MAX_MSG_SIZE - 1) {  // Ensure room for '\n' and null terminator
    msg[len] = '\n';
    msg[len + 1] = '\0';
  }

  // Add to queue (interrupt-safe per library)
  if (queue->add(queue, msg) == 0) {
    // Queue full: Optionally handle (e.g., drop silently or log error)
  }
}

void SerialLogging::warn(const char* fmt, ...) {
  char msg[MAX_MSG_SIZE];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, MAX_MSG_SIZE, fmt, args);
  va_end(args);

  // Prepend "WARNING: " to the message
  const char* warnPrefix = "WARNING: ";
  size_t prefixLen = strlen(warnPrefix);
  size_t msgLen = strlen(msg);

  if (prefixLen + msgLen < MAX_MSG_SIZE - 1) {
    // Shift the original message to make room for the prefix
    memmove(msg + prefixLen, msg, msgLen + 1);
    memcpy(msg, warnPrefix, prefixLen);
  }

  // Append newline if there's space
  size_t totalLen = strlen(msg);
  if (totalLen < MAX_MSG_SIZE - 1) {
    msg[totalLen] = '\n';
    msg[totalLen + 1] = '\0';
  }

  if (queue->add(queue, msg) == 0) {
    // Queue full: Optionally handle
  }
}


void SerialLogging::error(const char* fmt, ...) {
  char msg[MAX_MSG_SIZE];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, MAX_MSG_SIZE, fmt, args);
  va_end(args);

  // Prepend "ERROR: " to the message
  const char* errorPrefix = "ERROR: ";
  size_t prefixLen = strlen(errorPrefix);
  size_t msgLen = strlen(msg);

  if (prefixLen + msgLen < MAX_MSG_SIZE - 1) {
    // Shift the original message to make room for the prefix
    memmove(msg + prefixLen, msg, msgLen + 1);
    memcpy(msg, errorPrefix, prefixLen);
  }

  // Append newline if there's space
  size_t totalLen = strlen(msg);
  if (totalLen < MAX_MSG_SIZE - 1) {
    msg[totalLen] = '\n';
    msg[totalLen + 1] = '\0';
  }

  if (queue->add(queue, msg) == 0) {
    // Queue full: Optionally handle
  }
}

void SerialLogging::process() {
  char msg[MAX_MSG_SIZE];
  static char* currentMsg = nullptr;
  static size_t currentPos = 0;
  
  // If no message is currently being sent, try to get a new one
  if (currentMsg == nullptr && !queue->isEmpty(queue)) {
    if (queue->pull(queue, msg) != nullptr) {
      currentMsg = msg;
      currentPos = 0;
    }
  }
  
  // Send one byte if we have a message and output is ready
  if (currentMsg != nullptr && output.availableForWrite() > 0) {
    if (currentPos < strlen(currentMsg)) {
      output.print(currentMsg[currentPos]);
      currentPos++;
    } else {
      // Message complete, reset for next message
      currentMsg = nullptr;
      currentPos = 0;
    }
  }
  
  output.nextByteOut();
}
