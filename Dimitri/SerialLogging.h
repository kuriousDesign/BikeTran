#ifndef SERIAL_LOGGING_H
#define SERIAL_LOGGING_H

#include <RingBuf.h>      // Interrupt-safe ring buffer
#include <BufferedOutput.h>  // From SafeString library
#include <stdarg.h>       // For variadic args in add/addLine
#include <string.h>       // For strncat in addLine

class SerialLogging {
private:
  static RingBuf* queue;              // Queue for messages
  static const size_t BUFFER_SIZE = 512 + 4;  // Output buffer size (+4 for dropMark, per library)
  static uint8_t output_BUFFER[BUFFER_SIZE];  // Manually declared static buffer array
  static BufferedOutput output;       // Non-blocking output buffer
  static const size_t MAX_MSG_SIZE = 80;  // Max chars per message (accounts for null terminator; leave room for '\n')
  static const size_t QUEUE_LENGTH = 10;   // Max messages in queue

public:
  // Initialize: Call once in setup()
  static void init();

  // Add a message to the queue without newline (printf-style, safe from ISR/main)
  static void add(const char* fmt, ...);

  // Add a message to the queue with newline appended (printf-style, safe from ISR/main)
  static void info(const char* fmt, ...);

  // Warning: Add a warning message to the queue with newline appended (printf-style, safe from ISR/main)
  static void warn(const char* fmt, ...);

  // Error: Add an error message to the queue with newline appended (printf-style, safe from ISR/main)
  static void error(const char* fmt, ...);

  // Process: Dequeue messages to output buffer and release bytes (call in loop())
  static void process();
};

#endif