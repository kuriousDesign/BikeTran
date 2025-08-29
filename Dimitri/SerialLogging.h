// SerialLogging.h (Updated Header without RingBuf library)
#ifndef SERIAL_LOGGING_H
#define SERIAL_LOGGING_H

#include <Arduino.h>      // Add this for core declarations (HardwareSerial, Serial, noInterrupts, etc.)
#include <BufferedOutput.h>  // From SafeString library for non-blocking output
#include <stdarg.h>       // For variadic args in add/info/warn/error
#include <string.h>       // For strlen, memmove, memcpy
#include <avr/interrupt.h> // For noInterrupts()/interrupts() (redundant with Arduino.h, but harmless)

class SerialLogging {
private:
  static const size_t BYTE_BUFFER_SIZE = 2048/4;  // Total bytes in custom ring buffer (power of 2 for modulo efficiency)
  static uint8_t queue[BYTE_BUFFER_SIZE];       // Static byte array for the ring buffer
  static volatile size_t head;                  // Read index (where to pop from)
  static volatile size_t tail;                  // Write index (where to push to)
  static const size_t BUFFER_SIZE = 512 + 4;    // Output buffer size (+4 for dropMark)
  static uint8_t output_BUFFER[BUFFER_SIZE];    // Manual static buffer for BufferedOutput
  static BufferedOutput output;                 // Non-blocking output buffer
  static const size_t MAX_MSG_SIZE = 128/2;       // Max chars per formatted message (including null terminator)

  // Internal helpers for custom ring buffer
  static bool isFull();                         // Check if buffer is full
  static bool isEmpty();                        // Check if buffer is empty
  static bool pushByte(uint8_t byte);           // Push a single byte (returns true on success)
  static bool popByte(uint8_t& byte);           // Pop a single byte (returns true on success)

public:
  // Initialize: Call once in setup()
  static void init();

  // Add a message without newline
  static void add(const char* fmt, ...);

  // Add a message with newline (info level)
  static void info(const char* fmt, ...);

  // Add a warning message with "WARNING: " prefix and newline
  static void warn(const char* fmt, ...);

  // Add an error message with "ERROR: " prefix and newline
  static void error(const char* fmt, ...);

  // Add msg chunk of type byte*
  static void publishData(byte* &data, size_t len, const char* header);

  // Process: Dequeue bytes to output buffer and release to Serial (call in loop())
  static void process();
};

#endif