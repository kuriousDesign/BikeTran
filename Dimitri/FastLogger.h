#pragma once
#include <Arduino.h>
#include <stdarg.h>

#define MAX_MSG_SIZE 256
#define STX 0x02
#define ETX 0x03
#define NEWLINE 0x0A

class Logger {
public:
    static void info(const char* fmt, ...);
    static void warn(const char* fmt, ...);
    static void error(const char* fmt, ...);
    static void csv(const char* fmt);

    static void publishPacket(const byte* data, size_t len, uint8_t id);

    static void process();
    static void setDebug(bool state);

private:
    static constexpr size_t BUFFER_SIZE = 2048*2; // bumped up for binary traffic
    static char buffer[BUFFER_SIZE];
    static volatile size_t writeIndex;
    static volatile size_t readIndex;
    static bool _debug; 

    static void writeToBuffer(const char* msg);
    static void writeByte(uint8_t b);
    static void logMessage(const char* type, const char* fmt, va_list args);
};
