#include "FastLogger.h"
//#include "Packets.h"
//#include "Packets.h"

char Logger::buffer[Logger::BUFFER_SIZE] = {0};
volatile size_t Logger::writeIndex = 0;
volatile size_t Logger::readIndex = 0;
bool Logger::_debug = false;

void Logger::writeByte(uint8_t b) {
    noInterrupts();
    size_t nextIndex = (writeIndex + 1) % BUFFER_SIZE;

    // Prevent overwriting unread data
    if (nextIndex != readIndex) {
        buffer[writeIndex] = b;
        writeIndex = nextIndex;
    }
    interrupts();
}

void Logger::writeToBuffer(const char* msg) {
    while (*msg) {
        writeByte(static_cast<uint8_t>(*msg++));
    }
}

void Logger::logMessage(const char* type, const char* fmt, va_list args) {
    if (!_debug) return;

    char msg[MAX_MSG_SIZE];
    size_t prefixLen = 0;

    if (type) {
        prefixLen = strlen(type);
        if (prefixLen >= MAX_MSG_SIZE) prefixLen = MAX_MSG_SIZE - 1;
        memcpy(msg, type, prefixLen);
    }

    vsnprintf(msg + prefixLen, MAX_MSG_SIZE - prefixLen, fmt, args);
    msg[MAX_MSG_SIZE - 1] = 0;

    writeToBuffer(msg);
    writeByte('\n');
}

void Logger::info(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    logMessage(nullptr, fmt, args);
    va_end(args);
}

void Logger::warn(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    logMessage("WARN: ", fmt, args);
    va_end(args);
}

void Logger::error(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    logMessage("ERROR: ", fmt, args);
    va_end(args);
}

void Logger::publishPacket(const byte* data, size_t len, uint8_t packetId) {
    if (len > MAX_MSG_SIZE) {
        warn("publishPacket: Data length %u exceeds max %u", len, MAX_MSG_SIZE);
        return;
    }

    writeByte(STX);
    writeByte(static_cast<uint8_t>(len));
    writeByte(packetId);

    for (size_t i = 0; i < len; i++) {
        writeByte(data[i]);
    }

    writeByte(ETX);
    writeByte(NEWLINE);
}

void Logger::csv(const char* fmt) {
    char msg[MAX_MSG_SIZE];
    snprintf(msg, MAX_MSG_SIZE, fmt);
    writeToBuffer(msg);
    writeByte('\n');
}

void Logger::process() {
    noInterrupts();
    while (readIndex != writeIndex) {
        uint8_t b = buffer[readIndex];
        readIndex = (readIndex + 1) % BUFFER_SIZE;
        interrupts(); // allow ISR during Serial.write
        Serial.write(b);
        noInterrupts();
    }
    interrupts();
}

void Logger::setDebug(bool state) {
    _debug = state;
}
