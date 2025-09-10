#pragma once

#include "Cfg.h"

#define MAX_PACKET_SIZE 256 //bytes, max size of packet to send over serial


enum Packets : uint8_t
{
    DIMITRI = 0,
    CMD = 1,
    ERROR_PACKET = 3,
};

const uint8_t DIMITRI_DATA_SIZE = 6;                                          // 2 bytes for loopStep and 1 byte for operation mode and 1 byte for inputs and 2 bytes for clutchDeviceState
const uint8_t DIMITRI_PACKET_SIZE = NUM_MOTORS * MOTOR_DATA_SIZE + DIMITRI_DATA_SIZE; // 2 bytes for loopStep and 1 byte for operation mode


uint8_t getPacketSize(uint8_t packetId)
{
    switch (packetId)
    {
    case Packets::DIMITRI:
        return DIMITRI_PACKET_SIZE;
    case Packets::CMD:
        return sizeof(CmdData);
    case Packets::ERROR_PACKET:
        return 0;
    default:
        return 0; // Unknown packet ID
    }
}   