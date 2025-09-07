# Dimitri
This is the firmware to control the gear shifting for the demo model. 


# TESTING PROCEDURE

PRIMARY - 6.5mm width per gear
SECONDARY - 6.0mm width per gear




## SSH into joe's dell
ssh joeke@10.244.0.10
pw: Samantha12

## Serial Interfacing Specification
The serial interface on the arduino mega will provide the following functionality. Please note that all commands need to be terminated with carriage return '\n' character

### Status Query
when queried, this will spit back status info like current gear, current mode (state) of the controller

Command to send: T0

Command reply: stream of bytes that is captured as an array, see below for parsing rules

        i = 0
        self.next_mode = int.from_bytes(bytes(self.byteArray[i:i+3]), 'little',signed=True)
        is_new_mode = self.check_new_mode()

        i += 4
        self.next_step = int.from_bytes(bytes(self.byteArray[i:i+3]), 'little',signed=True)
        is_new_step = self.check_new_step(is_new_mode)
        i += 4

        self.ioByte = bytes(self.byteArray[i:i+1])
        self.io = self.io.convertByteToIo(self.ioByte)
        i += 1

### Info Query
when queried, this will spit back info like warnings/errors

### Shift Requests
when sent, this will induce a gear change

#### Relative Gear
when sent, the shifter will shift up or down the position

R0<desiredGearChange(with sign)>

example: if you want to go up one gear, you would send 'R01'
example: if you wanted to go down two gears, you sould send 'R0-2'

#### Absolute Gear
when sent, the shifter will go to the gear provided

P0<desiredGearNumber>

example: if you want to go to gear 7, you would send 'P07'

#### Homing
when sent, the shifter will go into homing mode. If successfully completed the motor will finish in 1st Gear

P00

### Diagnostic Query
This will be used if you want to see the latest control info for the most recent gear change. If queried, this will provide a json string of data arrays.

Command to send: XO

Command reply: json string containing the following keys and data
    "targetError": int_16_t[]
    "cmd

## Dependency Libraries
ISerial (custom lib from FirmwareWithSerialInterface by KuriousDesign)
SafeString - by Matthew Ford (Library Manager)
LoopTimer - included with SafeString
jsonlib - by Justin Shaw (Library Manager)
ArduinoJson - by Benoit Blanchon 
Encoder - by Paul Stoffregen
TimerOne - Jesse Tane
DueTimer - Ivan?