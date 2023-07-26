#include <Arduino.h>
//#include "consoleDebug.h"

void consoleShowAddrData(const char *label, uint16_t addr, uint8_t data, uint8_t format)
{
    Serial.print(label);
    Serial.print(" : addr ");
    Serial.print(addr, format);
    Serial.print(", data ");
    Serial.println(data, format);
    return;
}

void consoleShowBusPacketBuffer(const char *label, uint8_t *BusPacketBuffer, uint8_t format)
{
    Serial.print(label);
    Serial.print(": BusPacketBuffer - [3] Data ");
    Serial.print(BusPacketBuffer[3], format);
    Serial.print(", [2] Control ");
    Serial.print(BusPacketBuffer[2], format);
    Serial.print(", [1] AddrL ");
    Serial.print(BusPacketBuffer[1], format);
    Serial.print(", [0] AddrH ");
    Serial.println(BusPacketBuffer[0], format);
    return;
}