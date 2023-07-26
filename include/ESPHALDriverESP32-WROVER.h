#include <Arduino.h>
#include <SPI.h>

//
// Functions for the Hardware layer above

void Setup_ESPHALDriver(void);


void WriteDataBUSOperation(uint8_t Data, uint8_t Control, uint16_t Address);

uint8_t ReadDataBUSOperation(uint8_t Control, uint16_t Address);

uint8_t *getBUSstate(void);

void sendBusSignals(void);

void stopBusSignals(void);

//
// RIO ROM
void enableRIOProtection(void);
void disableRIOProtection(void);
void setROMbank(uint8_t rom);

void sendPulse(void);

void setESPHardlock(void);
void clearESPHardlock(void);
boolean getZ80HardlockState(void);

boolean getINT(void);
bool resetESPHardlock(void);
