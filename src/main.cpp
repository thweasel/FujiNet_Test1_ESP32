#include <Arduino.h>
#include "consoleDebug.h"
#include "ESPHALDriverESP32-WROVER.h"


#define LED_BUILTIN 2
uint8_t busState[4] = {0,0,0,0};


void setup() {
  //Serial.begin(115000);
  Serial.begin(9600);
  Serial.println("Running.");
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  //Setup_ESPHALDriver();
  // put your setup code here, to run once:
  
}

uint8_t data = 0x00;//B10000001; //0x01;
uint8_t control = 0x81;
uint16_t address = 0x0000;
uint8_t loAddr = 0; 
uint8_t hiAddr = 0;

uint64_t error =0;
uint64_t pass =0;

void loop()
{
  // Blinky code for enable to test if running
  
  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);
  
  // put your main code here, to run repeatedly:


  //consoleShowAddrData("write: ",address,data,HEX);
  WriteDataBUSOperation(data, control, address);
  


  busState = getBUSstate();
  
 // consoleShowBusPacketBuffer("busState-hex", busState, HEX);
 // consoleShowBusPacketBuffer("busState-bin", busState, BIN);
 
  uint8_t loAddr = (uint8_t)(address);
  uint8_t hiAddr = (uint8_t)(address >> 8);


  if ((data != busState[3]) || (loAddr != busState[1]) || (hiAddr != busState[0]))
  {

    if (data != busState[3])
    {
      Serial.print(" [!!] data error : ");
      Serial.print(data & busState[3], BIN);
    }
    if (loAddr != busState[1])
    {
      Serial.print(" [!!] addrLOW error : ");
      Serial.print(loAddr & busState[1], BIN);
    }
    if (hiAddr != busState[0])
    {
      Serial.print(" [!!] addrHIGH error : ");
      Serial.print(hiAddr & busState[0], BIN);
    }
    error++;
    Serial.print("\n pass: ");
    Serial.print(pass);
    Serial.print(" error: ");
    Serial.println(error);
  }
  else{
    pass++;
    if (pass % 1000 == 0)
    {
      Serial.print("\n pass: ");
      Serial.print(pass);
      Serial.print(" error: ");
      Serial.println(error);
    }
  }
  data++;
  address = address + 0x0101;
  
}



