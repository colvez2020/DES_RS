#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Comunicacion.h"

SoftwareSerial ComandoSerial(17,18); // RX, TX

void Setup_Comunicacion(char MODO)
{
  if(MODO==MODO_NORMAL)
    ComandoSerial.begin(9600);
  else
    ComandoSerial.begin(38400);

  while (!ComandoSerial) 
  {
    ; // Wait for Serial
  }
  
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for Serial
  }
  
}

boolean Read_serial_bluethoot(char* DATA)
{
  if (ComandoSerial.available()>0)
  {
    *DATA= ComandoSerial.read();
    return true;
  }
  return false;
}

void Write_serial_bluethoot(char DATA)
{
  ComandoSerial.print(DATA);
}


boolean Read_serial_Tableta(char* DATA)
{
  if (Serial.available()>0)
  {
    *DATA= Serial.read();
    return true;
  }
  return false; 
}
