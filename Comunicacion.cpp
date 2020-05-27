#include <EEPROM.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Comunicacion.h"
#include "Control_Main.h"

//18
SoftwareSerial ComandoSerial(17,4); // RX, TX

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

void Parametros_consola(int* base,float* Kprop,float* Kderiv,float* Kinte, int* setpoint,int*salida) 
{

  if (ComandoSerial.available() > 0) 
  {

    tone(BUZZER_PIN, 1300, 100);
    *base = ComandoSerial.parseInt();
    *Kprop = ComandoSerial.parseInt() / 10.0;
    *Kderiv = ComandoSerial.parseInt() / 10.0;
    *Kinte = ComandoSerial.parseInt() / 10.0;
    *setpoint = ComandoSerial.parseInt();
    *salida= ComandoSerial.parseInt();
    
    if (ComandoSerial.readString() == ("\n")) 
    {
      ComandoSerial.flush();
    }
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

void Ejecutar_modoAT(void)
{
if (ComandoSerial.available())    // read from HC-05 and send to Arduino Serial Monitor
      Serial.write(ComandoSerial.read());
 if (Serial.available())     // Keep reading from Arduino Serial Monitor and send to HC-05
     ComandoSerial.write(Serial.read());
  
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
