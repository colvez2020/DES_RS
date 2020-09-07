//#include <EEPROM.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Comunicacion.h"
#include "Control_Main.h"

//18
//SoftwareSerial ComandoSerial(17,18); // RX, TX
HardwareSerial & ComandoSerial = Serial1;

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

void Parametros_consola_blue(int* base,float* Kprop,float* Kderiv,float* Kinte,int*orden) 
{

  if (ComandoSerial.available() > 0) 
  {

    tone(BUZZER_PIN, 1300, 100);
    *base = ComandoSerial.parseInt();
    *Kprop = ComandoSerial.parseFloat();
    *Kderiv = ComandoSerial.parseFloat();
    *Kinte = ComandoSerial.parseFloat();
    //*setpoint = ComandoSerial.parseInt();
    *orden= ComandoSerial.parseInt();
    
    if (ComandoSerial.readString() == ("\n")) 
    {
      ComandoSerial.flush();
    }
  }
}


boolean Read_serial_bluethoot(char* DATA,char MODO)
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

void Write_serial_bluethoot_stream_nl(String DATA)
{
  ComandoSerial.print(DATA);
  ComandoSerial.println();
}

void Write_serial_bluethoot_stream(String DATA)
{
  ComandoSerial.print(DATA);
}

void Write_serial_bluethoot_int(int DATA)
{
  ComandoSerial.print(DATA);
 // ComandoSerial.println();
}



void Write_serial_bluethoot_uint16(uint16_t DATA)
{
  ComandoSerial.print(DATA);
 // ComandoSerial.println();
}

void Write_serial_bluethoot_double(double DATA)
{
  ComandoSerial.print(DATA);
 // ComandoSerial.println();
}

void Write_serial_bluethoot_float(float DATA)
{
  ComandoSerial.print(DATA);
 // ComandoSerial.println();
}

void Write_serial_bluethoot_long(long DATA)
{
  ComandoSerial.print(DATA);
}

void Write_serial_bluethoot_nl(void)
{
  ComandoSerial.println();
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
