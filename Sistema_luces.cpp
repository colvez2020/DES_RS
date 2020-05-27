#include <Arduino.h>
#include "Sistema_luces.h"
//Led
#define LedPin 15
#define LedDelay 500                          // How many milliseconds between each measurement (ping) ; best to keep > 5ms

//Lampara
#define LampPin    16
boolean Alterna_flag=true;


void Setup_luces(void)
{
  pinMode(LedPin, OUTPUT);                  // Set LedPin para prender los led azules
  pinMode(LampPin, OUTPUT);                 // Set LampPin para activar el relay de las lamparas
  digitalWrite(LampPin,LOW);
}

void Control_luces(char Acccion)
{
  switch(Acccion) //donde opción es la variable a comparar
  {
    case ON_LAMP:
    digitalWrite(LampPin,HIGH);
    break;
    case OFF_LAMP:
    digitalWrite(LampPin,LOW);
    break;  
   }
}

void Alternar_led(void)
{
  // Serial.println("--- led started ---");
  if(Alterna_flag==true)
  {
    digitalWrite(LedPin, HIGH);    // HIGH pulse for at least 10µs
    Alterna_flag=false;
  }
  else
  {
    digitalWrite(LedPin, LOW);    // HIGH pulse for at least 10µs
    Alterna_flag=true;
  }
}


void Control_led(char Accion)
{
  switch(Accion) //donde opción es la variable a comparar
  {
    case OFF_LAMP:
    digitalWrite(LedPin,LOW);
    break;
    case ON_LAMP:
    digitalWrite(LedPin,HIGH);
    break;  
   }
}