#include "Control_main.h"
#include "Sistema_luces.h"
#include "Sistema_Usonic.h"
#include "Comunicacion.h"

//Led
#define LedDelay 500
unsigned long lastPingled;

//Ultrasonic
#define pingDelay 500                          // How many milliseconds between each measurement (ping) ; best to keep > 5ms
unsigned long lastPingMillis;
float distancia_result;

//Comunicaciones
char option_bluethoot;
char option_Tableta;

void setup() 
{
  Setup_Comunicacion(MODO_NORMAL);
  Setup_Usonic();
  Setup_Seguidor_linea(CALIBRA);
  Setup_luces();  
  lastPingMillis = millis();                // Punto de partida del programa
}
void loop() 
{
  //Calibrar PID
  Mod_Parametros_PID();
  
  ///////////////////////
  //Sencesado de proximidad
  if (millis() - lastPingMillis >= pingDelay)
  {
    distancia_result=Read_Usonic(USONIC_NORMAL);
    if(distancia_result<15)
    {
      option_bluethoot='P';
      Control_Bluethoot(option_bluethoot);
    }         
    Write_serial_bluethoot(distancia_result);              
    Write_serial_bluethoot(",");              
    lastPingMillis = millis();
  }

  //Manejo de luces
  if (millis() - lastPingled >= LedDelay)
  {
    Control_led();
    lastPingled = millis();
  }

  //Control Motor bluethoot
  if (Read_serial_bluethoot(&option_bluethoot))
  {
    if(Comando_valido(option_bluethoot)==true)
    {
      Control_Bluethoot(option_bluethoot);
    }
  }
  
    
  if (Read_serial_Tableta(&option_Tableta))
  {
    //ComandoSerial.println(option);
    Contro_tableta(option_Tableta);
  }

}
