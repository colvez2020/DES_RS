#include <EEPROM.h>
#include "QTRSensors.h"
#include "Control_main.h"

#include "Sistema_luces.h"
#include "Sistema_Usonic.h"
#include "Comunicacion.h"


//Opciones de configuracion
//#define CONFIG_AT
//#define TEST_SONIC
#define TEST_OPTIC

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
  
  #ifdef CONFIG_AT
  Setup_Comunicacion(MODO_AT);
  #endif

  #ifndef CONFIG_AT
  Setup_Comunicacion(MODO_NORMAL);
  #endif

  
  if(EEPROM.read(FLAG_OPTIC_ADD)!=FLAG_OPTIC_OK)
  {
    do
    {
      Setup_Seguidor_linea(0);
      Read_serial_Tableta(&option_Tableta);
      if(option_Tableta=='S')
      {
        EEPROM.write(FLAG_OPTIC_ADD, FLAG_OPTIC_OK);
        break;
      }
    }while(1);
  }
  else
  {
    Setup_Seguidor_linea(6);
  }
    
  //Setup_luces();
  //Setup_Usonic();  
  lastPingMillis = millis();                // Punto de partida del programa
  
}
void loop() 
{
  
  #ifdef CONFIG_AT
  do{
    Ejecutar_modoAT();
  }while(1);
  
  #endif
  //Ciclo infinito leer_OPTIC
  #ifdef TEST_OPTIC
  do{
    Leer_sensor();
    delay(100);
  }while(1);
  #endif

  //Ciclo infinito probar Ultrasonicos.
  #ifdef TEST_SONIC
  do{
    Read_Usonic(USONIC_TEST);
  }while(1);
  #endif
  
  //Ciclo infinito Calibrar PID
  if(EEPROM.read(FLAG_PID_ADD)==FLAG_PID_RESET)
  {
    do{}while(Mod_Parametros_PID());
    EEPROM.write(FLAG_PID_ADD, FLAG_PID_OK);
  }

  //Activar Robot seguidor
  //No se toma en cuenta los sensores sonicos.
  if(EEPROM.read(FLAG_RS_ADD)==FLAG_RS_OK)
  {
    do{
      Ejecutar_seguidor_linea();
      Read_serial_bluethoot(&option_bluethoot);
      Control_Bluethoot(option_bluethoot);
      if (millis() - lastPingled >= LedDelay)
      {
        Control_led();
        lastPingled = millis();
      }
    }while(EEPROM.read(FLAG_RS_ADD)!=FLAG_RS_RESET);
  }

  
  //Leectura de SOnicos, solo aplica cuando se mueve por bluethoot
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
      Control_Bluethoot(option_bluethoot);
  }
    
  if (Read_serial_Tableta(&option_Tableta))
    Contro_tableta(option_Tableta);
}
