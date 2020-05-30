#include <EEPROM.h>

#include "Mem_add.h"
#include "QTRSensors.h"
#include "Control_main.h"

#include "Sistema_luces.h"
#include "Sistema_Usonic.h"
#include "Comunicacion.h"


//Opciones de configuracion
//#define CONFIG_AT
//#define TEST_SONIC
//#define TEST_OPTIC

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
  DF,MDVF,MDF
  Setup_Comunicacion(MODO_AT);
  #endif

  #ifndef CONFIG_AT
  Setup_Comunicacion(MODO_NORMAL);
  #endif
  //Mensaje de Inicio.
  Write_serial_bluethoot_stream_nl("INICIO");
  
  if(EEPROM.read(FLAG_OPTIC_ADD)!=FLAG_OPTIC_OK)
  {
    Control_led(ON_LAMP);
    Write_serial_bluethoot_stream_nl("CALIBRAR_OPTICOS?");
    do
    {
      Read_serial_bluethoot(&option_bluethoot,1);
      if(option_bluethoot=='S')
      {
        Write_serial_bluethoot_stream_nl("EJECUTANDO");
        Setup_Seguidor_linea(CALIBRA);
      }
      if(option_bluethoot=='N')
      {
        EEPROM.write(FLAG_OPTIC_ADD, FLAG_OPTIC_OK);
        break;
      }
    }while(1);
  }
  else
  {
    Write_serial_bluethoot_stream_nl("USANDO_VALORES_CAL");
    Setup_Seguidor_linea(NO_CALIBRA);
  }
    
  //Setup_luces();
  //Setup_Usonic();  
  lastPingMillis = millis();                // Punto de partida del programa
  
}

void loop() 
{
  
  #ifdef CONFIG_AT
  Write_serial_bluethoot_stream_nl("CONFIG_AT");
  do{

    Ejecutar_modoAT();
  }while(1);
  
  #endif
  //Ciclo infinito leer_OPTIC
  #ifdef TEST_OPTIC
  Write_serial_bluethoot_stream_nl("TEST_OPTIC");
  do{
    Control_led(ON_LAMP);
    Leer_sensor();
    delay(500);
  }while(1);
  #endif

  //Ciclo infinito probar Ultrasonicos.
  #ifdef TEST_SONIC
  Write_serial_bluethoot_stream_nl("TEST_SONIC");
  do{
    Read_Usonic(USONIC_TEST);
  }while(1);
  #endif
  
  //Ciclo infinito Calibrar PID en caliente
  if(EEPROM.read(FLAG_PID_ADD)==FLAG_PID_RESET)
  {
    Write_serial_bluethoot_stream_nl("CAL_PID");
    do{}while(Sintonizar_PID());
    EEPROM.write(FLAG_PID_ADD, FLAG_PID_OK);
  }

  //Activar Robot seguidor
  //No se toma en cuenta los sensores sonicos.
  if(EEPROM.read(FLAG_RS_ADD)==FLAG_RS_OK)
  {
    Write_serial_bluethoot_stream_nl("INICIA RUM RS");
    do{
      Ejecutar_seguidor_linea();
      Read_serial_bluethoot(&option_bluethoot,0);
      Control_Bluethoot(option_bluethoot);
      
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
    Write_serial_bluethoot_float(distancia_result);      
    Write_serial_bluethoot(',');              
    lastPingMillis = millis();
  }

  //Manejo de luces
  if (millis() - lastPingled >= LedDelay)
  {
    Alternar_led();
    lastPingled = millis();
  }

  //Control Motor bluethoot
  if (Read_serial_bluethoot(&option_bluethoot,0))
  {
    if(Comando_valido(option_bluethoot)==true)
      Control_Bluethoot(option_bluethoot);
  }
    
  if (Read_serial_Tableta(&option_Tableta))
    Contro_tableta(option_Tableta);
}
