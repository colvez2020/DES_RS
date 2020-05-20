#include <OpenLamborghino.h>
#include <EEPROM.h>
#include <LEANTEC_ControlMotor.h>
#include "Control_main.h"
#include "Comunicacion.h"
#include "Sistema_luces.h"



//Control PID
int setpoint = 0;
int gyroSpeed = 80;
int base = 40;
float Kprop = 1.1;
float Kderiv = 0.9;
float Kinte = 0.1;
OpenLamborghino Segidor_DID(BOTON_PIN, BUZZER_PIN);
                            //14
ControlMotor Control_DID(13,5,11,12,9,10); // MotorDer1,MotorDer2,MotorIzq1,MotorIzq2,PWM_Derecho,PWM_Izquierdo
                                            // IN01     ,IN02     ,IN11     ,IN12     ,PWM00      ,PWM10


void Setup_Seguidor_linea(void)
{
    //Configurar Sensores
    //Segidor_DID.WaitBoton();
    Serial.println("Inicio_Calibracion_Linea");
    Segidor_DID.calibracion();
    //Segidor_DID.WaitBoton();
    delay(1000);
}

long Leer_sensor(void)
{
  return Segidor_DID.LineaNegra();
}

boolean Mod_Parametros_PID(void)
{
  int salida;
  
  Parametros_consola(&base,&Kprop,&Kderiv,&Kinte,&setpoint,&salida);
  long pos =  Segidor_DID.LineaNegra();
  Segidor_DID.PIDLambo(Kprop, Kderiv, Kinte);
  int Power = Segidor_DID.PID(pos, setpoint, gyroSpeed);
  Control_DID.Segidor_linea(base - Power, base + Power );
  if(salida==1)
    return false;
  return true;
}

void Ejecutar_seguidor_linea(void)
{
  long pos =  Segidor_DID.LineaNegra();
  int Power = Segidor_DID.PID(pos, setpoint, gyroSpeed);
  Control_DID.Segidor_linea(base - Power, base + Power );
}



void Control_Bluethoot(char Comando)
{
  switch(Comando) //donde opción es la variable a comparar
  {
    case 'A': //Bloque de instrucciones 1;
    //          for (fade=0;fade>-200;fade-50) 
    //            control.Motor(fade,0);
      Control_DID.Motor(-125,0);
    break;
    case 'D': //Bloque de instrucciones 1;
    //      for (fade=0;fade<150;fade+50) 
    //        control.Motor(fade,100);
      Control_DID.Motor(125,100);
    break;
    case 'I': //Bloque de instrucciones 1;
    //  for (fade=0;fade<150;fade+50) 
    //    control.Motor(fade,-100);
      Control_DID.Motor(125,-100);
    break;
    case 'B': //Bloque de instrucciones 1;
    // for (fade=0;fade<200;fade+50) 
    //   control.Motor(fade,0);              
      Control_DID.Motor(125,0); //OK
    break;
    case 'P': //Bloque de instrucciones 1;
      Control_DID.Motor(0,0);
    break;
    case 'O':
      Control_luces(ON_LAMP);
    break;
    case 'F':
      Control_luces(OFF_LAMP);
    break; 
     
    case 'G': //activo modificacion de parametros
      EEPROM.write(FLAG_PID_ADD, FLAG_PID_RESET);
    break; 
    case 'S': //activo rutina segidor
      EEPROM.write(FLAG_RS_ADD, FLAG_RS_OK);
    break; 
    case 'T': //desactivo rutina segidor
      EEPROM.write(FLAG_RS_ADD, FLAG_RS_RESET);
    break; 
     
    

   }
 }

void Contro_tableta(char comando)
{
  switch(comando) //donde opción es la variable a comparar
  {
    case 0x01:
      Control_luces(ON_LAMP);
    break;
    case 0x03:
      Control_luces(OFF_LAMP);
    break;
  }  
}


boolean Comando_valido(char a)
{
  char resultcmd=false;

  switch(a) //donde opción es la variable a comparar
  {
    case 'A': //Bloque de instrucciones 1;
    case 'D': //Bloque de instrucciones 1;
    case 'I': //Bloque de instrucciones 1;
    case 'B': //Bloque de instrucciones 1;
    case 'P': //Bloque de instrucciones 1;
    case 'O':
    case 'F':
      resultcmd=true;
    break;
  }
  return resultcmd; 
}
