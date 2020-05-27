#include <EEPROM.h>
#include "Mem_add.h"
#include "OpenLamborghino.h"
#include "LEANTEC_ControlMotor.h"
#include "Control_main.h"
#include "Comunicacion.h"
#include "Sistema_luces.h"

//Valores por defecto.
int Velocidad_base= 120; //Velovidad en linea recta
int setpoint = 0;    //Sobre la linea
int gyroSpeed = 250; //Accion de control maxima
int Estado_config=0;

float Kprop = 1.1;
float Kderiv = 0.9; 
float Kinte = 0.1;


OpenLamborghino Segidor_DID(BUZZER_PIN);
                            //14
ControlMotor Control_DID(13,5,11,12,9,10); // MotorDer1,MotorDer2,MotorIzq1,MotorIzq2,PWM_Derecho,PWM_Izquierdo
                                            // IN01     ,IN02     ,IN11     ,IN12     ,PWM00      ,PWM10


void Setup_Seguidor_linea(uint8_t modo)
{
    //Configurar Sensores
    //Segidor_DID.WaitBoton();

    if(modo==CALIBRA)
    {
      Write_serial_bluethoot_stream("Inicio_Calibracion_Linea");
      Segidor_DID.calibracion();
      //Actualiza la eeprom del PID con los valores x defecto
      //Segidor_DID.PIDLambo(Kprop, Kderiv, Kinte);
    }
    else
    {
      Segidor_DID.Getcalibracion();
      Getparametros_VB(&Velocidad_base);
    }
    //Segidor_DID.WaitBoton();
    delay(1000);
}

long Leer_sensor(void)
{
  return Segidor_DID.LineaNegra();
}

boolean Sintonizar_PID(void)
{

  //Para si hay algo x serial.
  Parametros_consola_blue(&Velocidad_base,&Kprop,&Kderiv,&Kinte,&Estado_config);
  if(Estado_config==1)
  {
    Segidor_DID.PIDLambo(Kprop, Kderiv, Kinte);
    Estado_config=2;
  }
  if(Estado_config==3)
  {
    Segidor_DID.PIDLambo(Kprop, Kderiv, Kinte);
    Setparametros_VB(Velocidad_base);
  }
  long pos =  Segidor_DID.LineaNegra();
  int Power = Segidor_DID.PID(pos, setpoint, gyroSpeed);
  Control_DID.Segidor_linea(Velocidad_base - Power, Velocidad_base + Power );
  
  if(Estado_config==4)
    return false;
  return true;
}


void Getparametros_VB(int* VB )
{
  EEPROM.get(VBGS_ADD,*VB);
  //EEPROM.get(VBGS_ADD+sizeof(int),*GS);
}

void Setparametros_VB(int VB )
{
  EEPROM.put(VBGS_ADD,VB);
  //EEPROM.put(VBGS_ADD+sizeof(int),GS);
}

void Ejecutar_seguidor_linea(void)
{
  long pos =  Segidor_DID.LineaNegra();
  int Power = Segidor_DID.PID(pos, setpoint, gyroSpeed);
  Control_DID.Segidor_linea(Velocidad_base - Power, Velocidad_base + Power );
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
