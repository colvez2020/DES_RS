#include <OpenLamborghino.h>
#include <LEANTEC_ControlMotor.h>
#include "Control_main.h"
#include "Sistema_luces.h"



//Segidor linea
#define BOTON      12
#define BUZZER     10


//Control PID
int setpoint = 0;
int gyroSpeed = 80;
int base = 40;
float Kprop = 1.1;
float Kderiv = 0.9;
float Kinte = 0.1;
OpenLamborghino Segidor_DID(BOTON, BUZZER);

ControlMotor Control_DID(13,14,11,12,9,10); // MotorDer1,MotorDer2,MotorIzq1,MotorIzq2,PWM_Derecho,PWM_Izquierdo
                                            // IN01     ,IN02     ,IN11     ,IN12     ,PWM00      ,PWM10


void Setup_Seguidor_linea(char Modo)
{
  if(Modo==CALIBRA)
  {
    //Configurar Sensores
    Segidor_DID.WaitBoton();
    Segidor_DID.calibracion();
    Segidor_DID.WaitBoton();
    delay(1000);
  }
}

void Mod_Parametros_PID(void)
{
  Parametros_consola();
  int pos =  Segidor_DID.LineaNegra();
  Segidor_DID.PIDLambo(Kprop, Kderiv, Kinte);
  int Power = Segidor_DID.PID(pos, setpoint, gyroSpeed);
  Control_DID.Segidor_linea(base - Power, base + Power );
}

void Ejecutar_seguidor_linea(void)
{
  int pos =  Segidor_DID.LineaNegra();
  int Power = Segidor_DID.PID(pos, setpoint, gyroSpeed);
  Control_DID.Segidor_linea(base - Power, base + Power );
}

void Parametros_consola(void) 
{

  if (Serial.available() > 0) {

    tone(BUZZER, 1300, 100);
    base = Serial.parseInt();
    Kprop = Serial.parseInt() / 10.0;
    Kderiv = Serial.parseInt() / 10.0;
    Kinte = Serial.parseInt() / 10.0;
    setpoint = Serial.parseInt();

    if (Serial.readString() == ('\n')) {
      Serial.flush();
    }
  }
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
