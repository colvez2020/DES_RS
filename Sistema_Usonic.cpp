#include <Arduino.h>
#include "Sistema_Usonic.h"

#define soundSpeed 343.0                      // Speed of sound in m/s (343m/s at 20°C with dry air)
#define tSENS 6                               // Total number of sensors
#define echoPin 2                             // Pin to collect echo signal --> This must be any interrupt pin INT0, pin tarjeta #2

//Ultrasonic
int triggerPins[tSENS] = {3, 4, 5, 6, 7, 8};  // Trigger pin for each sensor
volatile unsigned long startEcho;             // Place to store start time (interrupt)
volatile unsigned long stopEcho;              // Place to store stop time (interrupt)

float distancesCm[tSENS];                     // Distances in cm


/****************************************************************
      INTERRUPT handling (keep as fast(=short) as possible)
****************************************************************/

void ISR_ECHO()
{
  // For ATmega328(P) chips, INT0 as at pin PD2
  byte pinRead = PIND >> 2 & B0001;  // Read state of interrupt pin
  if (pinRead)  {
    // State = HIGH -> Remember start time
    startEcho = micros();
  }
  else {
    // State is LOW -> Remember stop time
    stopEcho = micros();
  }
}







void Setup_Usonic(void)
{
    for (int i = 0; i < tSENS; i++) 
  {
    pinMode(triggerPins[i], OUTPUT);   // Set trigger pin x as OUTPUT
  }
  pinMode(echoPin, INPUT);                  // Set echo pin as INPUT (do not use pullup when using diodes !)
  //Configurar Interrupcion Sonic
  attachInterrupt(0, ISR_ECHO, CHANGE );    // Set interrupt call for echo pin
}

float Read_Usonic(char MODO)
{
  float respuesta=doMeasurement();
  
  if(MODO==USONIC_TEST)
  {
    Serial.println( String(distancesCm[0]) + " - " 
              + String(distancesCm[1]) + " - " 
              + String(distancesCm[2]) + " - " 
              + String(distancesCm[3]) + " - "
              + String(distancesCm[4]) + " - " 
              + String(distancesCm[5]));
    Serial.print( "Minima a:");
    Serial.println(respuesta);
  }
  return respuesta;
}


float doMeasurement(void)
{
  float temp=200;
  // First read will be bad (or 0)
  unsigned long startWait;
  unsigned long timeout = 50; // 50ms is the max amount of time we'll wait (23ms = 4m = out of range)
  // For each sensor
  for (int s = 0; s < tSENS; s++)
  {
    startEcho = 0;  // Reset start
    stopEcho = 0;   // Reset stop time
    digitalWrite(triggerPins[s], HIGH);    // HIGH pulse for at least 10µs
    delayMicroseconds(10);
    digitalWrite(triggerPins[s], LOW);    // Set LOW again
    startWait = millis();   // remember time we started to wait for the echo signal
    while (startEcho == 0 or stopEcho == 0) {
      // Keep looping unill start and stoptime is not 0 OR a timeout occurs
      if (startWait + timeout < millis())
      {
        // This will result in a distance of 0
        startEcho = 0;
        stopEcho = 0;
        break;  // exit the while loop
      }
    }
    noInterrupts();   // cli()
    distancesCm[s] = (stopEcho - startEcho) * (float)soundSpeed / 1000.0 / 10.0 / 2.0;   // in cm
    if(temp>distancesCm[s]&&distancesCm[s]!=0)
      temp=distancesCm[s];
    interrupts();   // sei();
  }
  return temp;
}
