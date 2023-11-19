//Bibliotheken
#include <SparkFun_TB6612.h>
#include "BluetoothSerial.h"
#define _TIMERINTERRUPT_LOGLEVEL_     4
#include <ESP32TimerInterrupt.h> 
#define TIMER0_INTERVAL_MS        100
#define TIMER0_DURATION_MS        10
BluetoothSerial SerialBT;

//Variablen Bluetooth
String device_name = "Linienfolger";
int On = 0;

//Variablen Sensormapping

int customMap(int x, int in_min, int in_max, int out_min, int out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//globale Variablen
int sensorMin1 = 3600; //Wert, wenn Sensor1 "schwarz" erkennt
int sensorMax1 = 3800; //Wert, wenn Sensor1 "weiß" erkennt
int sensorMin2 = 3600; //Wert, wenn Sensor2 "schwarz" erkennt
int sensorMax2 = 3800; //Wert, wenn Sensor2 "weiß" erkennt
int sensorMin3 = 3600; //Wert, wenn Sensor3 "schwarz" erkennt
int sensorMax3 = 3800; //Wert, wenn Sensor3 "weiß" erkennt
int sensorMin4 = 3600; //Wert, wenn Sensor4 "schwarz" erkennt
int sensorMax4 = 3800; //Wert, wenn Sensor4 "weiß" erkennt
int sensorMin5 = 3600; //Wert, wenn Sensor5 "schwarz" erkennt
int sensorMax5 = 3800; //Wert, wenn Sensor5 "weiß" erkennt



//Kalibrierte Sensorwerte
int sensorCal1 = 0; 
int sensorCal2 = 0;
int sensorCal3 = 0;
int sensorCal4 = 0;
int sensorCal5 = 0;


//Variablen PID

int P;
int I;
int D;
int error;
int preverror;
float Kp = 0.45 ;
float Ki = 0.0 ;
float Kd = 0.05;
float u;
float x;


//Variablen Error-Ausgabe

int error_feedback[100];
int ie;

// Mutex für kritische Abschnitte
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//Motordrehzahlen
int nA, nB; //Last, errechnet
int ngA = 60; //Grundlast
int ngB = 60;
int nmaxApos = 80; //maximale Last
int nmaxBpos = 80;
int nmaxAneg = -30;
int nmaxBneg = -30; 


//Motordriver-Pins
#define AIN1 14
#define BIN1 12
#define AIN2 2
#define BIN2 4
#define PWMA 27
#define PWMB 26
#define STBY 13

//Anpassung an Motorfunktionen 
const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup()
{
  pinMode(25, INPUT);                   //Input-Pins der Sensoren
  pinMode(33, INPUT);
  pinMode(32, INPUT);
  pinMode(35, INPUT);
  pinMode(34, INPUT);
  Serial.begin(115200);
  SerialBT.begin("Linienfolger");
  void IRAM_ATTR onTimer();     
  ESP32TimerInterrupt timer(0);                   // Interrupt definieren
  //timer.attachInterruptInterval(100000, onTimer); // Interval in Mikrosekunden
}



void loop()
{
while (millis() < 10000)
{
  //Aufruf Kalibrierung, wenn Zykluszeit < 10s
  calibration();
}
while (millis() > 10000)
{
  //Einlesen der Bluetoothdaten
  if (SerialBT.available()) 
  // Eintreten in kritischen Abschnitt
  portENTER_CRITICAL(&mux);

  // Deaktiviere den Timer-Interrupt
  //timer0.detachInterrupt();
  {
    String command = SerialBT.readString();
    
    // Überprüfe, ob der empfangene String Kp, Ki oder Kd enthält
    if (command.startsWith("Kp:")) 
    {
      Kp = command.substring(3).toFloat();                     //Umrechnen von String zu Float
      Serial.println("P-Wert empfangen: " + String(Kp));
    } 
    else if (command.startsWith("Ki:")) 
    {
      Ki = command.substring(3).toFloat();
      Serial.println("I-Wert empfangen: " + String(Ki));
    } 
    else if (command.startsWith("Kd:")) 
    {
      Kd = command.substring(3).toFloat();
      Serial.println("D-Wert empfangen: " + String(Kd));
    }
    else if (command.startsWith("On:")) 
    {
      On = command.substring(3).toInt();
      Serial.println("On/Off empfangen: " + String(On));
    }
    // Aktiviere den Timer-Interrupt wieder
    //timer0.attachInterrupt();

    // Verlassen des kritischen Abschnitts
    portEXIT_CRITICAL(&mux);
  }
  //Aufruf PID, wenn Zykluszeit > 10s
    pid();
 
  

}
  
}

//******************************************END MAIN************************************************************************
//Funktion PID
void pid()
{
  
  //Linearisierung Sensorsignal
  sensorCal1 = customMap(analogRead(25), sensorMin1, sensorMax1, 100, 1000);
  sensorCal2 = customMap(analogRead(33), sensorMin2, sensorMax2, 100, 1000);
  sensorCal3 = customMap(analogRead(32), sensorMin3, sensorMax3, 100, 1000);
  sensorCal4 = customMap(analogRead(35), sensorMin4, sensorMax4, 100, 1000);
  sensorCal5 = customMap(analogRead(34), sensorMin5, sensorMax5, 100, 1000);

  //Position berechenen
  int sumCal = sensorCal1 + sensorCal2 + sensorCal3 + sensorCal4 + sensorCal5;
  x = (sensorCal1 * 0 + sensorCal2 * 1000 + sensorCal3 * 2000 + sensorCal4 * 3000 + sensorCal5 * 4000) / sumCal;


  //Fehler berechnen
  error = x-2000;

  //PID
  P = error;

  I = I + error;                  //Aufsummierung in jedem Zyklus, nicht benutzen und ändern! Tn fehlt

  D = error - preverror;          //Tv integrieren

  u = ((Kp * P) + (Ki * I) + (Kd * D));  

  preverror = error;

  //I-Regler 0 setzen, wenn sich das Vorzeichen des Fehlers ändert

 int i;
 i++;
 if (i=5)
 {
  I=0;
 }

  //Motordrehzahl berechnen

 if (On == 1)
 {
  nA = ngA - u;
  nB = ngB + u;
 }
 else
 {
  nA = 0;
  nB = 0;
 }


  //Motordrehzahl begrenzen
  nA = constrain(nA, nmaxAneg, nmaxApos);
  nB = constrain(nB, nmaxBneg, nmaxBpos);


  //Motoransteuerung
  motor1.drive(nA,1);
  motor2.drive(nB,1);




}



//Funktion Kalibrierung, zum Kalibrieren Sensoren mehrmals per Hand über die Linie bewegen
void calibration() 
{
  unsigned long calibrationStartTime = millis();
  while (millis() - calibrationStartTime < 10000) {
    //motor1.drive(50);
    //motor2.drive(-50);
    calibrateSensor(25, sensorMin1, sensorMax1);
    calibrateSensor(33, sensorMin2, sensorMax2);
    calibrateSensor(32, sensorMin3, sensorMax3);
    calibrateSensor(35, sensorMin4, sensorMax4);
    calibrateSensor(34, sensorMin5, sensorMax5);
  }
}

//Unterfunktion für die Zuordnung der Min/Max-Werte
void calibrateSensor(int pin, int& sensorMin, int& sensorMax) {
  int sensorValue = analogRead(pin);
  if (sensorValue > sensorMax) {
    sensorMax = sensorValue;
  }
  if (sensorValue < sensorMin) {
    sensorMin = sensorValue;
  }
}

void IRAM_ATTR onTimer() {
  // Eintreten in kritischen Abschnitt
  portENTER_CRITICAL_ISR(&mux);

  // Füge den aktuellen Wert von "error" zum Array hinzu
  error_feedback[ie] = error;

  // Inkrementiere den Index und setze ihn auf 0, wenn das Array voll ist
  ie = (ie + 1) % 100;

  // Verlassen des kritischen Abschnitts
  portEXIT_CRITICAL_ISR(&mux);

}