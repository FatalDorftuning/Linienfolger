#include <SparkFun_TB6612.h>

//Variablen Sensormapping

int customMap(int x, int in_min, int in_max, int out_min, int out_max) {
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
float Kp = 0.45;
float Ki = 0;
float Kd = 0.05;
float u;
float x;


//Motordrehzahlen
int nA, nB; //Last, errechnet
int ngA = 75; //Grundlast
int ngB = 75;
int nmaxApos = 115; //maximale Last
int nmaxBpos = 115;
int nmaxAneg = -115;
int nmaxBneg = -115; 


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
}


void loop()
{
while (millis() < 10000)
{
  //Aufruf Kalibrierung
  calibration();
}
while (millis() > 10000)
{
  //Aufruf PID
  pid();

}
  
}


//Funktion PID
void pid()
{
  
  //Linearisierung Sensorsignal
  sensorCal1 = customMap(analogRead(25), sensorMin1, sensorMax1, 300, 1200);
  sensorCal2 = customMap(analogRead(33), sensorMin2, sensorMax2, 300, 1200);
  sensorCal3 = customMap(analogRead(32), sensorMin3, sensorMax3, 300, 1200);
  sensorCal4 = customMap(analogRead(35), sensorMin4, sensorMax4, 300, 1200);
  sensorCal5 = customMap(analogRead(34), sensorMin5, sensorMax5, 300, 1200);

  //Position berechenen
  int sumCal = sensorCal1 + sensorCal2 + sensorCal3 + sensorCal4 + sensorCal5;
  x = (sensorCal1 * 0 + sensorCal2 * 1000 + sensorCal3 * 2000 + sensorCal4 * 3000 + sensorCal5 * 4000) / sumCal;
  float error = x-2000;

  //PID
  P = error;

  I = I + error;

  D = error - preverror;

  u = ((Kp * P) + (Ki * I) + (Kd * D));                        
  preverror = error;
  //Motordrehzahl berechnen


  nA = ngA - u;
  nB = ngB + u;

  //Motordrehzahl begrenzen
  nA = constrain(nA, nmaxAneg, nmaxApos);
  nB = constrain(nB, nmaxBneg, nmaxBpos);
  //Motoransteuerung
  motor1.drive(nA,1);
  motor2.drive(nB,1);




}



//Funktion Kalibrierung
void calibration() 
{
  unsigned long calibrationStartTime = millis();
  while (millis() - calibrationStartTime < 10000) {
    motor1.drive(50);
    motor2.drive(-50);
    calibrateSensor(25, sensorMin1, sensorMax1);
    calibrateSensor(33, sensorMin2, sensorMax2);
    calibrateSensor(32, sensorMin3, sensorMax3);
    calibrateSensor(35, sensorMin4, sensorMax4);
    calibrateSensor(34, sensorMin5, sensorMax5);
  }
}

void calibrateSensor(int pin, int& sensorMin, int& sensorMax) {
  int sensorValue = analogRead(pin);
  if (sensorValue > sensorMax) {
    sensorMax = sensorValue;
  }
  if (sensorValue < sensorMin) {
    sensorMin = sensorValue;
  }
}