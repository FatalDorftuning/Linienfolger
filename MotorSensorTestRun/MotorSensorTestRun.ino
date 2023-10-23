
#include <SparkFun_TB6612.h>

//globale Variablen
int sensorMin1 = 1800; //Wert, wenn Sensor1 "schwarz" erkennt
int sensorMax1 = 3600; //Wert, wenn Sensor1 "weiß" erkennt
int sensorMin2 = 1800; //Wert, wenn Sensor2 "schwarz" erkennt
int sensorMax2 = 3600; //Wert, wenn Sensor2 "weiß" erkennt
int sensorMin3 = 1800; //Wert, wenn Sensor3 "schwarz" erkennt
int sensorMax3 = 3600; //Wert, wenn Sensor3 "weiß" erkennt
int sensorMin4 = 1800; //Wert, wenn Sensor4 "schwarz" erkennt
int sensorMax4 = 3600; //Wert, wenn Sensor4 "weiß" erkennt
int sensorMin5 = 1800; //Wert, wenn Sensor5 "schwarz" erkennt
int sensorMax5 = 3600; //Wert, wenn Sensor5 "weiß" erkennt



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
int Kp = 12;
int Ki = 0.3;
int Kd = 0.6;
int u;
//Motordrehzahlen
int nA, nB; //Last, errechnet
int ngA = 50; //Grundlast
int ngB = 50;
int nmaxA = 100; //maximale Last
int nmaxB = 100;

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
  Serial.begin(115200);
  pinMode(25, INPUT);                   //Input-Pins der Sensoren
  pinMode(33, INPUT);
  pinMode(32, INPUT);
  pinMode(35, INPUT);
  pinMode(34, INPUT);
}


void loop()
{
Serial.begin(115200);
while (millis() < 5000)
{
  //Aufruf Kalibrierung
  calibration();
}
while (millis() > 5000)
{

  //Auswertung Sensorkalibrierung
   sensorCal1 = map(analogRead(25), sensorMin1, sensorMax1,0,1000);
   sensorCal2 = map(analogRead(33), sensorMin2, sensorMax2,0,1000);
   sensorCal3 = map(analogRead(32), sensorMin3, sensorMax3,0,1000);
   sensorCal4 = map(analogRead(35), sensorMin4, sensorMax4,0,1000);
   sensorCal5 = map(analogRead(34), sensorMin5, sensorMax5,0,1000);

  //Aufruf PID
  pid();

  //Ansteuerung Motor
  nA = ngA + u;
  nB = ngB - u;
  if (nA > nmaxA)
  {
    nA = nmaxA;
  }

  if (nB > nmaxB)
  {
    nB = nmaxB;
  }

  motor1.drive(nA, 1000);
  motor2.drive(nB, 1000);
  Serial.begin(115200);
   Serial.print("Error:");
  Serial.print(error);
  Serial.print("   u:");
  Serial.print(u);
  

}
  
}


//Funktion PID
void pid()
{

int error = ((sensorCal1*1.5 + sensorCal2) - (sensorCal4 + sensorCal5*1.5));

P = error;

I = I + error;

D = error - preverror;

u = ((Kp * P) + (Ki * I) + (Kd * D))/100;
preverror = error;


}



//Funktion Kalibrierung
void calibration ()
{

 while (millis() < 5000)
 {
   motor1.drive(50);
   motor2.drive(-50);

   if (analogRead(25)>sensorMax1)
   {
     sensorMax1 = analogRead(25);
   }
    if (analogRead(25)<sensorMin1)
   {
     sensorMin1 = analogRead(25);
   }


     if (analogRead(33)>sensorMax2)
   {
     sensorMax2 = analogRead(33);
   }
    if (analogRead(33)<sensorMin2)
   {
     sensorMin2 = analogRead(33);
   }  
   

   if (analogRead(32)>sensorMax3)
   {
     sensorMax3 = analogRead(32);
   }
    if (analogRead(32)<sensorMin3)
   {
     sensorMin3 = analogRead(32);
   }  
   
   
   if (analogRead(35)>sensorMax4)
   {
     sensorMax4 = analogRead(35);
   }
    if (analogRead(35)<sensorMin4)
   {
     sensorMin4 = analogRead(35);
   }  
   
   
   if (analogRead(34)>sensorMax5)
   {
     sensorMax5 = analogRead(34);
   }
    if (analogRead(34)<sensorMin5)
   {
     sensorMin5 = analogRead(34);
   }

}
 
}