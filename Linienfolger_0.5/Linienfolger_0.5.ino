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
float Kp = 16;
float Ki = 0;
float Kd = 0;
float u;
float x;


//Motordrehzahlen
int nA, nB; //Last, errechnet
int ngA = 60; //Grundlast
int ngB = 60;
int nmaxApos = 80; //maximale Last
int nmaxBpos = 80;
int nmaxAneg = -80;
int nmaxBneg = -80; 


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
while (millis() < 3500)
{
  //Aufruf Kalibrierung
  calibration();
}
while (millis() > 3500)
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
  nA = ngA - u;
  nB = ngB + u;
/*
  if (nA > -50 && nA < 0)
  {
    nA = -50;
  }
  if (nA < 50 && nA > 0)
  {
    nA = 50;
  }
  
  if (nB > -50 && nB < 0)
  {
    nB = -50;
  }
  if (nB < 50 && nB > 0)
  {
    nB = 50;
  }

*/

  //Motordrehzahl begrenzen
  if (nA > nmaxApos)
  {
    nA = nmaxApos;
  }

  if (nB > nmaxBpos)
  {
    nB = nmaxBpos;
  }
    if (nA < nmaxAneg)
  {
    nA = nmaxAneg;
  }

  if (nB < nmaxBneg)
  {
    nB = nmaxBneg;
  }

  //Motoransteuerung
  motor1.drive(nA, 250);
  motor2.drive(nB, 250);
  

}
  
}


//Funktion PID
void pid()
{

x = (sensorCal1*1000+sensorCal2*2000+sensorCal3*3000+sensorCal4*4000+sensorCal5*5000)/(sensorCal1+sensorCal2+sensorCal3+sensorCal4+sensorCal5);

float error = x-3050;
Serial.begin(115200);
Serial.print(error);
Serial.print("   ");
/*
while(sensorCal1>=980 && sensorCal2>=980 && sensorCal3>=980 && sensorCal4>=980 && sensorCal5>=980)
{ 
  if(preverror>0)
  {       //nach links fahren, wenn der vorherige Fehler auf der rechten Seite war
   motor1.drive() ;
  }
  else
  {
    right(motor1, motor2, 60); //nach rechts fahren
  }
  x = (sensorCal1*1000+sensorCal2*2000+sensorCal3*3000+sensorCal4*4000+sensorCal5*5000)/(sensorCal1+sensorCal2+sensorCal3+sensorCal4+sensorCal5);
}
*/

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