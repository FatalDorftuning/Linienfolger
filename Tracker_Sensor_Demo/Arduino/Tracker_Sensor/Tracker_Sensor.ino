/* WaveShare ARPICAR Run Forward/Backward/Left/Right Test
   
   ARPICAR run forward,backward,left right and so on..
   
   Created 25th June 2016
           by Xinwu Lin
           
   CN: http://www.waveshare.net/
   EN: http://www.waveshare.com/
*/

void setup()
{
  Serial.begin(115200);
  Serial.println("TRSensor example");
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);

}


void loop()
{

  Serial.print(analogRead(A0));
  Serial.print("  ");
  Serial.print(analogRead(A1));
  Serial.print("  ");
  Serial.print(analogRead(A2));
  Serial.print("  ");
  Serial.print(analogRead(A3));
  Serial.print("  ");
  Serial.print(analogRead(A4));
  Serial.println();
  delay(200);

}
