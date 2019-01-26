#include <QTRSensors.h>
#include <math.h>
#define PI 3.14159265

QTRSensorsRC sensor1((unsigned char[]) {23,22,25,24,27,26,29,28,31,30,33,32,35,34,37,36}, 16, 2500, 255);
//QTRSensorsRC sensor2((unsigned char[]) {38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53}, 16, 3000, 255);
unsigned int sensor1_values[16];
unsigned int sensor2_values[16];
unsigned int line1;
unsigned int line2;
double angle;
double dist;

void setup() {
  Serial.flush();
  Serial.begin(9600);
  //for (int i = 0; i < 250; i++){
  //  sensor1.calibrate();
  //  delay(20);
  //}
}

void loop()
{
  sensor1.read(sensor1_values);
  //sensor2.read(sensor1_values, readMode=QTR_EMITTERS_OFF);
  //line2 = sensor2.readLine(sensor2_values, readMode=QTR_EMITTERS_OFF, whiteLine=1)
  //angle = atan((line1-line2)/1600.0);
  //dist = (8000-(line1+line2))/2000.0;
  for (int i=0; i<16; i++){
    Serial.print(sensor1_values[i]);
    Serial.print("\t");
  }
  Serial.print(sensor1.readLine(sensor1_values, QTR_EMITTERS_ON, 1));
  Serial.print('\n'); 
  delay(20); 

}
