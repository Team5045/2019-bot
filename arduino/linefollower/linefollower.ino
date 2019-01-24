#include <QTRSensors.h>
#include <math.h>
#define PI 3.14159265

QTRSensorsRC sensor1((unsigned char[]) {22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37}, 16);
QTRSensorsRC sensor2((unsigned char[]) {38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53}, 16);
unsigned int sensor1_values[16];
unsigned int sensor2_values[16];
unsigned int line1;
unsigned int line2;
unsigned double angle;
unsigned double dist;

void setup() {
  //Serial.begin(9600);
}

void loop()
{
  sensor1.read(sensor1_values, readMode=QTR_EMITTERS_OFF);
  line1 = sensor1.readLine(sensor1_values, readMode=QTR_EMITTERS_OFF, whiteLine=1)
  sensor2.read(sensor1_values, readMode=QTR_EMITTERS_OFF);
  line2 = sensor1.readLine(sensor1_values, readMode=QTR_EMITTERS_OFF, whiteLine=1)
  angle = atan((line1-line2)/1600.0);
  dist = (8500-(line1+line2))/2000.0;
  /*
  Serial.print(line1); 
  Serial.print('\t'); 
  Serial.print(line2);
  Serial.print('\n'); 
  delay(200); 
  */

}
