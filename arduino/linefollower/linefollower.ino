#include <QTRSensors.h>
#include <Wire.h>

QTRSensorsRC sensor1((unsigned char[]) {22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37}, 16, 2500, 255);
QTRSensorsRC sensor2((unsigned char[]) {53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38}, 16, 2500, 255);

double s1[16];
double s2[16];
unsigned int sensor1_values[16];
unsigned int sensor2_values[16];
unsigned int arr1[16];
unsigned int arr2[16];
unsigned int threshold = 200;
byte sender[33];

void setup() {  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  for (int i = 0; i < 100; i++){
    sensor1.calibrate();
    delay(20);
  }
  sensor1.read(sensor1_values);
  for (int i = 0; i < 100; i++){
    sensor2.calibrate();
    delay(20);
  }
  sensor2.read(sensor2_values);
  for (int i = 0; i<16; i++){
    s1[i] = sensor1_values[i];
  }
  for (int i = 0; i<16; i++){
    s2[i] = sensor2_values[i];
  }   
  Wire.onRequest(requestEvent);
}

void requestEvent(){
  sensor1.read(sensor1_values);
  sensor2.read(sensor2_values);
  sender[0] = (byte)45;
  for (int i = 0; i<16; i++){
      sender[i+1] = (byte)sensor1_values[i]/4;
  }
  for (int i = 0; i<16; i++){
      sender[17+i] = (byte)sensor2_values[i]/4;
  }
  Wire.write(sender,33);
}
void loop()
{
  delay(20);
}
