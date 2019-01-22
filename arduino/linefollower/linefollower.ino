#include <QTRSensors.h>

void setup() {
  QTRSensorsRC sensor1((unsigned char[]) {22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37}, 16);
  QTRSensorsRC sensor2((unsigned char[]) {38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53}, 16);
  unsigned int sensor1_values[16];
  unsigned int sensor2_values[16];
  Serial.begin(9600);
}

void loop()
{
  sensor1.read(sensor1_values);
  sensor2.read(sensor2_values);
  for (int i = 0; i <16; i++){
    Serial.print(sensor1_values[i]);
    Serial.print('\t');
  }
  Serial.print('\n');  
  for (int i = 0; i <16; i++){
    Serial.print(sensor2_values[i]);
    Serial.print('\t');
  }
  Serial.print('\n');
  delay(1000);
}
