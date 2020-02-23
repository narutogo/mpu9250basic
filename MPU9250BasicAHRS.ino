#include"src/mpu/mpu9250basic.h"
void setup()
{


  Serial.begin(115200);
  mpusetup();
}
void loop()
{
  float a[3], b[3];
  getoritation(a, b);
  Serial.println("ange:" + String(a[0]) + " " + String(a[1]) + " " + String(a[2]) + " " + String(b[0]) + " " + String(b[1]) + " " + String(b[2]) + " ");
}
