#include <Wire.h>
#include <LPS.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <math.h>

LIS3MDL mag;
LSM6 imu;
LPS ps;

char report[80];
  
unsigned long currentTime = 0;
float angleX = 0;
float angleY = 0;
float gyOld = 0;
float gy = 0;
void setup()
{
  Serial.begin(9600);
  Wire.begin();

  if (!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }  
  ps.enableDefault();
  
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();

  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }

  mag.enableDefault();
}

void loop()
{
  float pressure = ps.readPressureMillibars();
  float altitude = ps.pressureToAltitudeMeters(pressure);
  float temperature = ps.readTemperatureC();
  
  Serial.print("p: ");
  Serial.print(pressure);
  Serial.print(" mbar\ta: ");
  Serial.print(altitude);
  Serial.print(" m\tt: ");
  Serial.print(temperature);
  Serial.print(" deg C");
  Serial.print("\t");
  imu.read();
   mag.read();
  float ax = imu.a.x ;
  float ay = imu.a.y ;
  float az = imu.a.z ;
  ax = ax / 1638;
  ay = ay / 1638;
  az = az / 1638;
  
  gyOld = gy;
  float gx = imu.g.x ;
  gy = imu.g.y ;
  float gz = imu.g.z ;
  gx = gx/262,144;
  gy =  - gy/262,144;
  gz = gz/262,144;
  if(gy<3 && gy>-3)gy=0;
  
  float mx = mag.m.x + 182 ;
  float my = mag.m.y + 1341;
  float mz = mag.m.z - 2893;
  mx = mx/819,2;
  my = my/819,2;
  mz = mz/819,2;

  float dt = (millis() - currentTime);
  currentTime = millis();

  angleX = ((angleX + ((gy + gyOld) *1 * dt/1000))*0.9) + atan2(ax,az) *180/3.14*0.1;
  angleY = atan2(-1*ay,sqrt(ax*ax + az*az)) * 180/3.14;

  Serial.print(ax);
  Serial.print("||");
  Serial.print(ay);
  Serial.print("||");
  Serial.print(az);
  Serial.print("\t");
  Serial.print(gx);
  Serial.print("||");
  Serial.print(gy);
  Serial.print("||");
  Serial.print(gz);
  Serial.print("\t");
  Serial.print(angleX);
  Serial.print("\t");
  Serial.println(angleY);
//  Serial.print(mx);
//  Serial.print("||");
//  Serial.print(my);
//  Serial.print("||");
//  Serial.println(mz);
 // snprintf(report, sizeof(report), " %6d %6d %6d     %6d %6d %6d",
  //  imu.a.x, imu.a.y, imu.a.z,
  //  imu.g.x, imu.g.y, imu.g.z);
//  Serial.println(report);

//   

//  snprintf(report, sizeof(report), " %6d %6d %6d",
//    mag.m.x, mag.m.y, mag.m.z);
//  Serial.println(report);
//  Serial.println("S");
  
}
