#include <Arduino_LSM9DS1.h>
#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


float accelX,            accelY,             accelZ,            // units m/s/s i.e. accelZ if often 9.8 (gravity)
      gyroX,             gyroY,              gyroZ,             // units dps (degrees per second)
      gyroDriftX,        gyroDriftY,         gyroDriftZ,        // units dps
      gyroRoll,          gyroPitch,          gyroYaw,           // units degrees (expect major drift)
      gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw,  // units degrees (expect minor drift)
      accRoll,           accPitch,           accYaw,            // units degrees (roll and pitch noisy, yaw not possible)
      complementaryRoll, complementaryPitch, complementaryYaw;  // units degrees (excellent roll, pitch, yaw minor drift)

long lastTime;
long lastInterval;

int directions, angles, temps(0), acceleration, acc;

unsigned long temperature_timer = millis();
//float angles_offset;

void setup() {
  u8g2.begin();
  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");
    while (1);
  }
  calibrateIMU(250, 250);
  lastTime = micros();
  delay(2000);
}

//----------------------------------------------CALIBLRATION
void calibrateIMU(int delayMillis, int calibrationMillis) { 
  int calibrationCount = 0;
  delay(delayMillis); // to avoid shakes after pressing reset button
  float sumX, sumY, sumZ;
  int startTime = millis();
  while (millis() < startTime + calibrationMillis) {
    if (readIMU()) {
      // in an ideal world gyroX/Y/Z == 0, anything higher or lower represents drift
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;

      calibrationCount++;
    }
  }
  if (calibrationCount == 0) {
    //Serial.println("Failed to calibrate");
  }
  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;
}
//----------------------------------------------LIRE INFO
bool readIMU() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() ) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ); 
//    if(millis() < 2500){
//      accPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI;
//      angles_offset = accPitch;  
//    }
    return true;
  }
  return false;
}
//----------------------------------------------MAIN
void loop() {
  angles = complementaryPitch; 
  u8g2.firstPage();
  do {
    //----------------------------------------------UPDATE DES INFOS ET CALCULER L'ANGLE ACTUEL
    if (readIMU()) {
      long currentTime = micros();
      lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
      lastTime = currentTime;
      doCalculations();       
    }    
    //----------------------------------------------AFFICHAGE ANGLE
    u8g2.setFont(u8g2_font_helvB24_tn);
    u8g2.setCursor(47,40);
    u8g2.print(angles);
    //---------------------------------------------    
  } while ( u8g2.nextPage() );
  
}

//----------------------------------------------CALCULER L'ANGLE 
bool flag = 0;
void doCalculations() {
  accRoll = atan2(accelY, accelZ) * 180 / M_PI;
  accPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI;
  //gyroPitch = gyroPitch - angles_offset;
  float lastFrequency = (float) 1000000.0 / lastInterval;
  gyroRoll = gyroRoll + (gyroX / lastFrequency);
  gyroPitch = gyroPitch + (gyroY / lastFrequency);
  gyroYaw = gyroYaw + (gyroZ / lastFrequency);

  gyroCorrectedRoll = gyroCorrectedRoll + ((gyroX - gyroDriftX) / lastFrequency);
  gyroCorrectedPitch = gyroCorrectedPitch + ((gyroY - gyroDriftY) / lastFrequency);
  gyroCorrectedYaw = gyroCorrectedYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = complementaryRoll + ((gyroX - gyroDriftX) / lastFrequency);
  complementaryPitch = complementaryPitch + ((gyroY - gyroDriftY) / lastFrequency);
  complementaryYaw = complementaryYaw + ((gyroZ - gyroDriftZ) / lastFrequency);
  
  //gyroCorrectedPitch = gyroCorrectedPitch - angles_offset;
  
  complementaryRoll = 0.95 * complementaryRoll + 0.05 * accRoll;
  complementaryPitch = 0.95 * complementaryPitch + 0.05 * accPitch;
}
