/*
  Reading Position, Velocity and Time (PVT) via UBX binary commands
  By: Paul Clark
  SparkFun Electronics
  Date: December 21st, 2022
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to query a u-blox module for its position, velocity and time (PVT) data.
  We also turn off the NMEA output on the I2C port. This decreases the amount of I2C traffic dramatically.

  Note: Lat/Lon are large numbers because they are * 10^7. To convert lat/lon
  to something google maps understands simply divide the numbers by 10,000,000.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481
  SparkFun MAX-M10S Breakout (GPS-18037)     https://www.sparkfun.com/products/18037
  SparkFun ZED-F9K Breakout (GPS-18719)      https://www.sparkfun.com/products/18719
  SparkFun ZED-F9R Breakout (GPS-16344)      https://www.sparkfun.com/products/16344

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and your microcontroller board
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/


#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
#include "SparkFun_BNO08x_Arduino_Library.h"

// Create the BNO085 object
BNO08x imu;
sh2_SensorValue_t sensorValue;

// Create variables to hold the latest data for each sensor
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;

SFE_UBLOX_GNSS myGNSS; // SFE_UBLOX_GNSS uses I2C. For Serial or SPI, see Example2 and Example3


void setup()
{
  Serial.begin(115200);
  delay(1000); 
  Serial.println("SparkFun u-blox Example");

  Wire.begin(); // Start I2C

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  while (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Retrying..."));
    delay (1000);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE4g);

    if (myGNSS.setAopCfg(1) == true)
  {
    Serial.println(F("aopCfg enabled"));
  }
  else
  {
    Serial.println(F("Could not enable aopCfg. Please check wiring. Freezing."));
  }

  myGNSS.setNavigationFrequency(10);
  
  //myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Optional: save (only) the communications port settings to flash and BBR


  // bno085 --------------------------------
  imu.begin(0x4A, Wire);
  imu.enableAccelerometer(50);
  imu.enableGyro(50);
  imu.enableMagnetometer(50);
}

void loop()
{
  // Request (poll) the position, velocity and time (PVT) information.
  // The module only responds when a new position is available. Default is once per second.
  // getPVT() returns true when new data is received.
  if (myGNSS.getPVT() == true)
  {
    uint8_t dynModel = myGNSS.getDynamicModel();
    Serial.print("Dynamic Model: ");
    Serial.println(dynModel);

    float latitude = myGNSS.getLatitude() / 10000000.0; //degrees
    Serial.print(F("Lat: "));
    Serial.println(latitude, 7);

    float longitude = myGNSS.getLongitude() / 10000000.0; //degrees
    Serial.print(F(" Long: "));
    Serial.println(longitude, 7);

    float altitude = myGNSS.getAltitudeMSL() / 1000; // Altitude above Mean Sea Level (conv to meters)
    Serial.print(F(" Alt: "));
    Serial.print(altitude, 7);
    Serial.println(F(" (m)"));

    float hvel = myGNSS.getGroundSpeed() / 1000.0; //mm/s -> m/s
    float vvel = -myGNSS.getNedDownVel() / 10000.0; //flip to ascending 

    Serial.print(F(" hvel: "));
    Serial.print(hvel);
    Serial.println(F(" (m/s)"));

    Serial.print(F(" vvel: "));
    Serial.print(vvel);
    Serial.println(F(" (m/s)"));

    byte satellites = myGNSS.getSIV(); // Satellites In View

    Serial.print("Satellites: ");
    Serial.println(satellites);

    int fixType = myGNSS.getFixType(); //0 no fix, 3 3d fix

    Serial.print("Fix type: ");
    Serial.println(fixType);


    if (imu.getSensorEvent() == true) {
        
        switch (imu.getSensorEventID()) {
      case SENSOR_REPORTID_ACCELEROMETER:
        accelX = imu.getAccelX();
        accelY = imu.getAccelY();
        accelZ = imu.getAccelZ();
        break;
        
      case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
        gyroX = imu.getGyroX();
        gyroY = imu.getGyroY();
        gyroZ = imu.getGyroZ();
        break;
        
      case SENSOR_REPORTID_MAGNETIC_FIELD:
        magX = imu.getMagX();
        magY = imu.getMagY();
        magZ = imu.getMagZ();
        
        // Let's use the Magnetometer packet as our trigger to print everything.
        // Since they all update at 50ms, printing when the Mag updates gives us a clean row.
        // Serial.print(accelX); Serial.print(",");
        // Serial.print(accelY); Serial.print(",");
        // Serial.print(accelZ); Serial.print(" | ");
        
        // Serial.print(gyroX); Serial.print(",");
        // Serial.print(gyroY); Serial.print(",");
        // Serial.print(gyroZ); Serial.print(" | ");
        
        // Serial.print(magX); Serial.print(",");
        // Serial.print(magY); Serial.print(",");
        // Serial.println(magZ);
        break;
    }
    }

    Serial.print(accelX); Serial.print(",");
    Serial.print(accelY); Serial.print(",");
    Serial.print(accelZ); Serial.print(" | ");
    
    Serial.print(gyroX); Serial.print(",");
    Serial.print(gyroY); Serial.print(",");
    Serial.print(gyroZ); Serial.print(" | ");
    
    Serial.print(magX); Serial.print(",");
    Serial.print(magY); Serial.print(",");
    Serial.println(magZ);
    Serial.print(F("\n\n\n"));
  }
}
