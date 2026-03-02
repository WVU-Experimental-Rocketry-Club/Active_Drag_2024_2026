#include "computations.h"
#include "config_data.h"

#include <Wire.h>  //Needed for I2C to GNSS
#include <LittleFS.h>

#include <SparkFun_u-blox_GNSS_v3.h>  //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
#include "SparkFun_BNO08x_Arduino_Library.h"

#include "EasyBuzzer.h"

#define LOG_FILE "/flight_log.csv"

File logFile;
uint32_t lastLogTime   = 0;

// Define the pin your buzzer's positive (longer) leg or signal wire is connected to
const int BUZZER_PIN = 9; 

// Create the BNO085 object
BNO08x imu;  // -1 means we aren't using a hardware reset pin
sh2_SensorValue_t sensorValue;

SFE_UBLOX_GNSS myGNSS;  // SFE_UBLOX_GNSS uses I2C. For Serial or SPI, see Example2 and Example3

float predictApgee();
void updateSensors();

int flightState = 0;
bool initState = false;
void powerOn();      //0
void padIdle();      //1
void flightPower();  //2
void flightCoast();
void apogee();          //3
void landed();          //4
void serialTerminal();  //5
void logData();
void startLogging();
void stopLogging();
void logSample();
void handleSerialCommands();
void dumpLog();
void eraseLog();

#define stepPin 26
#define dirPin 27
#define enable 28

int currentPosition = 0;
int desiredPosition = 0;

String a;

#define stepDelayMicroseconds 83 //450rpm at 1600microsteps

int launchDetectGs = 4;
float motorBurntime = 4;
int soundSpeed = 343; //m/s at sea level
int flightBeginTime;

float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;

float launchAltitudeOffset = 0;

struct gpsState {
    uint8_t SIV;
    uint8_t fixType;
    float lat;
    float lon;
    float vx;
    float vy;
    float alt;
    float horAcc;
    float vertAcc;
    float speedAcc;
};

gpsState currGpsState; 

int powerOnDelaySecs = 5;
unsigned long powerOnDelayTimer = 0;

unsigned long lastRk4Time = 0;

unsigned long launchDetectTime = 0;

float dt = 0.1f;  // time step for RK4 integration

rk4State currState = { 0.0f, 0.0f, 0.0f, 0.0f };  // initial conditions: x=0, y=0, vx=0, vy=0

enum State { WAITING_FOR_LAUNCH, LOGGING, IDLE };
State state = WAITING_FOR_LAUNCH;

struct airbrakeState {
  float deployPct;
  float apoEstimate;
};
airbrakeState currAirbrakeState = {0.0f, 0.0f};

void setup() {
    // Initialization code here
    Serial.begin(115200);
    delay(1000);



    EasyBuzzer.setPin(BUZZER_PIN);

    Wire.begin();  // Start I2C

    //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

    while (myGNSS.begin() == false)  //Connect to the u-blox module using Wire port
    {
      Serial.println(F("u-blox GNSS not detected at default I2C address. Retrying..."));
      delay(1000);
    }

    myGNSS.setI2COutput(COM_TYPE_UBX);  //Set the I2C port to output UBX only (turn off NMEA noise)
    myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE4g);
    myGNSS.setAutoPVT(true);

    if (myGNSS.setAopCfg(1) == true) {
      Serial.println(F("aopCfg enabled"));
    } else {
      Serial.println(F("Could not enable aopCfg. Please check wiring. Freezing."));
    }

    myGNSS.setNavigationFrequency(10);

    //myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Optional: save (only) the communications port settings to flash and BBR


    // bno085 --------------------------------
    imu.begin(0x4A, Wire);
    imu.enableAccelerometer(50);
    imu.enableGyro(50);
    imu.enableMagnetometer(50);


          // Mount filesystem
    if (!LittleFS.begin()) {
      Serial.println("ERROR: LittleFS mount failed!");
    }
    else {
    Serial.println("LittleFS mounted.");
    }
}

void loop() {
    // Main code here
    // Serial.println("\n\nStarting Computation");
    // Serial.printf("Rk4 test: %f\n", rk4_step(startState, dt, 0.0f).y);
    // Serial.printf("get drag: %f\n", get_total_drag(startState.vy, startState.y, 0.0));
    // int startTime = millis();
    // float predictedApogee = predictApgee(startState, 0.0f, dt);
    // int elapsedTime = millis() - startTime;
    // Serial.println("Computation Complete");
    // Serial.printf("Predicted Apogee: %f\n", predictedApogee);
    // Serial.printf("Compute Time: %d ms", elapsedTime);

    // delay(2000);
    updateSensors();
    handleSerialCommands();



    if        (flightState == 0) {
      powerOn();
    } else if (flightState == 1) {
      padIdle();
    } else if (flightState == 2) {
      flightPower();
    } else if (flightState == 3) {
      flightCoast();
    } else if (flightState == 4) {
      apogee();
    } else if (flightState == 5) {
      landed();
    }
}

void loop1() {
    EasyBuzzer.update();
}

float predictApogee(rk4State , float deployAngle, float dt) {
  // Predict apogee logic here
  rk4State state = currState;
  Serial.printf("Alt: %f\nVy: %f\n", state.y, state.vy);
  int iterations = 0;
  while (state.vy > 0.0f) {
    state = rk4_step(state, dt, deployAngle);
    iterations++;
  }
  Serial.printf("Iterations: %d\n", iterations);
  return state.y;
}

void updateSensors() {
    // if (imu.getSensorEvent() == true) {
      
    //   switch (imu.getSensorEventID()) {
    //   case SENSOR_REPORTID_ACCELEROMETER:
    //     accelX = imu.getAccelX();
    //     accelY = imu.getAccelY();
    //     accelZ = imu.getAccelZ();
    //     break;
        
    //   case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
    //     gyroX = imu.getGyroX();
    //     gyroY = imu.getGyroY();
    //     gyroZ = imu.getGyroZ();
    //     break;
        
    //   case SENSOR_REPORTID_MAGNETIC_FIELD:
    //     magX = imu.getMagX();
    //     magY = imu.getMagY();
    //     magZ = imu.getMagZ();
    //   }
    // }

  if (myGNSS.getPVT() == true)
  {
    // uint8_t dynModel = myGNSS.getDynamicModel();

    float latitude = myGNSS.getLatitude() / 10000000.0; //degrees

    float longitude = myGNSS.getLongitude() / 10000000.0; //degrees

    float altitude = myGNSS.getAltitudeMSL() / 1000; // Altitude above Mean Sea Level (conv to meters)

    float hvel = myGNSS.getGroundSpeed() / 1000.0; //mm/s -> m/s
    float vvel = -myGNSS.getNedDownVel() / 10000.0; //flip to ascending 

    uint8_t satellites = myGNSS.getSIV(); // Satellites In View

    uint8_t fixType = myGNSS.getFixType(); //0 no fix, 3 3d fix

    float horAcc = myGNSS.getHorizontalAccuracy();
    float vertAcc = myGNSS.getVerticalAccuracy();
    float spdAcc = myGNSS.getSpeedAccEst();

    currGpsState = {satellites, fixType, latitude, longitude, hvel, vvel, altitude, horAcc, vertAcc, spdAcc};
  }
}

void powerOn() {
  if (!initState) {
    EasyBuzzer.beep(2000, 100, 100, 2, 2000, 0);
    initState = true;
  }
  if (currGpsState.fixType == 3 && currGpsState.SIV >= 10) {
    Serial.print("SIV: ");
    Serial.println(currGpsState.SIV);
    flightState = 1;
    // Frequency (Hz), On-Time (ms), Off-Time (ms), Beeps per cycle, Pause between cycles (ms), Number of cycles (0 = forever)
    EasyBuzzer.beep(2000, 100, 100, 3, 2000, 0); 
    launchAltitudeOffset = currGpsState.alt;
    initState = false;
  }
}

void padIdle() {
  if (!initState) {
    EasyBuzzer.beep(2000, 50, 50, 3, 500, 0);
    initState = true;
  }
  if (currGpsState.alt - launchAltitudeOffset >= 100) {
    flightState = 2;
    initState = false;
    startLogging();
  }
      Serial.print("SIV: ");
    Serial.println(currGpsState.SIV);

          Serial.print("vy: ");
    Serial.println(currGpsState.vy);

          Serial.print("alt: ");
    Serial.println(currGpsState.alt);

    //       Serial.print("accelx: ");
    // Serial.println(accelZ);
    Serial.print("\n\n\n");
}

void flightPower() {
  if (!initState) {
    launchDetectTime = millis();
    initState = true;
  }
  if (accelZ > 0 && currGpsState.vy < MAXIMUM_DEPLOYMENT_MACH * soundSpeed) {
    flightState = 3;
    initState = false;
  }
  logSample();

}

void flightCoast() {

  if (currGpsState.vy < 0) {
      flightState = 4; 
  }

  if (lastRk4Time == 0) {
    lastRk4Time = millis();


  }
  else if (millis() - lastRk4Time > 100){
    currState = {0, currGpsState.alt, currGpsState.vx, currGpsState.vy};
    
    float currEstimate = predictApogee(currState, 0, dt);
    currAirbrakeState = {currEstimate, 0.0f};

  }
  logSample();

}

void apogee() {

  if (currGpsState.alt - launchAltitudeOffset < 50) {
    flightState = 5;
    stopLogging();
  }

}

void landed() {
}

void serialTerminal() {
}

void logData() {
  
}


void startLogging() {
  Serial.println("LAUNCH DETECTED — starting log.");
  state = LOGGING;

  // Open file (append so multiple flights can stack, or use "w" to overwrite)
  logFile = LittleFS.open(LOG_FILE, "a");
  if (!logFile) {
    Serial.println("ERROR: Could not open log file!");
    state = IDLE;
    return;
  }

  // Write CSV header (only if file was just created / empty)
  if (logFile.size() == 0) {
    logFile.println("time_ms,gpsAlt,latitude,longitude,groundSpeed,verticalSpeed,sats,acceleration,deployPct,apogeeEstimate");
  }
}

void stopLogging() {
  logFile.flush();
  logFile.close();
  state = IDLE;
  Serial.println("Logging stopped. Send 'd' to download.");
}

void logSample() {
  // Build line manually to avoid String heap fragmentation
  char buf[128];
  snprintf(buf, sizeof(buf),
           "%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%d,%.4f,%.2f,%.2f",
           millis() - launchDetectTime, currGpsState.alt, currGpsState.lat, currGpsState.lon, currGpsState.vx, currGpsState.vy,currGpsState.SIV,accelZ,currAirbrakeState.deployPct, currAirbrakeState.apoEstimate);
  logFile.println(buf);
}

// ──────────────────────────────────────────────
// Serial command handler (for post-flight offload)
// ──────────────────────────────────────────────
void handleSerialCommands() {
  if (!Serial.available()) return;
  char cmd = Serial.read();

  if (cmd == 'd' || cmd == 'D') {
    dumpLog();
  } else if (cmd == 'e' || cmd == 'E') {
    eraseLog();
  } else if (cmd == 's' || cmd == 'S') {
    printStatus();
  }
}

void dumpLog() {
  if (!LittleFS.exists(LOG_FILE)) {
    Serial.println("NO_LOG");
    return;
  }

  File f = LittleFS.open(LOG_FILE, "r");
  if (!f) {
    Serial.println("ERROR: Cannot open log.");
    return;
  }

  Serial.println("BEGIN_LOG");
  while (f.available()) {
    // Read and forward in chunks for speed
    uint8_t buf[256];
    int n = f.read(buf, sizeof(buf));
    Serial.write(buf, n);
  }
  f.close();
  Serial.println("\nEND_LOG");
}

void eraseLog() {
  LittleFS.remove(LOG_FILE);
  Serial.println("Log erased.");
}

void printStatus() {
  Serial.printf("State: %s\n",
    state == WAITING_FOR_LAUNCH ? "WAITING" :
    state == LOGGING ? "LOGGING" : "IDLE");
  Serial.printf("Log exists: %s\n", LittleFS.exists(LOG_FILE) ? "yes" : "no");
}