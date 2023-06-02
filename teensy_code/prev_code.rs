/* Author: Yiwei Mao
 * Date: 2021-03-21
 * File: schedule_sensors.ino
 */


/* Table of Contents for Code
 * Configuration
 * Structs
 * Task Scheduler
 * Status LEDs
 * RTC functions
 * IMU functions
 * GPS functions
 * SD functions
 * Pressure/Humidity functions
 * Jetson time sync
 * Setup and Run
 */

/***************************** CONFIGURATION *****************************/
//#define SERIAL_SHOW         // define this to show diagnostic messages over Serial

#define RTC_REFRESH_TIME_MS 10
#define AIR_REFRESH_TIME_MS 10
#define IMU_REFRESH_TIME_MS 10
#define GPS_REFRESH_TIME_MS 50

#define SD_STORE_REFRESH_TIME_MS    200
#define JETSON_REFRESH_TIME_MS      10
#define STATUS_REFRESH_TIME_MS      100

#define INIT_TIME_MS        2000  // give a chance for all sensors to be initialised

/***************************** STRUCTS ***********************************/

#include "Adafruit_BNO055.h" // for imu::Vector<3> definitions
struct datastore_t {

  char  rtc_now[30];    // datetime64 string
  float rtc_temp;       // deg C

  float air_temp;       // deg C
  float air_pressure;   // hPa
  float air_humidity;   // relative humidity %

  int8_t imu_temp;      // deg C
  imu::Vector<3> euler; // deg
  imu::Quaternion quat;
  imu::Vector<3> mag;   // uT

  char gps_now[30];     // datetime64 string
  int32_t latitude;     // deg *10^-7
  int32_t longitude;    // deg *10^-7
  int32_t altitude;     // mm above ellipsoid
  uint8_t numSV;        // number of satellites in view (and used in compute)
  int32_t velN;         // velocity in NED coordinates in mm/s
  int32_t velE;
  int32_t velD;
  int32_t gSpeed;       // ground speed in mm/s
  int32_t heading;      // heading of motion deg * 1e-5
  uint32_t velAcc;      // Speed accuracy estimate: mm/s
  uint16_t pDOP;        // Position DOP * 0.01
  uint32_t hAcc;        // Horizontal accuracy estimate: mm
  uint32_t vAcc;        // Vertical accuracy estimate: mm
  uint32_t headAcc;     // Heading accuracy estimate (both motion and vehicle): deg * 1e-5
  int16_t magDec;       // Magnetic declination: deg * 1e-2
  uint16_t magAcc;      // Magnetic declination accuracy: deg * 1e-2

};
datastore_t store;

/* Sensor status to show on RGB status LED
 * R: all sensor init
 * G: GPS lock
 * B: IMU system calibration
 */
struct sensor_status_t {
  bool rtc = false;  
  bool imu = false;
  bool gps = false;
  bool air = false;
  bool rtc_sync = false;
  uint8_t imu_cal = 0;
  bool jetson_sync = false;
  bool gps_lock = false;
  bool sd_updating = false;
};
sensor_status_t status;

/***************************** RTOS **************************************/
// OS task scheduler
#include "os.h"

/***************************** STATUS LEDS *******************************/
#define LED_RED   5
#define LED_GREEN 6
#define LED_BLUE  7
#define GPIO_JETSON 8

void initStatusLEDs() {
  status.air = false;
  status.imu = false;
  status.rtc = false;
  status.gps = false;
  status.imu_cal = 0xFF;
  status.jetson_sync = false;
  status.gps_lock = false;
  status.sd_updating = false;

  pinMode(LED_RED,OUTPUT);
  pinMode(LED_GREEN,OUTPUT);
  pinMode(LED_BLUE,OUTPUT);
  pinMode(GPIO_JETSON,INPUT);

  run_later(updateStatus,INIT_TIME_MS);
}

void updateStatus() {
  /* Sensor status to show on RGB status LED
  * R: all sensor init
  * G: GPS lock
  * B: IMU system calibration
  */
  run_later(updateStatus,STATUS_REFRESH_TIME_MS);

  if ( status.jetson_sync )
    analogWrite(LED_RED, 128); // 0--255 but 255 is too bright
  else
    analogWrite(LED_RED, 255);

  if ( status.gps_lock )
    analogWrite(LED_GREEN, 192); 
  else
    analogWrite(LED_GREEN, 255);

  analogWrite(LED_BLUE, !status.imu_cal); 
}


/***************************** Real Time Clock *******************************/
#include "elapsedMillis.h"
#include "RTClib.h"
RTC_DS3231 rtc;
elapsedMillis elapsedTime;
elapsedMillis elapsedTime2;

void initRTC() {
  if (! rtc.begin()) {
    #if defined(SERIAL_SHOW)
    Serial.println("Couldn't find RTC");
    Serial.flush();
    #endif
    abort();
  }
  if (rtc.lostPower()) {
    #if defined(SERIAL_SHOW)
    Serial.println("RTC lost power, let's set the time!");
    #endif
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  #if defined(SERIAL_SHOW)
  Serial.println("PiRTC (DS3231) Real Time Clock Initialised!");
  #endif

  status.rtc_sync = false;

  run_later(checkRTC,INIT_TIME_MS);
}

void checkRTC() {
  run_later(checkRTC,RTC_REFRESH_TIME_MS);
  status.rtc = true;

  // Store it in a python datetime string format
  // >>> from datetime import datetime
  // >>> print( datetime.strptime("2018-12-19 09:26:03", "%Y-%m-%d %H:%M:%S") )
  DateTime now = rtc.now();
  //store.rtc_now = now.toString("YYYY-MM-DD hh:mm:ss");
  sprintf(store.rtc_now,"%04d-%02d-%02d %02d:%02d:%02d.%03d",now.year(),now.month(),
                        now.day(),now.hour(),now.minute(),now.second(),uint16_t(elapsedTime%1000));
  if ( !status.rtc_sync && status.gps_lock && elapsedTime >= 1000) {
    status.rtc_sync = true;
    elapsedTime = elapsedTime%1000;
    rtc.adjust(now.unixtime()+1);
  }

  store.rtc_temp = rtc.getTemperature(); // deg C to 2 decimal places

  // Diagnostics
  #if defined(SERIAL_SHOW)
  Serial.print("RTC Time = "); Serial.println(store.rtc_now);
  Serial.print("RTC Temperature = "); Serial.print(store.rtc_temp,2); Serial.println(" deg C");
  #endif
}



/***************************** Inertial Measurement Unit *********************/
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include <utility/imumaths.h>
Adafruit_BNO055 imuSensor = Adafruit_BNO055(55, 0x28);

void initIMU() {
  if (!imuSensor.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    #if defined(SERIAL_SHOW)
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    Serial.flush();
    #endif
    abort();
  }
  #if defined(SERIAL_SHOW)
  Serial.println("BNO-055 IMU Initialised!");
  #endif
  //imuSensor.setExtCrystalUse(true);

  run_later(checkIMU,INIT_TIME_MS);
}

void checkIMU() {
  run_later(checkIMU,IMU_REFRESH_TIME_MS);
  status.imu = true;

  store.imu_temp = imuSensor.getTemp();
  // access via store.euler.x(), y(), z()
  store.euler = imuSensor.getVector(Adafruit_BNO055::VECTOR_EULER);
  store.quat  = imuSensor.getQuat();
  store.mag   = imuSensor.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  //store.gyro  = imuSensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  //store.accel = imuSensor.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  uint8_t system, gyro, accel, mag = 0; // 0 = not calibrated, 3 = fully calibrated, system > 0 means found magnetic north
  imuSensor.getCalibration(&system, &gyro, &accel, &mag);
  status.imu_cal = (system << 6) + (gyro << 4) + (accel << 2) + mag; 

  // Diagnostics
  #if defined(SERIAL_SHOW)
  Serial.print("IMU Calibration: System = "); Serial.print(system); Serial.print(", Gyro = "); 
  Serial.print(gyro); Serial.print(", Accel = "); Serial.print(accel); Serial.print(", Mag = "); Serial.println(mag);
  Serial.print("IMU Temperature = "); Serial.print(store.imu_temp); Serial.println(" deg C");
  Serial.print("Euler angles = "); Serial.print(store.euler.x()); Serial.print(", "); 
  Serial.print(store.euler.y()); Serial.print(", "); Serial.print(store.euler.z()); Serial.println(" deg");
  Serial.print("Magnetic bearing = "); Serial.print(store.mag.x()); Serial.print(", "); 
  Serial.print(store.mag.y()); Serial.print(", "); Serial.print(store.mag.z()); Serial.println(" deg");
  #endif
}

/***************************** GPS *******************************************/
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS myGNSS;

// NAV PVT is 92 + 8 bytes in length (including the sync chars, class, id, length and checksum bytes)
#define PACKET_LEN 100 

// Callback: updateGPS will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallback
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void updateGPS(UBX_NAV_PVT_data_t ubxDataStruct);

void initGPS() {
  #if defined(AM_PART_APOLLO3)
  Wire2.setPullups(0); // On the Artemis, we can disable the internal I2C pull-ups too to help reduce bus errors
  #endif

  // NAV PVT messages are 100 bytes long.
  // In this example, the data will arrive no faster than one message per second.
  // So, setting the file buffer size to 2001 bytes should be more than adequate.
  // I.e. room for twenty messages plus an empty tail byte.
  myGNSS.setFileBufferSize(2001); // setFileBufferSize must be called _before_ .begin

  if (myGNSS.begin(Wire2) == false) //Connect to the u-blox module using Wire port
  {
    #if defined(SERIAL_SHOW)
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing..."));
    #endif // SERIAL_SHOW
    abort();
  }
  else {
    #if defined(SERIAL_SHOW)
    Serial.println(F("u-blox GNSS initialised!"));
    #endif // SERIAL_SHOW
  }

  // Uncomment the next line if you want to reset your module back to the default settings with 1Hz navigation rate
  // (This will also disable any "auto" messages that were enabled and saved by other examples and reduce the load on the I2C bus)
  //myGNSS.factoryDefault(); delay(5000);
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
  myGNSS.setNavigationFrequency( ceil(1000/GPS_REFRESH_TIME_MS) ); //Produce one navigation solution per second
  myGNSS.setAutoPVTcallback(&updateGPS); // Enable automatic NAV PVT messages with callback to printPVTdata
  myGNSS.logNAVPVT(); // Enable NAV PVT data logging

  run_later(checkGPS,INIT_TIME_MS);
}

void updateGPS(UBX_NAV_PVT_data_t ubxDataStruct) {
  status.gps = true;

  if ( !status.gps_lock && (store.numSV > 3) ) {
    status.gps_lock = true;
    run_later(initBlink,0);
    //run_later(initSD,0);
    // update the RTC to GPS time
    rtc.adjust(DateTime(ubxDataStruct.year, ubxDataStruct.month, ubxDataStruct.day, 
                        ubxDataStruct.hour, ubxDataStruct.min, ubxDataStruct.sec));
    if (!status.rtc_sync)
      elapsedTime = ubxDataStruct.iTOW%1000;
  }

  sprintf(store.gps_now,"%04d-%02d-%02d %02d:%02d:%02d.%03d",ubxDataStruct.year,ubxDataStruct.month,
          ubxDataStruct.day,ubxDataStruct.hour,ubxDataStruct.min,ubxDataStruct.sec,uint16_t(ubxDataStruct.iTOW%1000));
  store.latitude  = ubxDataStruct.lat;
  store.longitude = ubxDataStruct.lon; 
  //store.altitude  = ubxDataStruct.hMSL; // height above mean sea level (mm)
  store.altitude  = ubxDataStruct.height;  // Height above ellipsoid (mm)
  store.numSV     = ubxDataStruct.numSV;

  store.velN      = ubxDataStruct.velN;
  store.velE      = ubxDataStruct.velE;
  store.velD      = ubxDataStruct.velD;
  store.gSpeed    = ubxDataStruct.gSpeed;
  store.heading   = ubxDataStruct.headMot;

  store.velAcc    = ubxDataStruct.sAcc;
  store.pDOP      = ubxDataStruct.pDOP;
  store.hAcc      = ubxDataStruct.hAcc;
  store.vAcc      = ubxDataStruct.vAcc;
  store.headAcc   = ubxDataStruct.headAcc;

  store.magDec    = ubxDataStruct.magDec;
  store.magAcc    = ubxDataStruct.magAcc;

  // Diagnostics
  #if defined(SERIAL_SHOW)
  Serial.print("GPS time is "); Serial.println(store.gps_now);
  Serial.print("Longitude "); Serial.print(store.longitude); Serial.print(" deg * 10^-7, ");
  Serial.print("Latitude "); Serial.print(store.latitude); Serial.print(" deg * 10^-7, ");
  Serial.print("Altitude "); Serial.print(store.altitude); Serial.println(" mm");
  Serial.print("Number of Satellites "); Serial.println(store.numSV); Serial.print("\n");
  #endif // SERIAL_SHOW
}

void checkGPS() {
  run_later(checkGPS,GPS_REFRESH_TIME_MS/2);

  myGNSS.checkUblox();      // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks();  // Check if any callbacks are waiting to be processed.
}




/***************************** Air Sensor ************************************/

// Humidity, pressure, temperature sensor
#include <Wire.h>
#include "SparkFunBME280.h"
BME280 airSensor;

void initAirSensor() {
  if (airSensor.beginI2C() == false) //Begin communication over I2C
  {
    #if defined(SERIAL_SHOW)
    Serial.println("Air sensor did not respond. Please check wiring.");
    Serial.flush();
    #endif
    abort();
  }
  #if defined(SERIAL_SHOW)
  Serial.println("BME280 Air Sensor Initialised!");
  #endif
  //airSensor.setMode(MODE_SLEEP); //Sleep for now

  run_later(checkAir,INIT_TIME_MS);
}

void checkAir() {
  run_later(checkAir,AIR_REFRESH_TIME_MS);
  status.air = true;

  //airSensor.setMode(MODE_FORCED); //Wake up sensor and take reading
  //while(airSensor.isMeasuring() == false) ; //Wait for sensor to start measurment
  //while(airSensor.isMeasuring() == true) ; //Hang out while sensor completes the reading  
  //Sensor is now back asleep but we get get the data

  store.air_humidity = airSensor.readFloatHumidity();
  store.air_pressure = airSensor.readFloatPressure();
  store.air_temp     = airSensor.readTempC();

  // Diagnostics
  #if defined(SERIAL_SHOW)
  Serial.print("Air Relative Humidity = "); Serial.print(store.air_humidity,1); Serial.println("%");
  Serial.print("Air Pressure = "); Serial.print(store.air_pressure); Serial.println(" Pa");
  Serial.print("Air Temperature = "); Serial.print(store.air_temp,2); Serial.println(" deg C");
  #endif
}



/***************************** JETSON SYNC ***********************************/
void initJetsonSerial() {
  status.jetson_sync = false;
  pinMode(GPIO_JETSON,INPUT);

  Serial1.begin(921600);
  while (!Serial1); // wait for Serial to finish startup
  #if defined(SERIAL_SHOW)
  Serial.println(F("Serial port for Jetson initialised."));
  #endif
  run_later(updateJetson,INIT_TIME_MS);
}

void checkJetsonSerial() {
  run_later(checkJetsonSerial,JETSON_REFRESH_TIME_MS);

  // serial messages toggle sensor data transmits
  while ( Serial1.available() > 0 ) {
    char temp = Serial1.read();
    if (temp == 'S')
      status.jetson_sync = true;
    else if (temp == 'E')
      status.jetson_sync = false;
  }

  if ( status.jetson_sync ) { // == '\n') {
    if ( status.gps_lock && store.numSV > 3 ) { // && !status.jetson_sync)
      updateJetson();
    }
    else
      updateJetson();

    #if defined(SERIAL_SHOW)
    Serial.println(F("Jetson is now ready."));
    #endif
  }
  else {
    #if defined(SERIAL_SHOW)
    Serial.println(F("Jetson is NOT ready."));
    #endif
  }
}


void updateJetson() {
  run_later(updateJetson,JETSON_REFRESH_TIME_MS);

  if ( digitalRead(GPIO_JETSON) == HIGH && status.gps_lock) {
    status.jetson_sync = true;

    if (status.rtc) {
      Serial1.print(store.rtc_now); Serial1.print(",");
      Serial1.print(store.rtc_temp); Serial1.print(",");
    }
    else
      Serial1.print(",,");

    if (status.air) {
      Serial1.print(store.air_temp); Serial1.print(",");
      Serial1.print(store.air_pressure); Serial1.print(",");
      Serial1.print(store.air_humidity); Serial1.print(",");
    }
    else
      Serial1.print(",,,");

    if (status.imu) {
      Serial1.print(status.imu_cal); Serial1.print(",");
      Serial1.print(store.imu_temp); Serial1.print(",");
      Serial1.print(store.euler.x()); Serial1.print(",");
      Serial1.print(store.euler.y()); Serial1.print(",");
      Serial1.print(store.euler.z()); Serial1.print(",");

      Serial1.print(store.quat.w(),6); Serial1.print(","); 
      Serial1.print(store.quat.x(),6); Serial1.print(","); 
      Serial1.print(store.quat.y(),6); Serial1.print(",");
      Serial1.print(store.quat.z(),6); Serial1.print(",");

      Serial1.print(store.mag.x()); Serial1.print(",");
      Serial1.print(store.mag.y()); Serial1.print(",");
      Serial1.print(store.mag.z()); Serial1.print(",");
    }
    else
      Serial1.print(",,,,, ,,,, ,,,");

    if (status.gps) {
      Serial1.print(store.gps_now); Serial1.print(",");
      Serial1.print(store.latitude); Serial1.print(",");
      Serial1.print(store.longitude); Serial1.print(",");
      Serial1.print(store.altitude); Serial1.print(",");
      Serial1.print(store.numSV); Serial1.print(",");

      Serial1.print(store.velN); Serial1.print(",");
      Serial1.print(store.velE); Serial1.print(",");
      Serial1.print(store.velD); Serial1.print(",");
      Serial1.print(store.gSpeed); Serial1.print(",");
      Serial1.print(store.heading); Serial1.print(",");
      
      Serial1.print(store.velAcc); Serial1.print(",");
      Serial1.print(store.pDOP); Serial1.print(",");
      Serial1.print(store.hAcc); Serial1.print(",");
      Serial1.print(store.vAcc); Serial1.print(",");
      Serial1.print(store.headAcc); Serial1.print(",");
      
      Serial1.print(store.magDec); Serial1.print(",");
      Serial1.print(store.magAcc); Serial1.print("\n");
    }
    else
      Serial1.print(",,,,, ,,,,, ,,,,, ,\n");

    #if defined(SERIAL_SHOW)
    Serial.println("Jetson successfully updated.\n");
    #endif 
  
  }
  else {
    status.jetson_sync = false;
    #if defined(SERIAL_SHOW)
    Serial.println("Switch open. Skipping Jetson update.\n");
    #endif
  }

  // reset sensor read status
  status.rtc = false; status.imu = false; status.gps = false; status.air = false;
}



/***************************** SETUP and RUN *********************************/
void setup(){

  // Init Ports
  #if defined(SERIAL_SHOW)
  Serial.begin(115200);
  while (!Serial); // wait for Serial to finish startup
  Serial.println(F("schedule_sensors has started."));
  #endif // only save to SD card

  // regular I2C port
  Wire.begin();
  Wire.setClock(400000); 

  // GPS I2C port
  Wire2.begin();
  Wire2.setClock(100000); 

  // Init Sensors
  initOS();
  //initBlink(); // will auto start when everything is ready

  initStatusLEDs();
  initRTC();
  initIMU();
  initAirSensor();
  initGPS();
  initJetsonSerial();

  #if defined(SERIAL_SHOW)
  Serial.println("##### EVERYTHING HAS INITIALISED! #####\n");
  #endif
}

void loop(){  
  run();
}





/***************************** STRUCTS ***********************************/

#include "Adafruit_BNO055.h" // for imu::Vector<3> definitions
struct datastore_t {

  char  rtc_now[30];    // datetime64 string
  //float rtc_temp;       // deg C

  // timestamp
  float air_temp;       // deg C
  float air_pressure;   // hPa
  float air_humidity;   // relative humidity %

  //int8_t imu_temp;      // deg C
  //imu::Vector<3> euler; // deg
  // add 5 bytes 
  imu::Quaternion quat;
  //imu::Vector<3> mag;   // uT

  char gps_now[30];     // datetime64 string
  int32_t latitude;     // deg *10^-7
  int32_t longitude;    // deg *10^-7
  int32_t altitude;     // mm above ellipsoid
  uint8_t numSV;        // number of satellites in view (and used in compute)
  //int32_t velN;         // velocity in NED coordinates in mm/s
  //int32_t velE;
  //int32_t velD;
  int32_t gSpeed;       // ground speed in mm/s
  int32_t heading;      // heading of motion deg * 1e-5
  //uint32_t velAcc;      // Speed accuracy estimate: mm/s
  uint16_t pDOP;        // Position DOP * 0.01
  uint32_t hAcc;        // Horizontal accuracy estimate: mm
  uint32_t vAcc;        // Vertical accuracy estimate: mm
  uint32_t headAcc;     // Heading accuracy estimate (both motion and vehicle): deg * 1e-5
  //int16_t magDec;       // Magnetic declination: deg * 1e-2
  //uint16_t magAcc;      // Magnetic declination accuracy: deg * 1e-2

};
datastore_t store;

/* Sensor status to show on RGB status LED
 * R: all sensor init
 * G: GPS lock
 * B: IMU system calibration
 */
struct sensor_status_t {
  bool rtc = false;  
  bool imu = false;
  bool gps = false;
  bool air = false;
  bool rtc_sync = false;
  uint8_t imu_cal = 0;
  bool jetson_sync = false;
  bool gps_lock = false;
  bool sd_updating = false;
};
sensor_status_t status;


/***************************** Inertial Measurement Unit *********************/
//#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
Adafruit_BNO055 imuSensor = Adafruit_BNO055(55, 0x28);

void initIMU() {
  if (!imuSensor.begin())  {
    /* There was a problem detecting the BNO055 ... check your connections */
    _PP("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    abort();
  }
  _PL("BNO-055 IMU Initialised!");
  //imuSensor.setExtCrystalUse(true);
}

void updateIMU() {
  status.imu = true;

  store.imu_temp = imuSensor.getTemp();
  // access via store.euler.x(), y(), z()
  store.euler = imuSensor.getVector(Adafruit_BNO055::VECTOR_EULER);
  store.quat  = imuSensor.getQuat();
  store.mag   = imuSensor.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  //store.gyro  = imuSensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  //store.accel = imuSensor.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  uint8_t system, gyro, accel, mag = 0; // 0 = not calibrated, 3 = fully calibrated, system > 0 means found magnetic north
  imuSensor.getCalibration(&system, &gyro, &accel, &mag);
  status.imu_cal = (system << 6) + (gyro << 4) + (accel << 2) + mag; 

  _PP("IMU Calibration: System = "); _PP(system); _PP(", Gyro = "); 
  _PP(gyro); _PP(", Accel = "); _PP(accel); _PP(", Mag = "); _PL(mag);
  _PP("IMU Temperature = "); _PP(store.imu_temp); _PL(" deg C");
  _PP("Euler angles = "); _PP(store.euler.x()); _PP(", "); 
  _PP(store.euler.y()); _PP(", "); _PP(store.euler.z()); _PL(" deg");
  _PP("Magnetic bearing = "); _PP(store.mag.x()); _PP(", "); 
  _PP(store.mag.y()); _PP(", "); _PP(store.mag.z()); _PL(" deg");

}