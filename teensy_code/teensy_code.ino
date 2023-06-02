/* Author: Yiwei Mao
 * Date: 2021-10-28
 * File: sensor_board.ino
 */


/* Table of Contents for Code
 * Configuration
 * Structs
 * Task Scheduler
 * Status LEDs
 * RTC functions
 * IMU functions
 * GPS functions
 * Pressure/Humidity functions
 * Setup and Run
 */


// Debug options
//#define _DEBUG_

#ifdef _DEBUG_
#define _PP(a) Serial.print(a);
#define _PL(a) Serial.println(a);
#else
#define _PP(a)
#define _PL(a)
#endif // _DEBUG_


/***************************** CONFIGURATION *****************************/

#define BLINK_RATE_HZ       1
#define RTC_REFRESH_RATE_HZ 100
#define AIR_REFRESH_RATE_HZ 100
#define GPS_REFRESH_RATE_HZ 1
#define SERIAL_REFRESH_RATE_HZ 100
#define IMU_REFRESH_RATE_HZ 100
#define XBEE_REFRESH_RATE_HZ 1
#define BUTTON_REFRESH_RATE_HZ 3

/***************************** TASK SCHEDULER *****************************/

// #define _TASK_TIMECRITICAL      // Enable monitoring scheduling overruns
#define _TASK_SLEEP_ON_IDLE_RUN // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass
#define _TASK_STATUS_REQUEST    // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
// #define _TASK_WDT_IDS           // Compile with support for wdt control points and task ids
// #define _TASK_LTS_POINTER       // Compile with support for local task storage pointer
// #define _TASK_PRIORITY          // Support for layered scheduling priority
// #define _TASK_MICRO_RES         // Support for microsecond resolution
// #define _TASK_STD_FUNCTION      // Support for std::function (ESP8266 and ESP32 ONLY)
// #define _TASK_DEBUG             // Make all methods and variables public for debug purposes
// #define _TASK_INLINE            // Make all methods "inline" - needed to support some multi-tab, multi-file implementations
// #define _TASK_TIMEOUT           // Support for overall task timeout
// #define _TASK_OO_CALLBACKS      // Support for dynamic callback method binding
#include "TaskScheduler.h"

Scheduler ts;

void blink_cb();
Task t_blink ( 500 / BLINK_RATE_HZ * TASK_MILLISECOND, TASK_FOREVER, &blink_cb, &ts, true );

void blink_cb() {
  if ( t_blink.isFirstIteration() ) {
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,HIGH);
    _PL("LED blink initialised!");
  }
  
  digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
}

/***************************** STRUCTS ***********************************/

struct datastore_t {

  char header = '*';
  uint8_t rtc_status = 0;
  uint8_t imu_status = 0;
  uint8_t air_status = 0;
  uint8_t gps_status = 0;

  uint16_t rtc_year = 2022;
  uint8_t rtc_month = 02;
  uint8_t rtc_day   = 25;
  uint8_t rtc_hour  = 23;
  uint8_t rtc_minute= 13;
  uint8_t rtc_second= 2;
  uint16_t rtc_ms   = 3;

  int16_t air_timeoffset; // ms from stored rtc time
  float air_temp;       // deg C
  float air_pressure;   // hPa
  float air_humidity;   // relative humidity %

  int16_t imu_timeoffset; // ms from stored rtc time
  uint8_t imu_calibration; // in binary 2 bits each (0-3 decimal) for system, gyro, accel, mag respectively
  float quat_w; // quaternions
  float quat_x;
  float quat_y;
  float quat_z;

  int16_t gps_timeoffset; // ms from stored rtc time
  int32_t latitude;     // deg *10^-7
  int32_t longitude;    // deg *10^-7
  int32_t altitude;     // mm above ellipsoid
  uint8_t numSV;        // number of satellites in view (and used in compute)
  uint16_t pDOP;        // position dilution of precision
  char footer = '\n';
};
datastore_t store;

#define GPIO_SWITCH 21
#define SWITCH_LED  20

/***************************** Real Time Clock *******************************/

#include <Wire.h>
#include "elapsedMillis.h"
#include "RTClib.h"
RTC_DS3231 rtc; // I2C address 0x68
elapsedMillis elapsedTime;
elapsedMillis rtc_offset;
bool rtc_sync = false; bool gps_lock = false;
int16_t time_diff = 0;

bool rtc_init();
void rtc_cb();
Task t_rtc ( 1000 / RTC_REFRESH_RATE_HZ * TASK_MILLISECOND, TASK_FOREVER, &rtc_cb, &ts, false, &rtc_init);

bool rtc_init() {
  if (!rtc.begin(&Wire1)) {
    _PL("RTC not found");
    return false;
  }
  if (rtc.lostPower()) {
    _PL("RTC lost power. Setting the time as time at compilation.");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  rtc_sync = false;
  _PL("RTC (DS3231) Real Time Clock Initialised!");
  return true;
}

void rtc_cb() {
  if (store.rtc_status == 1) {
    time_diff = rtc_offset;
    store.air_timeoffset -= time_diff;
    store.gps_timeoffset -= time_diff;
    store.imu_timeoffset -= time_diff;
  }
  store.rtc_status = 1;

  DateTime now = rtc.now();
  rtc_offset = 0;
  
  store.rtc_year  = now.year();
  store.rtc_month = now.month();
  store.rtc_day   = now.day();
  store.rtc_hour  = now.hour();
  store.rtc_minute= now.minute();
  store.rtc_second= now.second();
  store.rtc_ms    = uint16_t(elapsedTime%1000);

  if ( !rtc_sync && gps_lock && elapsedTime >= 1000) {
    rtc_sync = true;
    elapsedTime = elapsedTime%1000;
    rtc.adjust(now.unixtime()+1);
  }
  
  #if defined(_DEBUG_)
  char rtc_now[30];
  //rtc_now = now.toString("YYYY-MM-DD hh:mm:ss");
  sprintf(rtc_now,"%04d-%02d-%02d %02d:%02d:%02d.%03d",now.year(),now.month(),
                        now.day(),now.hour(),now.minute(),now.second(),uint16_t(elapsedTime%1000));
  _PP("RTC Time = "); _PL(rtc_now);
  #endif // _DEBUG_
  
}

/***************************** AIR SENSOR *********************************/

#include "Adafruit_BME280.h"

Adafruit_BME280 bme;

bool air_init();
void air_cb();
Task t_air ( 1000 / AIR_REFRESH_RATE_HZ * TASK_MILLISECOND, TASK_FOREVER, &air_cb, &ts, false, &air_init);

bool air_init() {
  if (!bme.begin(0x77,&Wire1)) {
    _PL("BME280 sensor not found");
    return false;
  }
  _PL("Air sensor initialised!");
  return true;
}
void air_cb() {
  store.air_status     = 1; 
  store.air_timeoffset = rtc_offset;

  store.air_temp     = bme.readTemperature();
  store.air_pressure = bme.readPressure() / 100.0F;
  store.air_humidity = bme.readHumidity();

  _PP("AIR time offset from RTC "); _PP(store.air_timeoffset); _PL(" ms");
  _PP("Temp = ");     _PP(store.air_temp);     _PL(" °C");
  _PP("Pressure = "); _PP(store.air_pressure); _PL(" hPa");
  _PP("Humidity = "); _PP(store.air_humidity); _PL(" %");
}

/***************************** Inertial Measurement Unit *********************/

#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
Adafruit_BNO055 imuSensor = Adafruit_BNO055(55, 0x28, &Wire1);

bool imu_init();
void imu_cb();
Task t_imu ( 1000 / IMU_REFRESH_RATE_HZ * TASK_MILLISECOND, TASK_FOREVER, &imu_cb, &ts, false, &imu_init);

bool imu_init() {
  if (!imuSensor.begin()) {
    _PL("There was a problem detecting the BNO055 ... check your connections");
    return false;
  }
  _PL("BNO-055 IMU Initialised!");
  //imuSensor.setExtCrystalUse(true);
  return true;
}
void imu_cb() {
  store.imu_status     = 1;
  store.imu_timeoffset = rtc_offset;

  imu::Quaternion quat = imuSensor.getQuat();
  store.quat_w = quat.w();
  store.quat_x = quat.x();
  store.quat_y = quat.y();
  store.quat_z = quat.z();

  uint8_t system, gyro, accel, mag = 0; // 0 = not calibrated, 3 = fully calibrated, system > 0 means found magnetic north
  imuSensor.getCalibration(&system, &gyro, &accel, &mag);
  store.imu_calibration = (system << 6) + (gyro << 4) + (accel << 2) + mag; 

  _PP("IMU time offset from RTC "); _PP(store.imu_timeoffset); _PL(" ms");
  _PP("IMU Calibration: System = "); _PP(system); _PP(", Gyro = "); 
  _PP(gyro); _PP(", Accel = "); _PP(accel); _PP(", Mag = "); _PL(mag);
  _PP("IMU Temperature = "); _PP(imuSensor.getTemp()); _PL(" deg C");
  #if defined(_DEBUG_)
  //store.imu_temp = imuSensor.getTemp();
  imu::Vector<3> euler = imuSensor.getVector(Adafruit_BNO055::VECTOR_EULER);
  _PP("Euler angles = "); _PP(euler.x()); _PP(", "); 
  _PP(euler.y()); _PP(", "); _PP(euler.z()); _PL(" deg");
  //_PP("Magnetic bearing = "); _PP(store.mag.x()); _PP(", "); 
  //_PP(store.mag.y()); _PP(", "); _PP(store.mag.z()); _PL(" deg");
  #endif // _DEBUG_
}


/***************************** GPS *******************************************/
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
SFE_UBLOX_GNSS myGNSS; // I2C address 0x42

// NAV PVT is 92 + 8 bytes in length (including the sync chars, class, id, length and checksum bytes)
#define PACKET_LEN 100 

// Callback: gps_update will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallback
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void gps_update(UBX_NAV_PVT_data_t ubxDataStruct);
bool gps_init();
void gps_cb();
Task t_gps ( 1000 / GPS_REFRESH_RATE_HZ * TASK_MILLISECOND, TASK_FOREVER, &gps_cb, &ts, false, &gps_init);

bool gps_init() {
  //myGNSS.enableDebugging();

  // NAV PVT messages are 100 bytes long.
  // In this example, the data will arrive no faster than one message per second.
  // So, setting the file buffer size to 2001 bytes should be more than adequate.
  // I.e. room for twenty messages plus an empty tail byte.
  myGNSS.setFileBufferSize(20001); // setFileBufferSize must be called _before_ .begin

  if (myGNSS.begin(Wire) == false) { //Connect to the u-blox module using Wire port. Can use &Wire2
    _PL("u-blox GNSS not detected at default I2C address. Please check wiring.");
    return false;
  }
  _PL("u-blox GNSS initialised!");

  // Uncomment the next line if you want to reset your module back to the default settings with 1Hz navigation rate
  // (This will also disable any "auto" messages that were enabled and saved by other examples and reduce the load on the I2C bus)
  //myGNSS.factoryDefault(); delay(5000);
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
  myGNSS.setNavigationFrequency( GPS_REFRESH_RATE_HZ ); //Produce one navigation solution per second
  //myGNSS.setAutoPVTcallback(&gps_update); // Enable automatic NAV PVT messages with callback to printPVTdata
  myGNSS.logNAVPVT(); // Enable NAV PVT data logging

  return true;
}
void gps_cb() {
  // Calling getPVT returns true if there actually is a fresh navigation solution available.
  // Start the reading only when valid LLH is available
  if (myGNSS.getPVT(10) && (myGNSS.getInvalidLlh() == false)) {

    store.gps_status     = 1;
    store.gps_timeoffset = rtc_offset;

    if ( !gps_lock && (store.numSV > 3) ) {
      gps_lock = true;

      // update the RTC to GPS time
      rtc.adjust(DateTime(myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay(), 
                          myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond()));
      if (!rtc_sync)
        elapsedTime = myGNSS.getMillisecond();
    }
    if (store.numSV > 3  && (ts.timeUntilNextIteration(t_blink) > 1000 / store.numSV ) ) {
      t_blink.setInterval( 1000 / store.numSV * TASK_MILLISECOND ); // blink frequency match number of satellites found 
      t_blink.restart();
    }

    store.latitude  = myGNSS.getLatitude();
    store.longitude = myGNSS.getLongitude(); 
    store.altitude  = myGNSS.getAltitudeMSL(); // height above mean sea level (mm)
    //store.altitude  = ubxDataStruct.height;  // Height above ellipsoid (mm)
    store.numSV     = myGNSS.getSIV();
    store.pDOP      = myGNSS.getPDOP();

    /* unused but could be useful? 
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
    */

    #if defined(_DEBUG_)
    char gps_now[30];
    sprintf(gps_now,"%04d-%02d-%02d %02d:%02d:%02d.%03d",myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay(), myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond(),myGNSS.getMillisecond());
    _PP("GPS time is "); _PL(gps_now);
    #endif // _DEBUG_
    _PP("GPS time offset from RTC "); _PP(store.gps_timeoffset); _PL(" ms");
    _PP("Longitude "); _PP(store.longitude); _PP(" deg * 10^-7, ");
    _PP("Latitude "); _PP(store.latitude); _PP(" deg * 10^-7, ");
    _PP("Altitude "); _PP(store.altitude); _PL(" mm");
    _PP("Number of Satellites "); _PL(store.numSV); 
  }
  else 
    _PP(".");
}

/********************************* XBEE ***************************************/

bool xbee_init();
void xbee_cb();
Task t_xbee ( 1000 / XBEE_REFRESH_RATE_HZ * TASK_MILLISECOND, TASK_FOREVER, &xbee_cb, &ts, false, &xbee_init);
int xbee_counter = 0;
bool rpi_ready = false;

bool xbee_init() {
  Serial3.begin(115200);
  delay(10); // wait for Serial to finish startup
  if (!Serial3) {
    _PL("Xbee failed to initialise.");
    return false;
  }
  _PL("Xbee initialised!");
  return true;
}
template <typename T>
void send_xbee_data(T x) {
  byte * data = (byte *) &x;
  for (uint16_t i = 0; i < sizeof(x); i++) {
    Serial3.write(data[i]);
  }
}
void xbee_cb() {  
  _PL("Data sent to XBee");
  //send_xbee_data(store.header);
  send_xbee_data(store.latitude); send_xbee_data(store.longitude); 
  send_xbee_data(store.air_temp); send_xbee_data(store.air_pressure); send_xbee_data(store.air_humidity);
  send_xbee_data(store.numSV); send_xbee_data(rpi_ready); send_xbee_data(store.imu_calibration);
  send_xbee_data(store.footer);
}

/***************************** JETSON/RPi SYNC ***********************************/

bool serial_data_init();
void serial_cb();
Task t_serial ( 1000 / SERIAL_REFRESH_RATE_HZ * TASK_MILLISECOND, TASK_FOREVER, &serial_cb, &ts, false, &serial_data_init);

template <typename T>
void send_data(T x) {
  byte * data = (byte *) &x;
  for (uint16_t i = 0; i < sizeof(x); i++) {
    Serial1.write(data[i]);
  }
}
bool serial_data_init() {
  Serial1.begin(921600);
  delay(10); // wait for Serial to finish startup
  if (!Serial1) {
    _PL("Serial failed to initialise.");
    return false;
  }
  _PL("Serial port initialised!");
  return true;
}
void serial_cb() {
  if(digitalRead(GPIO_SWITCH)) {
    send_data(store);
    store.rtc_status = 0;
    store.imu_status = 0;
    store.air_status = 0;
    store.gps_status = 0;
    //store.xbee_status= 0;
  }
  _PL("Packet sent!");
}

/***************************** LED BUTTON *****************************************/

// A latching button, blink when pressed waiting for RPi4 to be ready
// when RPi4 sends ready message, make solid on
// after switch is off, it blinks until RPi4 ends collect

bool led_button_init();
void led_button_cb();
Task t_button ( 1000 / BUTTON_REFRESH_RATE_HZ * TASK_MILLISECOND, TASK_FOREVER, &led_button_cb, &ts, false, &led_button_init);
char serial_buff = 0;

bool led_button_init() {
  pinMode(GPIO_SWITCH,INPUT);
  pinMode(SWITCH_LED,OUTPUT);

  _PL("LED button initialised!");
  return true;
}
void led_button_cb() {
  while (Serial1.available() > 0) {
    serial_buff = Serial1.read();
    if (serial_buff == 'y')
      rpi_ready = true;
    else
      rpi_ready = false;
  }

  if(digitalRead(GPIO_SWITCH)) {
    if (!rpi_ready)
      digitalWrite(SWITCH_LED,(byte)(t_button.getRunCounter()%2));
    else
      digitalWrite(SWITCH_LED,HIGH);
  }
  else { // switch is off
    if (rpi_ready)
      digitalWrite(SWITCH_LED,(byte)(t_button.getRunCounter()%2));
    else
      digitalWrite(SWITCH_LED,LOW);
  }
}




/***************************** RESET *****************************************/
//from https://forum.pjrc.com/threads/52512-External-RESET-button-Teensy-3-2
void reset_init() {
  pinMode(22,INPUT_PULLUP);
  attachInterrupt(22, pin_reset, FALLING);
}
void pin_reset() {
  SCB_AIRCR = 0x05FA0004;
}

/***************************** SETUP and RUN *********************************/
void setup(){

  // Init Ports
  #if defined(_DEBUG_)
  Serial.begin(115200);
  while (!Serial); // wait for Serial to finish startup
  _PL(F("sensor board has started."));
  #endif // _DEBUG_

  reset_init();

  // GPS is connected to SDA/SCL, everything else is on SDA1/SCL1
  Wire.begin();
  Wire.setClock(400000); 
  Wire1.begin();
  Wire1.setClock(400000);

  t_rtc.enable();
  t_air.enable();
  t_imu.enable();
  t_gps.enable();
  t_serial.enable();
  t_xbee.enable();
  t_button.enable();
}

void loop() {
  ts.execute();
}
