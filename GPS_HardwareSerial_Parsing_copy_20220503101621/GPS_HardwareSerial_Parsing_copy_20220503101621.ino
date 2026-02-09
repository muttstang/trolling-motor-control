

// Trolling motor control code for driving Minnkota Powerdrive V1 12V motor

#include <Adafruit_GPS.h>
#include <HardwareSerial.h>
#include <math.h>
#include <wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>


//#include "AD520X.h"

// what's the name of the hardware serial port?
//#define GPSSerial Serial1
HardwareSerial GPSSerial(1);

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false
#define right_pin 7
#define left_pin 9
#define on_off_pin 11
#define speed_pin 8


uint32_t timer = millis();
//uint32_t start, stop;

//AD5204 pot(15, 255, 255, 16, 42);  // SW SPI

int initialize = 1;

double prev_lat = 46;
double prev_long = -97;
double latitude = 46;
double longitude = -97;
double wp_lat = 43.49681854;
double wp_long = -96.77335358;

void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  GPSSerial.begin(115200, SERIAL_8N1, 13, 12);
  Serial.begin(115200);
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // Ask for firmware version
//  GPSSerial.println(PMTK_Q_RELEASE);
}

void loop() // run over and over again
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Latitude: ");
      latitude = gps_convert_deg(GPS.latitude);
      if (GPS.lat == 'S') {latitude = -1 * latitude;}
      Serial.print(latitude, 8); Serial.println(GPS.lat);
      
      Serial.print("Longitude: ");
      longitude = gps_convert_deg(GPS.longitude);
      if (GPS.lon == 'W') {longitude = -1 * longitude;}
      Serial.print(longitude, 8); Serial.println(GPS.lon);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      //Serial.print("Angle: "); Serial.println(GPS.angle);
      //Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      float waypoint_distance = calc_offset(latitude, wp_lat, longitude, wp_long);
      Serial.print("Waypoint Distance: "); Serial.println(waypoint_distance);
    }
  }

  prev_lat = latitude;
  prev_long = longitude;
  
}


double gps_convert_deg(float gps_value) 
{
  int gps_val_int = gps_value / 100;  // parse out degrees
  double gps_decimal = gps_val_int + (gps_value - gps_val_int * 100) / 60; // convert remaining minutes portion to degrees and add to degrees
  return gps_decimal;
}

// calculate the latitude offset from waypoint in feet
float lat_offset(double _curLat, double _wLat){ 
  float latitude_offset = 730445.49 * (_curLat - _wLat);
  Serial.print("Latitude offset: "); Serial.println(latitude_offset);
  return latitude_offset;
}

// calculate the longitude offset from waypoint in feet
float long_offset(double _curLat, double _curLong, double _wLong){ 
  float _long_offset = 730445.49 * cos(_curLat/57.295779)* (_curLong - _wLong);
  Serial.print("Longitude offset: "); Serial.println(_long_offset);
  return _long_offset;
}

// calculate the surface distance from the waypoint in feet
float calc_offset(double _curLat, double _wLat, double _curLong, double _wLong){ 
  float latitude_offset = lat_offset(_curLat, _wLat);
  float longitude_offset = long_offset(_curLat, _curLong, _wLong);
  float position_offset = sqrt(latitude_offset*latitude_offset + longitude_offset*longitude_offset);
  float ang_offset = angle_offset(latitude_offset, longitude_offset);
  return position_offset;
}

float angle_offset(float _lat_offset, float _long_offset){
  float _angle;
  if (_long_offset == 0)
  {
    if (_lat_offset >= 0) 
    {
      _angle = 0;  // if there is no longitude offset then if the lat offset is positive, the angle = 0
    }        
    else 
    {
      _angle = 180; // if there is no longitude offset then if the lat offset is negative, the angle is 180
    }
  }
  else 
  {
      _angle = atan(_lat_offset/_long_offset) * 180 / 3.14159;  // calculate angle offset between current location and waypoint
  }    
  if (_angle < 0) {_angle = 360 + _angle;}  // make sure that the angle is between 0 and 360
  Serial.print("Offset angle to waypoint: "); Serial.println(_angle);
  return _angle;
}

float motor_heading(float _val){
  float value;
  return value;
}

float boat_heading(float _val){
  float value;
  return value;
}

float motor_power(float dist){
  float power = 0;
  return power;
}

float motor_turn(float angle, float distance){
  float turn_time = 0;
  return turn_time;
}

float off_path_dist(float in1, float in2, float in3, float in4){
  float dist_from_path = 0;
  return dist_from_path;
}

void set_motor_power(float dist){
  // make sure motor is on
  // set the digital potentiometer value
}

void turn_motor(float dir, float time1, float angle){
  // check angle
  // 
  // set end time
  // turn motor the correct direction
  // check time until end time reached
  // stop turning motor

}


