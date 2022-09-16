// Trolling motor control code for driving Minnkota Powerdrive V1 12V motor

#include <Adafruit_GPS.h>
#include <HardwareSerial.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_SH110X.h>
#include <splash.h>
#include <PID_v1.h>

// define output pins
#define left_turn_pin     7
#define right_turn_pin    9
#define output_power_pin  11

// define input pins
#define menu_pin          1
#define power_button_pin  10
#define exit_pin          2
#define Kp_power_pin      6
#define Ki_power_pin      5
#define Kp_turn_pin       4
#define Ki_turn_pin       17
#define Kp_speed_pin      18
#define Ki_speed_pin      3

#define GPSECHO false // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);    // initialize the display

uint8_t broadcastAddress[] = {0x84, 0xF7, 0x03, 0xD8, 0xF2, 0x1A};    // Define receiver MAC address

typedef struct struct_incoming {    // STRUCTURE to receive data
  int manual;
  int nav;
  int anchor;
  int power;
  int speed_up;
  int slow_down;
  int left;
  int right;
  int hold_left;
  int hold_right;
  int sleep;
} struct_incoming;

typedef struct struct_outgoing {    // structure for sending data
  float wp_dist = 0;
  float wp_angle = 0;
  int boat_compass = 0;
  int motor_compass = 0;
  int gps_fix = 0;
  int satellites = 0;
  float boat_speed = 0;
  float set_speed = 0;
  int motor_power = 0;
  bool power_state = 0;
} struct_outgoing;

// create variables to store incoming data
int in_manual = 0;        // manual control button press
int in_nav = 0;         // nav button press
int in_anchor = 0;      // anchor button press
int in_power = 0;       // power button press
int in_speed_up = 0;    // speed up button press
int in_slow_down = 0;   // slow down button press
int in_left = 0;        // left button press
int in_right = 0;       // right button presst
int in_hold_left = 0;      // for holding a left button press
int in_hold_right = 0;     // for holding a right button press
int in_sleep = 0;       // for holding the screen sleep state

String current_mode = "off";    // keep track of the current mode
String turn_direction = "left"; // set the turn direction

// define local variables used in sending data and making calculations
double waypoint_distance = 0;  // calculated distance from current location to waypoint. this is the position error in anchor mode
float waypoint_angle = 0;     // calculated angle from current location to waypoint in gps coordinates
float gps_angle = 0;          // this is the heading according to GPS
int compass_boat = 0;         // direction that the boat is facing according to boat compass
int compass_motor = 0;        // direction that the motor is facing according to the motor compass
int boat_motor_angle = 0;     // angle between motor and boat
int ang = 0;                  // integer angle without sign for calling motor turn
double boat_speed = 0;         // speed of the boat according to the GPS
double dist_to_path = 0;


bool sendData = true;         // logic for sending data over ESP-now to pendant
bool powerState = false;      // logic for turning motor power on/off
bool sendError = false;       // logic for adding a note on screen if there is a send error


String success;               // Variable to store if sending data was successful
String turn_dir = "right";                 // for setting motor turn direction

double gps_timer = millis();          // for setting gps timer to get readings from gps on a regular interval
float  gps_delay = 2000;              // time between gps readings
double screen_sleep_timer = millis(); // for screen sleep timer
float  screen_sleep_delay = 300000;   // time for screen to go to sleep if no input is made
double esp_send_timer = millis();     // for setting intervals to send data to pendant
float  esp_send_delay = 250;          // Time between sending data to pendant in milliseconds
double wp_move_timer = millis();      // This is  the reset variable for waypoint moves
float  wp_move_delay = 2000;          // time between waypoint moves in milliseconds

double prev_lat = 46;             // initialize value for previous latitude taken to determine if it has changed
double prev_long = -97;           // initialize value for previous longitude taken to determine if it has changed
double latitude = 46;             // this will be the current latitude
double longitude = -97;           // this will be the current longitude
double latitude_offset = 0;
double long_offset = 0;
double wp_lat = 43.49681854;      // initialize the waypoint latitude variable
double wp_long = -96.77335358;    // initialize the waypoint longitude variable
double set_speed = 0;             // desired speed
double set_motor_angle = 0;       // desired angle from PID (pathPID output)
double turn_angle = 0;            // distance to turn the motor
double set_power = 0;             // power set for PID control
float turn_coef = 45;             // deg/second: amount motor turns per second
float max_speed = 2;              // set the maximum speed of the trolling motor

double Kp_power_max = 5;         // maximum power proportional coefficient
double Ki_power_max = .5;          // maximum power integral coefficient
double Kp_turn_max = 2;           // maximum path offset proportional coefficient
double Ki_turn_max = .5;           // maximum path offset integral coefficient
double Kp_speed_max = 100;         //  speed proportional coefficient
double Ki_speed_max = 1;          //  speed integral coefficient


double Kp_power = 10;             //  power proportional coefficient
double Ki_power = 1;              //  power integral coefficient
double Kd_power = 0;              //  power derivative coefficient
double Kp_turn = 10;              //  path offset proportional coefficient
double Ki_turn = 1;               //  path offset integral coefficient
double Kd_turn = 0;               //  path offset derivative coefficient
double Kp_speed = 10;             //  speed proportional coefficient
double Ki_speed = 1;              //  speed integral coefficient
double Kd_speed = 0;              //  speed derivative coefficient

float deg_to_rad = 0.01745;       // for converting degrees to radians
int motor_boat_angle = 0;         // angle difference between the motor and the boat
int max_motor_angle_nav = 45;     // maximum angle delta between motor and boat when tracking a path
int max_motor_angle_anchor = 120; // maximum angle delta between motor and boat when spot locked
int max_turn_angle = 0;           // to hold the maximum motor angle location to avoid over rotating motor
int min_turn_angle = 0;           // to hold the minimum motor angle location to avoid over rotating motor
int boat_wp_angle = 0;            // Angle between boat compass angle and waypoint angle for calculating motor turns


float boat_compass_offset = 0;    // correction factor for boat compass readings
float motor_compass_offset = 0;   // correction factor for motor compass readings

//double Input = 0;               // temporary values so PID doesn't crash on compile
//double Output = 0;              // temporary values so PID doesn't crash on compile
//double Setpoint = 0;            // temporary values so PID doesn't crash on compile
double wp_home = 0;               // drive the waypoint distance to zero with PID
double path_distance = 0;         // drive the distance from the path to zero with PID
int max_path_turn_angle = 60;     // maximum angle to turn when following a path

// set up PID tuning functions
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID speedPID(&boat_speed, &set_power, &set_speed, Kp_speed, Ki_speed, Kd_speed, DIRECT);
PID pathPID(&dist_to_path, &set_motor_angle, &path_distance, Kp_turn, Ki_turn, Kd_turn, DIRECT);
PID powerPID(&waypoint_distance, &set_power, &wp_home, Kp_power, Ki_power, Kd_power, REVERSE);

struct_outgoing gpsDataOut;   // Create a struct_message called myData to send to base unit

struct_incoming pendantData;    // create a struct_message to receive from base unit

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //  Serial.print("\r\nLast Packet Send Status:\t");
  //  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    success = "Delivery Success :)";
  }
  else {
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&pendantData, incomingData, sizeof(pendantData));
//    Serial.print("Bytes received: ");
//    Serial.println(len);

  in_manual = pendantData.manual;
  in_nav = pendantData.nav;
  in_anchor = pendantData.anchor;
  in_power = pendantData.power;
  in_speed_up = pendantData.speed_up;
  in_slow_down = pendantData.slow_down;
  in_left = pendantData.left;
  in_right = pendantData.right;
  in_hold_left = pendantData.hold_left;
  in_hold_right = pendantData.hold_right;
  in_sleep = pendantData.sleep;

//  if(in_hold_left) { Serial.println("holding left button (on data receive)");
//  if(in_hold_right) { Serial.println("holding right butoon (on data receive)");

  check_buttons();  // do what's needed with button press
  screen_sleep_timer = millis();  // reset the screen sleep timer
  
}

HardwareSerial GPSSerial(1);    // create hardware serial instance for GPS

Adafruit_GPS GPS(&GPSSerial);   // Connect to the GPS on the hardware port

Adafruit_BNO055 boat_dir = Adafruit_BNO055(55, 0x28);   // create boat compass object
Adafruit_BNO055 motor_dir = Adafruit_BNO055(55, 0x29);  // create motor compass object

void setup()
{
  //turn the PID on
  powerPID.SetMode(AUTOMATIC);
  powerPID.SetOutputLimits(0, 100);
  pathPID.SetMode(AUTOMATIC);
  pathPID.SetOutputLimits(-max_path_turn_angle, max_path_turn_angle);
  speedPID.SetMode(AUTOMATIC);
  speedPID.SetOutputLimits(0, 100);

  // shut off all relays on board here
  pinMode(left_turn_pin, OUTPUT);
  pinMode(right_turn_pin, OUTPUT);
  pinMode(output_power_pin, OUTPUT);
  pinMode(menu_pin, INPUT);
  pinMode(power_button_pin, INPUT);
  pinMode(exit_pin, INPUT);

  digitalWrite(left_turn_pin, 0);
  digitalWrite(right_turn_pin, 0);
  digitalWrite(output_power_pin, 0);

  Wire.begin(); // start wire for I2C communication with AD5273 digital potentiometer
  delay(100);
  set_motor_power(0); // initialize motor power at zero volts

  // start digital compasses
  boat_dir.begin();
  motor_dir.begin();
  display.begin(0x3C, true); // address 0x3C by default for OLED
  delay(1000);
  display.display();
  delay(1000);

  display.clearDisplay();
  display.display();

  display.setRotation(1);   // set screen rotation to 128x64 standard

  GPSSerial.begin(115200, SERIAL_8N1, 13, 12);  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  GPS.begin(9600);  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status, comment out to keep quiet

  WiFi.mode(WIFI_STA);    // Set device as a Wi-Fi Station

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  delay(1000);

}


//****************************************************************************
//****************************************************************************

void loop() // run over and over again
{
  char c = GPS.read();  // read data from the GPS in the 'main loop'

  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, querry GPS for data
  if (millis() - gps_timer > gps_delay) {
    get_GPS();

  }

  // put ESP send function within a timer reset so we don't send data too often
  if (millis() - esp_send_timer > esp_send_delay) {
    compass_output();
    send_data_to_pendant();
    update_pid_coef();
  }
  if (powerState) {
    digitalWrite(output_power_pin, 1);  // motor power is on
    if (current_mode == "anchor") {
      if (millis() - wp_move_timer > wp_move_delay) {
        // do spotlock stuff
        if (waypoint_distance > 10) { // if the waypoint distance is over 10 feet then do spotlock stuff
          // calculate motor turn
          calc_anchor_motor_turn();
          // calculate power, a function of waypoint distance
          calc_anchor_motor_power();
        }
        // if spot lock then set speed to zero
        wp_move_timer = millis(); // reset timer
      }
    }
    else if (current_mode == "nav") {
      if (millis() - wp_move_timer > wp_move_delay) {
        // call nav mode calculations
        // calculate distance from the path
        dist_to_path = waypoint_distance * sin((turn_angle - waypoint_angle) * deg_to_rad);
        // calculate power
        calc_nav_motor_power();
        // update path
        calc_nav_motor_turn();
        // send motor power via digital pot
        set_motor_power(set_power);

        wp_move_timer = millis(); // reset timer
      }
    }
    else if (current_mode == "manual") {
      double turn_timer = millis();
      while (in_hold_left == 1 && millis() - turn_timer < 2000) {
        digitalWrite(left_turn_pin, 1);   // turn on left turn relay
        delay(100);
      }
      digitalWrite(left_turn_pin, 0);   // turn off left turn relay
      while (in_hold_right == 1 && millis() - turn_timer < 2000) {
        digitalWrite(right_turn_pin, 1);   // turn on right turn relay
        delay(100);
      }
      digitalWrite(right_turn_pin, 0);   // turn off right turn relay
      if (millis() - wp_move_timer > wp_move_delay) {
        // update motor power to maintain desired speed
        speedPID.SetTunings(Kp_speed, Ki_speed, Kd_speed);
        speedPID.Compute();
        // send motor power via digital pot
        set_motor_power(set_power);
        wp_move_timer = millis(); // reset timer
      }
    }
  }
  else {
    digitalWrite(output_power_pin, 0);  // Shut off motor power
    // set speed to zero
  }

  prev_lat = latitude;
  prev_long = longitude;

  update_display();

  delay(100);


}

//****************************************************************************
//****************************************************************************

void check_buttons() {
  if (in_manual) {// Manual mode
    // do manual mode stuff
    // check for button press
    current_mode = "manual";
    in_manual = 0;
  }
  else if (in_nav) { // path tracking mode
    // do path tracking stuff
    set_path(); // set the path when button is pressed
    current_mode = "nav";
    in_nav = 0;
    powerState = true;  // turn power on if nav mode activated
    // check if need to adjust path
  }
  else if (in_anchor) {// spot lock
    // do spot lock stuff
    set_waypoint(); // should not need to press button again
    in_anchor = 0;  // reset after button press is registered
    // when first going into spot lock:
    // (use boolean to switch here so it calls this function or call it right from here? )
    set_motor_power(0); // set motor power to zero
    // if speed > .5 then turn motor to face backwards,
    if (boat_speed > 0.5) {
      powerState = true;
      // calculate angle to turn motor
      calc_boat_motor_angle();
      if (boat_motor_angle <= 0) {
        ang = 180 + boat_motor_angle;
        turn_dir = "right";
      }
      else {
        ang = 180 - boat_motor_angle;
        turn_dir = "left";
      }
      turn_motor(turn_dir, ang);
      set_motor_power(75); // set motor power to 75 percent
      double reverse_timeout = millis();
      while (boat_speed >= .1 && millis() - reverse_timeout < 6000) {
        // check speed
        get_GPS();
        delay(1000);
      }
      set_motor_power(0); // turn off motor
    }
    //  run motor until speed < 0,
    //  then turn motor forward again
    set_power = 0;
    powerState = true;  // turn power on if anchor mode activated
    current_mode = "anchor";
  }
  else if (in_power) {
    // toggle motor power
    powerState = !powerState;
    in_power = 0;
  }
  else if (in_speed_up) {// if speed input from button is not equal to zero, then move waypoint forward or back
    if (current_mode == "anchor") {
      move_waypoint("forward");
    }
    else if (current_mode == "nav") {
      // increase set speed
      if (set_speed < max_speed) {
        set_speed = set_speed + .1; // increment speed
      }
      if (set_speed > max_speed) {
        set_speed = max_speed;  // set maximum speed
      }
      Serial.print("Set Speed nav increase: "); Serial.print(set_speed); Serial.println("mph");
    }
    else if (current_mode == "manual") {
      if (set_speed < max_speed) {
        set_speed = set_speed + .1; // increment speed
      }
      if (set_speed > max_speed) {
        set_speed = max_speed;  // set maximum speed
      }
      Serial.print("Set Speed manual increase: "); Serial.print(set_speed); Serial.println("mph");
    }

    in_speed_up = 0;
  }
  else if (in_slow_down) {  // if direction input from button is not equal to zero then move waypoint to the side
    if (current_mode == "anchor") {
      move_waypoint("backward");
    }
    else if (current_mode == "nav") {
      // decrease set speed
      if (set_speed > 0) {
        set_speed = set_speed - .1;

      }
      if (set_speed < 0) {
        set_speed = 0;
      }
      Serial.print("Set Speed nav decrease: "); Serial.print(set_speed); Serial.println("mph");
    }
    else if (current_mode == "manual") {
      // manual mode slow down
      if (set_speed > 0) {
        set_speed = set_speed - .1;

      }
      if (set_speed < 0) {
        set_speed = 0;
      }
      Serial.print("Set Speed manual decrease: "); Serial.print(set_speed); Serial.println("mph");
    }
    in_slow_down = 0;
  }
  else if (in_right) {
    if (current_mode == "anchor") {
      move_waypoint("right");
    }
    else if (current_mode == "nav") {
      // adjust path to the right
      modify_path("right");
    }
    in_right = 0;
  }
  else if (in_left) {
    if (current_mode == "anchor") {
      move_waypoint("left");
    }
    else if (current_mode == "nav") {
      // adjust path to the left
      modify_path("left");
    }
    in_left = 0;
  }

}


//****************************************************************************
//* ***************************************************************************

double gps_convert_deg(float gps_value)
{
  int gps_val_int = gps_value / 100;  // parse out degrees
  double gps_decimal = gps_val_int + (gps_value - gps_val_int * 100) / 60; // convert remaining minutes portion to degrees and add to degrees
  return gps_decimal;
}


//****************************************************************************
//****************************************************************************

// calculate the surface distance from the waypoint in feet
void calc_offset() {
  latitude_offset = 730445.49 * (latitude - wp_lat);
  long_offset = 730445.49 * cos(latitude / 57.295779) * (longitude - wp_long);
  waypoint_distance = sqrt(latitude_offset * latitude_offset + long_offset * long_offset);
  if (long_offset == 0)
  {
    if (latitude_offset >= 0)
    {
      waypoint_angle = 0;  // if there is no longitude offset then if the lat offset is positive, the angle = 0
    }
    else
    {
      waypoint_angle = 180; // if there is no longitude offset then if the lat offset is negative, the angle is 180
    }
  }
  else
  {
    waypoint_angle = atan(latitude_offset / long_offset) * 180 / 3.14159; // calculate angle offset between current location and waypoint
  }
  if (waypoint_angle < 0) {
    waypoint_angle = 360 + waypoint_angle; // make sure that the angle is between 0 and 360
  }



}



//****************************************************************************
//****************************************************************************

// Get compass values and compensate for errors
void compass_output() {
  // get data from digital compasses
  sensors_event_t motor_mag_data, motor_grav_data, boat_mag_data, boat_grav_data;

  boat_dir.getEvent(&boat_mag_data, Adafruit_BNO055::VECTOR_MAGNETOMETER);    // boat magnetometer data
  motor_dir.getEvent(&motor_mag_data, Adafruit_BNO055::VECTOR_MAGNETOMETER);  // motor magnetometer data

  boat_dir.getEvent(&boat_grav_data, Adafruit_BNO055::VECTOR_GRAVITY);        // boat gravity data
  motor_dir.getEvent(&motor_grav_data, Adafruit_BNO055::VECTOR_GRAVITY);      // motor gravity data

  float boat_grav_x = boat_grav_data.acceleration.x;
  float boat_grav_y = boat_grav_data.acceleration.y;
  float boat_grav_z = boat_grav_data.acceleration.z;

  float motor_grav_x = motor_grav_data.acceleration.x;
  float motor_grav_y = motor_grav_data.acceleration.y;
  float motor_grav_z = motor_grav_data.acceleration.z;

  // get boat pitch and yaw for tilt compensation
  float boat_roll = atan2(boat_grav_y, boat_grav_z);
  float boat_pitch = atan2(boat_grav_x, boat_grav_z * cos(boat_roll) + boat_grav_y * sin(boat_roll));

  // get motor pitch and yaw for tilt compensation
  float motor_roll = atan2(motor_grav_y, motor_grav_z);                                                       // phi
  float motor_pitch = atan2(motor_grav_x, motor_grav_z * cos(motor_roll) + motor_grav_y * sin(motor_roll));   // theta

  // calculate boat magnetometer values with tilt compensation
  float boat_mag_z = boat_mag_data.magnetic.z;
  float boat_mag_y = boat_mag_z * sin(boat_roll) - boat_mag_data.magnetic.y * cos(boat_roll);
  float boat_mag_x = boat_mag_data.magnetic.x * cos(boat_pitch) - boat_mag_y * sin(boat_roll) * sin(boat_pitch) - boat_mag_z * cos(boat_roll) * sin(boat_pitch);

  // calculate motor magnetometer values with tilt compensation
  float motor_mag_z = motor_mag_data.magnetic.z;
  float motor_mag_y = motor_mag_z * sin(motor_roll) - motor_mag_data.magnetic.y * cos(motor_roll);
  float motor_mag_x = motor_mag_data.magnetic.x * cos(motor_pitch) - motor_mag_y * sin(motor_roll) * sin(motor_pitch) - motor_mag_z * cos(motor_roll) * sin(motor_pitch);

  if (boat_mag_x == 0) {
    if (boat_mag_y > 0) {
      compass_boat = 0;
    }
    else {
      compass_boat = 180;
    }
  }

  compass_boat = 360 - atan2(boat_mag_y, boat_mag_x) / deg_to_rad;
  if (compass_boat > 360) {
    compass_boat = compass_boat - 360;
  }

  if (motor_mag_x == 0) {
    if (motor_mag_y > 0) {
      compass_motor = 0;
    }
    else {
      compass_motor = 180;
    }
  }

  compass_motor = 360 - atan2(motor_mag_y, motor_mag_x) / deg_to_rad;
  if (compass_motor > 360) {
    compass_motor = compass_motor - 360;
  }

  motor_boat_angle = compass_motor - compass_boat;
  if (motor_boat_angle < 0) {
    motor_boat_angle = motor_boat_angle + 360;
  }

  if (current_mode == "nav") {
    max_turn_angle = compass_boat + max_motor_angle_nav;
    min_turn_angle = compass_boat - max_motor_angle_nav;
  } else if (current_mode == "anchor") {
    max_turn_angle = compass_boat + max_motor_angle_anchor;
    min_turn_angle = compass_boat - max_motor_angle_anchor;
  }

  boat_wp_angle = compass_boat - waypoint_angle;  //calculate the angle between the boat and the waypoint
  if (boat_wp_angle < 0) {
    boat_wp_angle = 360 - boat_wp_angle;  // keep it between 0 and 360 degrees
  }

  if (boat_wp_angle > 180) {
    turn_direction = "left";
  }
  else {
    turn_direction = "right";
  }


  //  Serial.print("Boat orientation angle: "); Serial.print(compass_boat); Serial.print(" degrees,     ");

  //  Serial.print("Motor orientation angle: "); Serial.print(compass_motor); Serial.println(" degrees");

}


//****************************************************************************
//****************************************************************************

// Calculate the desired motor power for spot lock move
void calc_anchor_motor_power() {
  //
  powerPID.SetTunings(Kp_power, Ki_power, Kd_power);
  powerPID.Compute();
  Serial.print("Spotlock motor power: "); Serial.println(set_power);
  set_motor_power(set_power);

}

//****************************************************************************
//****************************************************************************

void calc_boat_motor_angle() {
  // check that new angle falls within the desired range with respect to the boat angle
  boat_motor_angle = compass_boat - compass_motor;
  Serial.print("unaltered boat to motor angle = "); Serial.println(boat_motor_angle);

  // make sure that the angle between the boat and the motor is set between -180 and +180
  if (boat_motor_angle > 180) {
    boat_motor_angle = boat_motor_angle - 360;
    Serial.print("boat motor angle > 180... boat motor angle = "); Serial.println(boat_motor_angle);
  }
  else if (boat_motor_angle < -180) {
    boat_motor_angle = 360 + boat_motor_angle;
    Serial.print("boat motor angle < -180... boat motor angle ="); Serial.println(boat_motor_angle);
  }
}

//****************************************************************************
//****************************************************************************

void turn_angle_dir() {

  if (turn_angle >= 0) {
    turn_dir = "right";
    ang = turn_angle; // convert angle to an integer
  }
  else {
    turn_dir = "left";
    ang = -turn_angle; // convert angle to an integer
  }
}


//****************************************************************************
//****************************************************************************

// calculate the motor angle to turn for spot lock move
void calc_anchor_motor_turn() {
  // update motor angle
  turn_angle = waypoint_angle - compass_motor;
  // make sure that the turn angle is within the desired range of -180 to 180
  if (turn_angle > 180) {
    turn_angle = 360 - turn_angle;
  }
  if (turn_angle < -180) {
    turn_angle = 360 + turn_angle;
  }

  calc_boat_motor_angle();

  // make sure that the motor doesn't turn past 180 degrees from the boat orientation.  go the other way if it would
  if (boat_motor_angle + turn_angle < -180) {
    Serial.println("Boat motor angle + turn angle less than -180 degrees, flip to spin other way");
    Serial.print("Unchanged turn angle + boat/motor angle = "); Serial.println(turn_angle + boat_motor_angle);
    turn_angle = 360 + turn_angle;
  }
  else if (boat_motor_angle + turn_angle > 180) {
    Serial.println("Boat motor angle + turn angle greater than 180 degrees, flip to spin other way");
    Serial.print("Unchanged turn angle + boat/motor angle = "); Serial.println(turn_angle + boat_motor_angle);
    turn_angle = turn_angle - 360;
  }

  turn_angle_dir();

  // send command to turn the motor
  turn_motor(turn_dir, ang);

}


//****************************************************************************
//****************************************************************************

// calculate the motor power while tracking the desired path
void calc_nav_motor_power() {
  // make sure motor is on
  speedPID.SetTunings(Kp_speed, Ki_speed, Kd_speed);
  speedPID.Compute();
  // set the digital potentiometer value
  set_motor_power(set_power);

}


//****************************************************************************
//****************************************************************************

// Turn the motor to track the desired path
void calc_nav_motor_turn() {
  // check angle
  pathPID.SetTunings(Kp_turn, Ki_turn, Kd_turn);
  pathPID.Compute();
  if (set_motor_angle > max_path_turn_angle) {
    set_motor_angle = max_path_turn_angle;
  }
  if (set_motor_angle < -max_path_turn_angle) {
    set_motor_angle = -max_path_turn_angle;
  }

  // check that new angle falls within the desired range with respect to the boat angle
  int boat_motor_angle = compass_boat - compass_motor;

  // make sure that the angle between the boat and the motor is set between -180 and +180
  if (boat_motor_angle > 180) {
    boat_motor_angle = boat_motor_angle - 360;
  }
  else if (boat_motor_angle < -180) {
    boat_motor_angle = 360 + boat_motor_angle;
  }

  // if boat motor angle is outside of the desired range then turn the motor back to inside of this range
  if (boat_motor_angle > max_path_turn_angle) {
    set_motor_angle = max_path_turn_angle - boat_motor_angle; // turn back until at 45 degrees (negative turn)
  }
  else if (boat_motor_angle < -max_path_turn_angle) {
    set_motor_angle = -boat_motor_angle - max_path_turn_angle;  // turn forward until at -45 degrees (positive turn)
  }
  else {  // boat motor angle is in the desired range
    if (boat_motor_angle + set_motor_angle > max_path_turn_angle) { // both angles are positive and the addition of the 2 angles is bigger then 45 degrees
      set_motor_angle = max_path_turn_angle - boat_motor_angle;   // turn so final angle results in boat_motor_angle = 45 next time around
    }
    else if (boat_motor_angle + set_motor_angle < -max_path_turn_angle) {
      set_motor_angle = -boat_motor_angle - max_path_turn_angle;  // turn so final angle results in boat_motor_angle = -45 next time around
    }
  }
  // find the amount of motor turn needed based on the desired angle versus the
  int cmd_motor_turn_angle = boat_motor_angle + set_motor_angle;
  int abs_turn_angle;
  if (cmd_motor_turn_angle >= 0) {
    turn_dir = "right";
    abs_turn_angle = cmd_motor_turn_angle;
    Serial.print("Nav right turn: "); Serial.print(abs_turn_angle); Serial.println(" degrees");
  }
  else {
    turn_dir = "left";
    abs_turn_angle = -cmd_motor_turn_angle;
    Serial.print("Nav left turn: "); Serial.print(abs_turn_angle); Serial.println(" degrees");
  }
  // turn the motor if needed
  if (cmd_motor_turn_angle != 0) {
    turn_motor(turn_dir, abs_turn_angle);
  }

}

//****************************************************************************
//****************************************************************************

// calculate motor power in manual mode
void calc_manual_motor_power() {
  // make sure motor is on
  speedPID.SetTunings(Kp_speed, Ki_speed, Kd_speed);
  speedPID.Compute();
  // set the digital potentiometer value
  set_motor_power(set_power);
}


//****************************************************************************
//****************************************************************************

// this sets the actual motor turn based on angle
void turn_motor(String _direction, int _angle) {
  double motor_turn_timeout = millis();
  // set direction
  // turn for required time to get angle desired
  float turn_time = 1000 * _angle / turn_coef;
  //  Serial.print("manual turn time: "); Serial.print(turn_time); Serial.print(" ms, Manual Turn angle: "); Serial.print(_angle); Serial.println(" deg");
  if (_direction == "left") {
    // turn to the left some angle
    digitalWrite(left_turn_pin, 1);   // turn on left turn relay
    Serial.println("left turn");
    delay(turn_time);
    //        Serial.print(pendantData.hold_left); Serial.print(", "); Serial.print(current_mode); Serial.println(" mode");
    //        while (pendantData.hold_left == 1 && current_mode == "manual" && millis() - motor_turn_timeout < 5000) {  // keep turning the motor if hold left button is still pressed
    //          delay(50);
    //          Serial.println("Holding left turn button");
    //        }
    digitalWrite(left_turn_pin, 0);   // turn off left turn relay
  }
  else if (_direction == "right") {
    // turn to the right some angle by
    digitalWrite(right_turn_pin, 1);    // turn on right turn relay
    Serial.println("right turn");
    delay(turn_time);
    //        Serial.print(pendantData.hold_right); Serial.print(", "); Serial.print(current_mode); Serial.println(" mode");
    //        while (pendantData.hold_right == 1 && current_mode == "manual" && millis() - motor_turn_timeout < 5000) {   // keep turning the motor if the hold right button is pressed
    //          delay(50);
    //          Serial.println("Holding right turn button");
    //        }
    digitalWrite(right_turn_pin, 0);   // turn off right turn relay
  }
}

//****************************************************************************
//****************************************************************************

//
void set_motor_power(int _power) {
  // Percentage of total power received from 0 to 100
  int _resistance = _power / 100 * 63;
  if (_resistance > 63) {
    _resistance = 63;
  }
  if (_resistance < 0) {
    _resistance = 0;
  }
  // send potentiometer info to AD5273 between 0 and 63
  Wire.beginTransmission(44);     // transmit to device #44 (0x2C)
  Wire.write(byte(0x00));         // instruction byte MUST BE UNDER 128 OR IT WILL TAKE A PERMANENT SET
  Wire.write(_resistance);   // send resistance to set, it will clip values above 63
  Wire.endTransmission();         // stop transmitting

}

//****************************************************************************
//****************************************************************************

// function to check the set value on the digital potentiometer
int check_motor_power() {
  Wire.requestFrom(44, 1);
  char C;
  while (Wire.available()) {
    C = Wire.read();
  }
  int Res = C;
  Res = Res / 63 * 100; // convert to percentage to return between 0 and 100
  return Res;
}

//****************************************************************************
//****************************************************************************

// set the waypoint with current GPS location
void set_waypoint() {
  wp_lat = latitude;
  wp_long = longitude;
  Serial.println("SET WAYPOINT*************************************SET WAYPOINT");

}

//****************************************************************************
//****************************************************************************

// move the waypoint based on input from hand pendant
void move_waypoint(String _dir) {
  // adjust waypoint
  float shift_waypoint = 20; // feet to move the waypoint

  double Y_change = shift_waypoint * sin(compass_boat * deg_to_rad);   // latitude change in feet
  double X_change = shift_waypoint * cos(compass_boat * deg_to_rad);  // longitude change in feet

  if (_dir == "forward") {
    wp_lat = wp_lat + Y_change / 730445.49;
    wp_long = wp_long + X_change / (730445.49 * cos(latitude * deg_to_rad) );
  }
  else if (_dir == "backward") {
    wp_lat = wp_lat - Y_change / 730445.49;
    wp_long = wp_long - X_change / (730445.49 * cos(latitude * deg_to_rad) );
  }
  else if (_dir == "right") {
    wp_lat = wp_lat + X_change / 730445.49;
    wp_long = wp_long + Y_change / (730445.49 * cos(latitude * deg_to_rad) );
  }
  else if (_dir == "left") {
    wp_lat = wp_lat - X_change / 730445.49;
    wp_long = wp_long - Y_change / (730445.49 * cos(latitude * deg_to_rad) );
  }
  calc_offset();  // calculate updated waypoint distance and angle
  send_data_to_pendant(); // send updated info to pendant

  // do trig to calculate new waypoint based on what button was pressed and current boat angle
}

//****************************************************************************
//****************************************************************************

// Set the path tracking
void set_path() {
  // call for new GPS waypoint
  wp_lat = latitude;
  wp_long = longitude;
  turn_angle = compass_boat;
  Serial.println("SET PATH**********************************SET PATH");

}

//****************************************************************************
//****************************************************************************

// update the current path based on button press on pendant
void modify_path(String _direction) {
  wp_lat = latitude;
  wp_long = longitude;
  if (_direction == "left") {
    set_motor_angle = set_motor_angle - 5;  // adjust the angle 5 degrees to the left
  }
  if (set_motor_angle < 0) {
    set_motor_angle = 360 + set_motor_angle;
  }
  if (_direction == "right") {
    set_motor_angle = set_motor_angle + 5;  // adjust the angle 5 degrees to the right
  }
  if (set_motor_angle > 360) {
    set_motor_angle = set_motor_angle - 360;
  }

}

//****************************************************************************
//****************************************************************************

// Update all of the PID coefficients based on trim pot input
void update_pid_coef() {
  Kp_power = Kp_power_max - Kp_power_max * analogRead(Kp_power_pin) / 8184;
  if (Kp_power < 0) {
    Kp_power = 0;
  }
  Ki_power = Ki_power_max - Ki_power_max * analogRead(Ki_power_pin) / 8184;
  if (Ki_power < 0) {
    Ki_power = 0;
  }
  Kp_turn = Kp_turn_max - Kp_turn_max * analogRead(Kp_turn_pin) / 8184;
  if (Kp_turn < 0) {
    Kp_turn = 0;
  }
  Ki_turn = Ki_turn_max - Ki_turn_max * analogRead(Ki_turn_pin) / 8184;
  if (Ki_turn < 0) {
    Ki_turn = 0;
  }
  Kp_speed = Kp_speed_max - Kp_speed_max * analogRead(Kp_speed_pin) / 8184;
  if (Kp_speed < 0) {
    Kp_speed = 0;
  }
  Ki_speed = Ki_speed_max - Ki_speed_max * analogRead(Ki_speed_pin) / 8184;
  if (Ki_speed < 0) {
    Ki_speed = 0;
  }
  //  Serial.print("Kp_power: "); Serial.print(Kp_power); Serial.print(", Ki_power: "); Serial.print(Ki_power); Serial.print(", Kp_turn: "); Serial.print(Kp_turn);
  //  Serial.print("||| Ki_turn: "); Serial.print(Ki_turn); Serial.print(", Kp_speed: "); Serial.print(Kp_speed); Serial.print(", Ki_speed: "); Serial.println(Ki_speed);
}

//****************************************************************************
//****************************************************************************

void update_display() {
  display.clearDisplay();
  if (!in_sleep) {
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(1, 1);
    display.print("Lat:  "); display.println(latitude, 8);
    display.print("Long: "); display.println(longitude, 8);
    display.print("Mode: "); display.print(current_mode); display.print(", PWR: "); display.println(gpsDataOut.motor_power);
    display.print("Boat: "); display.print(compass_boat); display.print(" Motor: "); display.println(compass_motor);
    if (current_mode == "anchor") {
      display.print("PIDp "); display.print(Kp_power); display.print(", "); display.println(Ki_power);
    }
    else if (current_mode == "NAV") {
      display.print("PIDt "); display.print(Kp_turn); display.print(", "); display.println(Ki_turn);
      display.print("PIDs "); display.print(Kp_speed); display.print(", "); display.println(Ki_speed);
    }
    else if (current_mode == "manual") {
      display.print("PIDs "); display.print(Kp_speed); display.print(", "); display.println(Ki_speed);
    }

    if (sendError) {
      Serial.print("Error sending data");
    }
  }

  display.display();
}

//****************************************************************************
//****************************************************************************

// get current GPS info
void get_GPS() {
  gps_timer = millis(); // reset the timer
  if (!GPS.fix) {
    Serial.println("GPS fix not found");
  }
  if (GPS.fix) {
    latitude = gps_convert_deg(GPS.latitude);
    longitude = gps_convert_deg(GPS.longitude);
    if (GPS.lat == 'S') {
      latitude = -1 * latitude;
    }
    if (GPS.lon == 'W') {
      longitude = -1 * longitude;
    }
    gps_angle = GPS.angle;
    gpsDataOut.satellites = (int)GPS.satellites;
    gpsDataOut.gps_fix = (int)GPS.fix;
    boat_speed = GPS.speed * 1.15;  // convert from knots to mph
    calc_offset();
    //    Serial.print("Latitude: "); Serial.println(latitude, 8);
    //    Serial.print("Longitude: "); Serial.println(longitude, 8);
    //    Serial.print("Satellites: "); Serial.println(gpsDataOut.satellites);
    //    Serial.print("Fix: "); Serial.println(gpsDataOut.gps_fix);
    //    Serial.print("Boat Speed: "); Serial.print(boat_speed); Serial.println(" MPH");
    //    Serial.print("Waypoint distance: "); Serial.print(waypoint_distance); Serial.println(" feet");
    //    Serial.print("Waypoint angle: "); Serial.print(waypoint_angle); Serial.println(" degrees");
    //    Serial.print("Motor power: "); Serial.print(gpsDataOut.motor_power); Serial.println("%");

  }
}

void send_data_to_pendant() {
  esp_send_timer = millis(); // reset timer for sending data to hand controller
  // Set values to send
  gpsDataOut.wp_dist = waypoint_distance;     // gps offset distance to waypoint
  gpsDataOut.wp_angle = waypoint_angle;       // gps offset angle to waypoint
  gpsDataOut.boat_compass = compass_boat;     // boat compass output
  gpsDataOut.motor_compass = compass_motor;   // motor compass output
  gpsDataOut.boat_speed = boat_speed;         // boat speed from gps
  gpsDataOut.set_speed = set_speed;           // send the set speed to the pendant
  gpsDataOut.motor_power = set_power;         // send motor power to pendant
  gpsDataOut.power_state = powerState;        // send motor power state to pendant

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &gpsDataOut, sizeof(gpsDataOut));

  if (result == ESP_OK) {
    //    Serial.println("Sent with success");
    sendError = false;
  }
  else {
    Serial.println("Error sending the data");
    sendError = true;
  }
}



// END OF CODE
