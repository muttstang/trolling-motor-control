
#include "AD520X.h"

uint32_t start, stop;
// select, reset, shutdown, data, clock
AD8400 pot(15, 255, 255, 16, 42);  // SW SPI

// define inputs
const byte right_turn_input = A1;
const byte left_turn_input = 5;
const byte on_off_input = A3;
const byte speed_input = A2;

// define outputs
const byte right_turn_output = 2;
const byte left_turn_output = 4;
const byte on_off_output = 6;
const byte speed_output = 3;

// define variables

int speed_value = 0;
int satelite_count = 0;

bool motor_state = false;

float motor_offset_angle = 0;
float magnetic_declination = 0;
float boat_heading = 0;
float desired_heading = 0;

double actual_lattitude = 0;
double actual_longitude = 0;
double desired_lattitude = 0;
double desired_longitude = 0;





void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Begin program");
  pot.setGPIOpins(42, 0, 16, 15); //pot.setGPIOpins(uint8_t clk, uint8_t miso, uint8_t mosi, uint8_t select);
  pot.begin(0); // set initial potentiometer value to zero (this gives you the wiper resistance 50-100 ohms)

  // define pin inputs
  pinMode(right_turn_input, INPUT);
  pinMode(left_turn_input, INPUT);
  pinMode(on_off_input, INPUT);
  pinMode(speed_input, INPUT);

  // define pin outputs
  pinMode(right_turn_output, OUTPUT);
  pinMode(left_turn_output, OUTPUT);
  pinMode(on_off_output, OUTPUT);
  pinMode(speed_output, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10);
  on_off();
  
  if (motor_state == true) {
    speed_calculation();
    steering_calculation();
  }
  //Serial.println(" ");
}

void speed_calculation() {
  // figure out what the speed value is
  
  speed_value = analogRead(speed_input) / 4;
  if (speed_value > 256) { speed_value = 256; }
  if (speed_value < 20) { speed_value = 0; }
  pot.setValue(0,speed_value);  // Set digital potentiometer value for the motor to read
  //analogWrite(speed_output, speed_value);
  Serial.print("Output speed value = "); Serial.print(speed_value); Serial.print(", ");
}

void steering_calculation() {
  // figure out what the steering should change
  if (digitalRead(right_turn_input) == 1) {
    digitalWrite(left_turn_output, 0);
    digitalWrite(right_turn_output, 1);
    Serial.print("Turn Right,  ");
  }
  delay(5);
  
  if (digitalRead(right_turn_input) == 0) {
    digitalWrite(right_turn_output, 0);
    Serial.print("deactivate right turn,  ");
  }
  delay(5);
  
  if (digitalRead(left_turn_input) == 1) {
    digitalWrite(right_turn_output, 0);
    digitalWrite(left_turn_output, 1);
    Serial.print("Turn Left,  ");
  }
  delay(5);
  
  if (digitalRead(left_turn_input) == 0) {
    digitalWrite(left_turn_output, 0);
    Serial.print("deactivate left turn,  ");
  }
  delay(5);
}

void on_off() {
  motor_state = digitalRead(on_off_input);
  digitalWrite(on_off_output, motor_state);
  
  if (motor_state == true) {
    Serial.println(", Motor on");
  }
  else {
    Serial.println(", Motor off");
  }
}
