#include <Adafruit_SSD1327.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Adafruit_LC709203F.h"

// Define receiver MAC address
uint8_t broadcastAddress[] = {0x7C, 0xDF, 0xA1, 0x00, 0x4A, 0x9A};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_outgoing {
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
  bool sleep;

} struct_outgoing;

typedef struct struct_incoming {
  float wp_dist;
  float wp_angle;
  int boat_compass;
  int motor_compass;
  int gps_fix;
  int satelites;
  float boat_speed;
  float set_speed;
  int motor_power;
  bool power_state;
} struct_incoming;

// create variables to store incoming data
float wpDist = 0;
float wpAngle = 0;
int boatCompass = 0;
int motorCompass = 0;
int gpsFix = 0;
int gpsSat = 0;
float boatSpeed = 0;
float SPEED = 0;
int motorPower = 0;
bool powerState = false;

double screen_sleep_timer = millis();
volatile double interrupt_debounce_timer = millis();
volatile int debounce_time = 50;
String current_mode = "Manual";
int left_button_old = 0;
int right_button_old = 0;


bool sleepState = false;
float battVolt = 4;

volatile bool sendData = false;
volatile bool screenResetTimer = false;
volatile bool navInteruptCalled = false;
volatile bool anchorInteruptCalled = false;
volatile bool manualInteruptCalled = false;
volatile bool powerInteruptCalled = false;
volatile bool rightInteruptCalled = false;
volatile bool leftInteruptCalled = false;
volatile bool speedupInteruptCalled = false;
volatile bool slowdownInteruptCalled = false;

// Create a struct_message called myData to send to base unit
struct_outgoing myData;
// create a struct_message to receive from base unit
struct_incoming gpsData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&gpsData, incomingData, sizeof(gpsData));
  //  Serial.print("Bytes received: ");
  //  Serial.println(len);
  wpDist = gpsData.wp_dist;
  wpAngle = gpsData.wp_angle;
  boatCompass = gpsData.boat_compass;
  motorCompass = gpsData.motor_compass;
  gpsFix = gpsData.gps_fix;
  gpsSat = gpsData.satelites;
  boatSpeed = gpsData.boat_speed;
  SPEED = gpsData.set_speed;
  motorPower = gpsData.motor_power;
  powerState = gpsData.power_state;
//  Serial.print("Motor power = "); Serial.print(motorPower); Serial.println("% (from main control box ESP now comms)");
  
}

// Used for software SPI
#define OLED_CLK 36
#define OLED_MOSI 35

// Used for software or hardware SPI
#define OLED_CS 10
#define OLED_DC 37

// Used for I2C or SPI
#define OLED_RESET -1

// define pins used on button pad and interrupt
#define nav_pin 17
#define manual_pin 6
#define power_pin 18
#define anchor_pin 15
#define left_pin 5
#define speedup_pin 16
#define right_pin 8
#define slowdown_pin 14

// software SPI
Adafruit_SSD1327 display(128, 128, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

Adafruit_LC709203F lc;  // battery management sensor

void IRAM_ATTR set_nav_pin() {
  if (millis() - interrupt_debounce_timer > debounce_time) {
    if (digitalRead(nav_pin)) {
      if (!sleepState) {
        myData.nav = 1;
        sendData = true;
        current_mode = "NAV";
      }
      else {
        screenResetTimer = true;
      }
      navInteruptCalled = true;
    }
  }
  interrupt_debounce_timer = millis();
  screen_sleep_timer = millis();
  screenResetTimer = false;
  myData.sleep = false;
  sleepState = false;
}

void IRAM_ATTR set_anchor_pin() {
  if (millis() - interrupt_debounce_timer > debounce_time) {
    if (digitalRead(anchor_pin)) {
      if (!sleepState) {
        myData.anchor = 1;
        sendData = true;
        current_mode = "Anchor";
      }
      else {
        screenResetTimer = true;
      }
      anchorInteruptCalled = true;
    }
  }
  interrupt_debounce_timer = millis();
  screen_sleep_timer = millis();
  screenResetTimer = false;
  myData.sleep = false;
  sleepState = false;
}

void IRAM_ATTR set_manual_pin() {
  if (millis() - interrupt_debounce_timer > debounce_time) {
    if (digitalRead(manual_pin)) {
      if (!sleepState) {
        myData.manual = 1;
        sendData = true;
        current_mode = "Manual";
      }
      else {
        screenResetTimer = true;
      }
      manualInteruptCalled = true;
    }
  }
  interrupt_debounce_timer = millis();
  screen_sleep_timer = millis();
  screenResetTimer = false;
  myData.sleep = false;
  sleepState = false;
}

void IRAM_ATTR set_power_pin() {
  if (millis() - interrupt_debounce_timer > debounce_time) {
    if (digitalRead(power_pin)) {
      if (!sleepState) {
        myData.power = 1;
        sendData = true;
      }
      else {
        screenResetTimer = true;
      }
      powerInteruptCalled = true;
    }
  }
  interrupt_debounce_timer = millis();
  screen_sleep_timer = millis();
  screenResetTimer = false;
  myData.sleep = false;
  sleepState = false;
}

void IRAM_ATTR set_right_pin() {
  if (millis() - interrupt_debounce_timer > debounce_time) {
    if (digitalRead(right_pin)) { 
      if (!sleepState) {
        myData.right = 1;
        sendData = true;
      }
      else {
        screenResetTimer = true;
      }
      rightInteruptCalled = true;
    }
  }
  interrupt_debounce_timer = millis();
  screen_sleep_timer = millis();
  screenResetTimer = false;
  myData.sleep = false;
  sleepState = false;
}

void IRAM_ATTR set_left_pin() {
  if (millis() - interrupt_debounce_timer > debounce_time) {
    if (digitalRead(left_pin)) { 
      if (!sleepState) {
        myData.left = 1;
        sendData = true;
      }
      else {
        screenResetTimer = true;
      }
      leftInteruptCalled = true;
    }
  }
  interrupt_debounce_timer = millis();
  screen_sleep_timer = millis();
  screenResetTimer = false;
  myData.sleep = false;
  sleepState = false;
}

void IRAM_ATTR set_speed_up_pin() {
  if (millis() - interrupt_debounce_timer > debounce_time) {
    if (digitalRead(speedup_pin)) {
      if (!sleepState) {
        myData.speed_up = 1;
        sendData = true;
      }
      else {
        screenResetTimer = true;
      }
      speedupInteruptCalled = true;
    }
  }
  interrupt_debounce_timer = millis();
  screen_sleep_timer = millis();
  screenResetTimer = false;
  myData.sleep = false;
  sleepState = false;
}

void IRAM_ATTR set_slow_down_pin() {
  if (millis() - interrupt_debounce_timer > debounce_time) {
    if (digitalRead(slowdown_pin)) {
      if (!sleepState) {
        myData.slow_down = 1;
        sendData = true;
      }
      else {
        screenResetTimer = true;
      }
      slowdownInteruptCalled = true;
    }
  }
  interrupt_debounce_timer = millis();
  screen_sleep_timer = millis();
  screenResetTimer = false;
  myData.sleep = false;
  sleepState = false;
}


void setup()   {

  pinMode(nav_pin, INPUT); // NAVIGATION PIN
  pinMode(manual_pin, INPUT);  // manual PIN
  pinMode(power_pin, INPUT); // POWER PIN
  pinMode(anchor_pin, INPUT); // ANCHOR PIN
  pinMode(left_pin, INPUT);  // LEFT TURN PIN
  pinMode(speedup_pin, INPUT); // SPEED UP PIN
  pinMode(right_pin, INPUT);  // RIGHT TURN PIN
  pinMode(slowdown_pin, INPUT); // SLOW DOWN PIN
  //pinMode(interrupt_pin, INPUT); // INTERRUPT PIN
  #if defined (ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
    // turn
    pinMode(PIN_I2C_POWER, INPUT);
    delay(1);
    bool polarity = digitalRead(PIN_I2C_POWER);
    pinMode(PIN_I2C_POWER, OUTPUT);
    digitalWrite(PIN_I2C_POWER, !polarity);
  #endif

  if(!lc.begin()) {
    Serial.println(F("Couldn't find Adafruit LC709203F?\nMake sure the battery is plugged in!"));
    while (1) delay(10);
  }
  Serial.println(F("Found LC709203F"));
  Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);

  lc.setThermistorB(3950);
  Serial.print("Thermistor B = "); Serial.println(lc.getThermistorB());

  lc.setPackSize(LC709203F_APA_3000MAH); // Actual battery is 3350 mAh Li-Ion 18650
  lc.setAlarmVoltage(3.8);
  

  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(nav_pin), set_nav_pin, RISING);
  attachInterrupt(digitalPinToInterrupt(manual_pin), set_manual_pin, RISING);
  attachInterrupt(digitalPinToInterrupt(power_pin), set_power_pin, RISING);
  attachInterrupt(digitalPinToInterrupt(anchor_pin), set_anchor_pin, RISING);
  attachInterrupt(digitalPinToInterrupt(left_pin), set_left_pin, RISING);
  attachInterrupt(digitalPinToInterrupt(right_pin), set_right_pin, RISING);
  attachInterrupt(digitalPinToInterrupt(speedup_pin), set_speed_up_pin, RISING);
  attachInterrupt(digitalPinToInterrupt(slowdown_pin), set_slow_down_pin, RISING);


  //while (! Serial) delay(100);
  Serial.println("SSD1327 OLED test");
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  if ( ! display.begin(0x3D) ) {
    Serial.println("Unable to initialize OLED");
    while (1) yield();
  }

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  display.clearDisplay();
  display.display();
}


void loop() {
  Serial.println("Main loop");
  Serial.print("home screen sleep state: "); Serial.println(myData.sleep);
//  Serial.print("Batt Voltage: "); Serial.println(lc.cellVoltage(), 2);
//  Serial.print("Batt Percent: "); Serial.println(lc.cellPercent(), 1);
//  Serial.print("Batt Temp: "); Serial.println(lc.getCellTemperature(), 1);

  if (navInteruptCalled) {
    Serial.println("Nav interrupt called");
    navInteruptCalled = false;
  }
  if (powerInteruptCalled) {
    Serial.println("Power interrupt called");
    powerInteruptCalled = false;
  }
  if (manualInteruptCalled) {
    Serial.println("manual interrupt called");
    manualInteruptCalled = false;
  }
  if (anchorInteruptCalled) {
    Serial.println("Anchor interrupt called");
    anchorInteruptCalled = false;
  }
  if (rightInteruptCalled) {
    Serial.println("Right turn interrupt called");
    rightInteruptCalled = false;
  }
  if (leftInteruptCalled) {
    Serial.println("Left turn interrupt called");
    leftInteruptCalled = false;
  }
  if (speedupInteruptCalled) {
    Serial.println("Speed up interrupt called");
    speedupInteruptCalled = false;
  }
  if (slowdownInteruptCalled) {
    Serial.println("Slow Down interrupt called");
    slowdownInteruptCalled = false;
  }

  if (myData.hold_left) {
    Serial.println("holding left pin");
  }
  if (myData.hold_right) {
    Serial.println("holding right pin");
  }
  // check for right/left turn in manual mode
  if(current_mode == "Manual") {
    if(digitalRead(left_pin)) {
      // turn on left turn
      myData.hold_left = 1;
//      Serial.println("Manual Left turn");
      sendData = true;
    }
    else {
      // turn off left turn
      myData.hold_left = 0;
      sendData = true;
    }
    if(digitalRead(right_pin)) {
      // turn on right turn
      myData.hold_right = 1;
//      Serial.println("Manual Right turn");
      sendData = true;
    }
    else {
      // turn off right turn
      myData.hold_right = 0;
      sendData = true;
    }
  }

  // Set values to send - do this in the button interrupt function

  if (sendData) {
    send_data();    // send data to other esp32 device CHANGE THIS
  }
  
  display_refresh();
  delay(50);
}

void send_data() {
  //   send data to other ESP32
  //   send: button press, current mode,
  //   clear out struct after sending the data
  //   Set values to send
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  myData.manual = 0; // may take this one out as I don't want it to reset each time
  myData.nav = 0; // this is the button press state
  myData.anchor = 0;
  myData.power = 0;
  myData.speed_up = 0;
  myData.slow_down = 0;
  myData.left = 0;
  myData.right = 0;
  sendData = false;

}



void display_refresh() {
  // update pendant display
  int compass_center_X = 86;
  int compass_center_Y = 86;
  int compass_radius = 30;
  int sz1_ltr_offset_x = 3;   // offset to center letters on compass
  int sz1_ltr_offset_y = 4;   // offset to center letters on compass
  display.clearDisplay();
  if (screenResetTimer) {
    screen_sleep_timer = millis();
    screenResetTimer = false;
    sleepState = false;
    Serial.println("Screen sleep timer reset");

  }
  if (millis() - screen_sleep_timer > 600000) {
    //  do nothing, just don't write the screen
    //    Serial.println("Screen is sleeping, nite nite");
    myData.sleep = true;
    sleepState = true;
  }
  else {
    display.setTextSize(1);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(1, 2);
    display.print(current_mode); display.println(" Mode"); 
    //display.setCursor(2, 12);
    int dist = wpDist;
    if (current_mode == "Anchor") {
      display.print("DIST: "); display.print(dist); display.println(" FT  ");
      //display.setCursor(2, 22);
    }
    else if (current_mode == "NAV") {
      // display distance from path
      // not currently sending this from base station
    }
    display.print("SET: "); display.print(SPEED); display.println(" mph");
    //display.setCursor(2, 32);
    display.print("SPD: "); display.print(boatSpeed); display.println(" mph");
    //display.setCursor(2, 42);
    // display number of satelites
    display.print("SAT: "); display.println(gpsSat);
    // display fix quality
    //display.setCursor(2, 52);
    display.print("FIX: "); display.println(gpsFix);
    if (powerState) {
      display.println("PWR ON");
    }
    else {
      display.println("PWR OFF");
    }
    display.print(lc.cellVoltage(), 2); display.println(" V");
    // display motor power
    display.setCursor(2, 116);
    display.print("PWR: "); display.print(motorPower); display.println("%");


    display.drawCircle(compass_center_X, compass_center_Y, compass_radius, SSD1327_WHITE);
    display.drawCircle(compass_center_X, compass_center_Y, compass_radius - 1, SSD1327_WHITE);
    display.drawCircle(compass_center_X, compass_center_Y, compass_radius + 1, SSD1327_WHITE);

    int sin_boat_angle = (compass_radius + 7) * sin(boatCompass*3.14/180);
    int cos_boat_angle = (compass_radius + 7) * cos(boatCompass*3.14/180);

    int north_X = compass_center_X - sin_boat_angle - sz1_ltr_offset_x;
    int north_Y = compass_center_Y - cos_boat_angle - sz1_ltr_offset_y;

    int south_X = compass_center_X + sin_boat_angle - sz1_ltr_offset_x;
    int south_Y = compass_center_Y + cos_boat_angle - sz1_ltr_offset_y;

    int east_X = compass_center_X + cos_boat_angle - sz1_ltr_offset_x;
    int east_Y = compass_center_Y - sin_boat_angle - sz1_ltr_offset_y;

    int west_X = compass_center_X - cos_boat_angle - sz1_ltr_offset_x;
    int west_Y = compass_center_Y + sin_boat_angle - sz1_ltr_offset_y;

    // put north/south/east/west on the compass
    display.setCursor(north_X, north_Y); display.println("N");
    display.setCursor(south_X, south_Y); display.println("S");
    display.setCursor(east_X, east_Y); display.println("E");
    display.setCursor(west_X, west_Y); display.println("W");
    int wp_X;
    int wp_Y;
    // draw circle on the compass towards waypoint/desired path
    if (current_mode == "Anchor") {
      if (dist == 0) {
        wp_X = compass_center_X;
        wp_Y = compass_center_Y;
      }
      else {
        wp_X = compass_center_X - (compass_radius - 5) * sin((boatCompass - wpAngle)*3.14/180);
        wp_Y = compass_center_Y - (compass_radius - 5) * cos((boatCompass - wpAngle)*3.14/180);
      }
    }
    else if (current_mode == "NAV") {
      wp_X = compass_center_X + (compass_radius - 5) * sin((boatCompass - wpAngle)*3.14/180);
      wp_Y = compass_center_Y + (compass_radius - 5) * cos((boatCompass - wpAngle)*3.14/180);
    }
    if (current_mode == "Anchor" || current_mode == "NAV") { // if either nav or anchor mode, place indicator of direction wanted
      for (int16_t i = 1; i <= 4; i++) {
        display.drawCircle(wp_X, wp_Y, i, SSD1327_WHITE);
      }
    }

    // draw line on the compass for motor direction
    int mc_X = compass_center_X - compass_radius * sin((boatCompass - motorCompass)*3.14/180);
    int mc_Y = compass_center_Y - compass_radius * cos((boatCompass - motorCompass)*3.14/180);
    display.drawLine(compass_center_X, compass_center_Y, mc_X, mc_Y, SSD1327_WHITE);

    // draw battery charge state
    display.drawRoundRect(110, 5, 15, 35, 3, SSD1327_WHITE);  // X, Y, WIDTH, HEIGHT, RADIUS, COLOR
    display.drawRect(114, 3, 7, 2, SSD1327_WHITE);  // TOP OF BATTERY
    if (lc.cellVoltage() > 3.4){
      display.fillRect(112, 34, 11, 3, SSD1327_WHITE);
    }
    if (lc.cellVoltage() > 3.55){
      display.fillRect(112, 29, 11, 3, SSD1327_WHITE);
    }
    if (lc.cellVoltage() > 3.7){
      display.fillRect(112, 24, 11, 3, SSD1327_WHITE);
    }
    if (lc.cellVoltage() > 3.85){
      display.fillRect(112, 19, 11, 3, SSD1327_WHITE);
    }
    if (lc.cellVoltage() > 4.0){
      display.fillRect(112, 14, 11, 3, SSD1327_WHITE);
    }
    if (lc.cellVoltage() > 4.1){
      display.fillRect(112, 9, 11, 3, SSD1327_WHITE);
    }
  }
  
  display.display();
}
