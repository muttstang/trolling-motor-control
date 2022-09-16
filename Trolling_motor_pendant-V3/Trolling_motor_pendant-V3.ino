#include <Adafruit_SSD1327.h>
#include <esp_now.h>
#include <WiFi.h>

// Define receiver MAC address
uint8_t broadcastAddress[] = {0x7C, 0xDF, 0xA1, 0x00, 0x4A, 0x9A};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_outgoing {
  int mode;
  int speed;
  int direction;
} struct_outgoing;

typedef struct struct_incoming {
  int wp_dist;
  int wp_angle;
  int boat_compass;
  int motor_compass;
  int gps_fix;
  int satelites;
  float boat_speed;
} struct_incoming;

// create variables to store incoming data
int wpDist = 0;
int wpAngle = 0;
int boatCompass = 0;
int motorCompass = 0;
int gpsFix = 0;
int gpsSat = 0;
float boatSpeed = 0;

bool sendData = true;
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
  Serial.print("Bytes received: ");
  Serial.println(len);
  wpDist = gpsData.wp_dist;
  wpAngle = gpsData.wp_angle;
  boatCompass = gpsData.boat_compass;
  motorCompass = gpsData.motor_compass;
  gpsFix = gpsData.gps_fix;
  gpsSat = gpsData.satelites;
  boatSpeed = gpsData.boat_speed;
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
#define exit_pin 6
#define power_pin 18
#define anchor_pin 15
#define left_pin 5
#define speedup_pin 16
#define right_pin 8
#define slowdown_pin 14
#define interrupt_pin 13

// software SPI
Adafruit_SSD1327 display(128, 128, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

void setup()   {

  pinMode(nav_pin, INPUT); // NAVIGATION PIN
  pinMode(exit_pin, INPUT);  // EXIT PIN
  pinMode(power_pin, INPUT); // POWER PIN
  pinMode(anchor_pin, INPUT); // ANCHOR PIN
  pinMode(left_pin, INPUT);  // LEFT TURN PIN
  pinMode(speedup_pin, INPUT); // SPEED UP PIN
  pinMode(right_pin, INPUT);  // RIGHT TURN PIN
  pinMode(slowdown_pin, INPUT); // SLOW DOWN PIN
  pinMode(interrupt_pin, INPUT); // INTERRUPT PIN

  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(interrupt_pin), set_pin, RISING);
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
  // Set values to send - do this in the button interrupt function
  myData.mode = random(1, 20);
  myData.speed = random(21, 40);
  myData.direction = random(41, 60);
  if (sendData) {
    send_data();    // send data to other esp32 device CHANGE THIS
  }
  display_refresh();
  delay(100);
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

  // myData.mode = 0; // may take this one out as I don't want it to reset each time
  myData.speed = 0; // this is the button press state
  myData.direction = 0;
}

void display_refresh() {
  // update pendant display
  display.clearDisplay();
  int cir_x = 64;
  int cir_y = 64;
  if (digitalRead(nav_pin)) {
    display.setTextSize(1);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(5, 5);
    display.println("Navigation");
    Serial.println("Navigation");
    cir_x = 38;
    cir_y = 90;
  }
  if (digitalRead(exit_pin)) {
    display.setTextSize(1);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(5, 5);
    display.println("Exit");
    Serial.println("Exit");
    cir_x = 38;
    cir_y = 90;
  }
  if (digitalRead(power_pin)) {
    display.setTextSize(1);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(5, 5);
    display.println("Power");
    Serial.println("Power");
    cir_x = 90;
    cir_y = 90;
  }
  if (digitalRead(anchor_pin)) {
    display.setTextSize(1);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(5, 5);
    display.println("Anchor");
    Serial.println("Anchor");
    cir_x = 90;
    cir_y = 90;
  }
  if (digitalRead(left_pin)) {
    display.setTextSize(1);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(5, 5);
    display.println("Left");
    Serial.println("Left");
    cir_x = 90;
    cir_y = 38;
  }
  if (digitalRead(speedup_pin)) {
    display.setTextSize(1);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(5, 5);
    display.println("Speed up");
    Serial.println("Speed up");
    cir_x = 90;
    cir_y = 38;
  }
  if (digitalRead(right_pin)) {
    display.setTextSize(1);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(5, 5);
    display.println("Right");
    Serial.println("Right");
    cir_x = 38;
    cir_y = 38;
  }
  if (digitalRead(slowdown_pin)) {
    display.setTextSize(1);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(5, 5);
    display.println("Slow Down");
    Serial.println("Slow Down");
    cir_x = 38;
    cir_y = 38;
  }
  display.setTextSize(1);
  display.setTextColor(SSD1327_WHITE);
  display.setCursor(5, 15);
  display.print("WP Distance: "); display.println(wpDist);
  // display number of satelites
  display.setCursor(5, 25);
  display.print("Satelites: "); display.println(gpsSat);
  // display fix quality
  display.setCursor(5, 35);
  display.print("GPS Fix: "); display.println(gpsFix);
  // display compass

  display.drawCircle(cir_x, cir_y, 30, SSD1327_WHITE);
  display.drawCircle(cir_x, cir_y, 29, SSD1327_WHITE);
  display.drawCircle(cir_x, cir_y, 28, SSD1327_WHITE);

  display.display();
}


void set_pin() {
  // read which pin is pressed and set the myData structure value correctly
  if (digitalRead(nav_pin)) {
    //    myData.mode = 1;
  }
  else if (digitalRead(exit_pin)) {
    //    myData.mode = 2;
  }
  else if (digitalRead(anchor_pin)) { // make this a mode call out
    //    myData.mode = 3;
  }
  else if (digitalRead(power_pin)) { // change this to myData.a
    //    myData.mode = 4;
  }
  else if (digitalRead(left_pin)) { // direction call out
    //    myData.direction = 1;
  }
  else if (digitalRead(right_pin)) { // direction call out
    //    myData.direction = 2;
  }
  else if (digitalRead(speedup_pin)) { // speed call out
    //    myData.speed = 1;
  }
  else if (digitalRead(slowdown_pin)) { // speed call out
    //    myData.speed = 2;
  }
  // change boolean to so that data gets sent
}
