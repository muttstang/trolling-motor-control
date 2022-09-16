#include <Adafruit_SSD1327.h>
//#include <esp_now.h>
//#include <WiFi.h>

// Define receiver MAC address
uint8_t broadcastAddress[] = {0x7C, 0xDF, 0xA1, 0x00, 0x4A, 0x9A};

// Structure example to send data
// Must match the receiver structure
//typedef struct struct_message {
//  int mode;
//  int speed;
//  int direction;
//} struct_message;

// Create a struct_message called myData
//struct_message myData;
//
//esp_now_peer_info_t peerInfo;

// callback when data is sent
//void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//}

// Used for software SPI
#define OLED_CLK 36
#define OLED_MOSI 35

#define nav_pin 17
#define exit_pin 6
#define power_pin 18
#define anchor_pin 15
#define left_pin 5
#define speedup_pin 16
#define right_pin 8
#define slowdown_pin 14
//#define interrupt_pin 11

// Used for software or hardware SPI
#define OLED_CS 10
#define OLED_DC 37

// Used for I2C or SPI
#define OLED_RESET -1

// software SPI for 128x128 OLED 
Adafruit_SSD1327 display(128, 128, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

void setup()   {                
  Serial.begin(9600);

//  pinMode(interrupt_pin, INPUT);
//  attachInterrupt(digitalPinToInterrupt(interrupt_pin), set_pin, RISING);
  //while (! Serial) delay(100);
  Serial.println("SSD1327 OLED test");
  // Set device as a Wi-Fi Station
//  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
//  if (esp_now_init() != ESP_OK) {
//    Serial.println("Error initializing ESP-NOW");
//    return;
//  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
//  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
//  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
//  peerInfo.channel = 0;  
//  peerInfo.encrypt = false;
  
  // Add peer        
//  if (esp_now_add_peer(&peerInfo) != ESP_OK){
//    Serial.println("Failed to add peer");
//    return;
//  }
  
  // define IO pins
  pinMode(nav_pin, INPUT);
  pinMode(exit_pin, INPUT);
  pinMode(power_pin, INPUT);
  pinMode(anchor_pin, INPUT);
  pinMode(left_pin, INPUT);
  pinMode(speedup_pin, INPUT);
  pinMode(right_pin, INPUT);
  pinMode(slowdown_pin, INPUT);
  //pinMode(interrupt_pin, INPUT);
  
  display.clearDisplay();

  // draw a bitmap icon and 'animate' movement
  //testdrawbitmap(logo16_glcd_bmp, LOGO16_GLCD_HEIGHT, LOGO16_GLCD_WIDTH);
  Serial.println("Setup Loop Complete");
}


void loop() {

  // Set values to send - do this in the button interrupt function
//  myData.mode = random(1,20);
//  myData.speed = random(21,40);
//  myData.direction = random(41,60);
//  
  // Send message via ESP-NOW
//  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
//  if (result == ESP_OK) {
//    Serial.println("Sent with success");
//  }
//  else {
//    Serial.println("Error sending the data");
//  }


  display_refresh();
  delay(100);
  //testdrawbitmap(logo16_glcd_bmp, LOGO16_GLCD_HEIGHT, LOGO16_GLCD_WIDTH);
}





void display_refresh(){  
  // update the display here
  display.clearDisplay(); // clear out the previous display
  // put in all of the new display info here
  // boat angle
  // motor angle
  // update compass
  int cir_x = 64;
  int cir_y = 64;
  // update current state
  // update gps coordinates
  // update distance from waypoint (if in spot lock mode)
  // update distance from intended path (if in nav mode)
  // update motor power gauge
  // update battery gauge
  if(digitalRead(nav_pin)){
    display.setTextSize(2);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(8,8);
    display.println("Navigation");
    cir_x = 38;
    cir_y = 90;
  }
  if(digitalRead(exit_pin)){
    display.setTextSize(2);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(8,8);
    display.println("Exit");
    cir_x = 38;
    cir_y = 90;
  }
  if(digitalRead(anchor_pin)){
    display.setTextSize(2);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(8,8);
    display.println("Spot Lock");
    cir_x = 90;
    cir_y = 90;
  }
  if(digitalRead(power_pin)){
    display.setTextSize(2);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(8,8);
    display.println("Power");
    cir_x = 90;
    cir_y = 90;
  }
  if(digitalRead(left_pin)){
    display.setTextSize(2);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(8,8);
    display.println("Turn Left");
    cir_x = 90;
    cir_y = 38;
  }
  if(digitalRead(speedup_pin)){
    display.setTextSize(2);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(8,8);
    display.println("Speed Up");
    cir_x = 90;
    cir_y = 38;
  }
  if(digitalRead(right_pin)){
    display.setTextSize(2);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(8,8);
    display.println("Turn Right");
    cir_x = 38;
    cir_y = 38;
  }
  if(digitalRead(slowdown_pin)){
    display.setTextSize(2);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(8,8);
    display.println("Slow Down");
    cir_x = 38;
    cir_y = 38;
  }
  display.drawCircle(cir_x, cir_y, 30, SSD1327_WHITE);
  display.drawCircle(cir_x, cir_y, 29, SSD1327_WHITE);
  display.drawCircle(cir_x, cir_y, 28, SSD1327_WHITE);

  display.display();  // update the display with the new info
}

//void send_data(){
  // send data to other ESP32
  // send: button press, current mode, 
  // clear out struct after sending the data
  // Set values to send
  //myData.mode = 0; // may take this one out as I don't want it to reset each time
//  myData.speed = 0; // this is the button press state
//  myData.direction = 0;
//}

/*
#define nav_pin 17
#define exit_pin 6
#define power_pin 18
#define anchor_pin 15
#define left_pin 5
#define speedup_pin 16
#define right_pin 8
#define slowdown_pin 14
#define interrupt_pin 11
*/
/*
void set_pin(){
  // read which pin is pressed and set the myData structure value correctly
  if(digitalRead(nav_pin)){
//    myData.mode = 1;
  }
  else if(digitalRead(exit_pin)){
//    myData.mode = 2;
  }
  else if(digitalRead(anchor_pin)){ // make this a mode call out
//    myData.mode = 3;
  }
  else if(digitalRead(power_pin)){ // change this to myData.a
//    myData.mode = 4;
  }
  else if(digitalRead(left_pin)){ // direction call out
//    myData.direction = 1;
  }
  else if(digitalRead(right_pin)){ // direction call out
//    myData.direction = 2;
  }
  else if(digitalRead(speedup_pin)){ // speed call out
//    myData.speed = 1;
  }
  else if(digitalRead(slowdown_pin)){ // speed call out
//    myData.speed = 2;
  }
  // change boolean to so that data gets sent
}

*/
