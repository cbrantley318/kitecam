// //List of known MACs for my ESPs:
 /*

//CAMERAS:
// ESP32cam (broken antenna one):        A0:DD:6C:A2:78:B0 ** the one currently on the kitecam
// the fried esp32cam (doesn't work)     MAC Unknown
//MY personal esp32cam (still has pins)  2C:BC:BB:83:6B:88 **the primary backup
//MY personal wrover cam devboard:       34:98:7a:bd:5c:3c ** this camera is shit but possible backup

//DEV BOARDS
// ESP32 Wroom DevKit:                   CC:DB:A7:16:DC:3C *not necessary

//DISPLAYS 
// ESP32 CYD (micro usb only):           08:A6:F7:23:B0:10 * the one in use
//USB-C compatible: (gave it to Joyce):  MAC UNknown
//UCB-C compatible (the one we kept):    94:54:c5:e7:3c:e0 * the backup


*/

/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.  
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <WiFi.h>
#include <esp_wifi.h>

void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

void setup(){
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.STA.begin();

  Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();
}
 
void loop(){

}