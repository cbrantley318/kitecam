#include <Arduino.h>
#include <ESPNowCam.h>
#include <drivers/CamAIThinker.h>

#include "WiFi.h"       //changing from ESPNow to bare UDP
#include "AsyncUDP.h"
const int PRINT_CLIENT_TO_SERIAL = 0;

CamAIThinker Camera;
ESPNowCam radio;

void processFrame() {
  if (Camera.get()) {

    uint8_t *out_jpg = NULL;
    size_t out_jpg_len = 0;
    frame2jpg(Camera.fb, 12, &out_jpg, &out_jpg_len);
    radio.sendData(out_jpg, out_jpg_len);
    free(out_jpg);
    Camera.free();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();


  delay(1000); // only for debugging.
  
  // CYD MAC address: 08:A6:F7:23:B0:10
  uint8_t macRecv[6] = {0x08,0xA6,0xF7,0x23,0xB0,0x10};
  // radio.setTarget(macRecv);
  // radio.init();
  
  if (!Camera.begin()) {
    Serial.println("Camera Init Fail");
  }
  delay(500);
}

void loop() {
  processFrame();
}

void createWifiStation() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while (1) {
      delay(1000);
    }
  }
}

handleUDPServer() {
  if (udp.listen(1234)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    udp.onPacket([](AsyncUDPPacket packet) {

      if (PRINT_CLIENT_TO_SERIAL) { //this is just to clean up the serial monitor in case I don't want to print stuff
        Serial.print("UDP Packet Type: "); Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
        Serial.print(", From: ");   Serial.print(packet.remoteIP());  Serial.print(":");        Serial.print(packet.remotePort());
        Serial.print(", To: ");     Serial.print(packet.localIP());   Serial.print(":");        Serial.print(packet.localPort());
        Serial.print(", Length: "); Serial.print(packet.length());    Serial.print(", Data: "); Serial.write(packet.data(), packet.length()); Serial.println();
      }

      //reply to the client
      packet.printf("Got %u bytes of data", packet.length());
    });
  }
}
