#include <Arduino.h>
#include <ESPNowCam.h>
#include <drivers/CamAIThinker.h>

#include "WiFi.h"       //changing from ESPNow to bare UDP
#include "AsyncUDP.h"

const char *ssid = "KiteCam";
const char *password = "flykitesgB8iINPE_cDOaT6uarecool@";
const int PRINT_CLIENT_TO_SERIAL = 1;
const int UDP_PORT = 1234;
const int CHUNK_SIZE = 1024;  //fpr splitting the jpeg img
const int HEADER_SIZE = 8;
AsyncUDP udp;

CamAIThinker Camera;


IPAddress screenIP = IPAddress(192, 168, 4, 1);
uint16_t screenPort = 1234;
bool screenConnected = false;



void setup() {
  Serial.begin(115200);
  
  // CYD MAC address: 08:A6:F7:23:B0:10
  uint8_t macRecv[6] = {0x08,0xA6,0xF7,0x23,0xB0,0x10};
  createWifiStation();
  handleUDPServer();
  Camera.begin();
  

}

void loop() {
  if (screenConnected) {
    processFrame();
  }
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

void handleUDPServer() {
  if (udp.listen(UDP_PORT)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    udp.onPacket([](AsyncUDPPacket packet) {

      if (packet.length() < 4) return;

      if (PRINT_CLIENT_TO_SERIAL) { //this is just to clean up the serial monitor in case I don't want to print stuff
        Serial.print("UDP Packet Type: "); Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
        Serial.print(", From: ");   Serial.print(packet.remoteIP());  Serial.print(":");        Serial.print(packet.remotePort());
        Serial.print(", To: ");     Serial.print(packet.localIP());   Serial.print(":");        Serial.print(packet.localPort());
        Serial.print(", Length: "); Serial.print(packet.length());    Serial.print(", Data: "); Serial.write(packet.data(), packet.length()); Serial.println();
      }

      screenIP = packet.remoteIP();
      screenPort = packet.remotePort();
      screenConnected = true;

      uint8_t* recvData = packet.data();
      int length = packet.length();

      if (length > 4) {
        switch(recvData[0]) {

          case 0x03:
            packet.println("received number 3");
          break;

          case 0x04:
            packet.println("received number 4");
          break;
          default:
            packet.printf("Camera says try another command\n");

            for (int i = 0; i < packet.length(); i++) {
              packet.printf("%s", recvData);
              break;
            } 
        }
      }

    });
  }
}

void processFrame() {
  if (Camera.get()) {
    uint8_t *out_jpg = NULL;
    size_t out_jpg_len = 0;
    frame2jpg(Camera.fb, 12, &out_jpg, &out_jpg_len);
    // radio.sendData(out_jpg, out_jpg_len);
    // udp.broadcast(out_jpg,out_jpg_len);
    Serial.println("------------------------------------");



    //constructing header for each chunk packet
    //byte 0 is 0x25 (identifies it as the thing)
    //byte 1 is seq num
    //bytes 2,3,4,5 are the chunk size, 2 is MSB and 5 is LSB
    //bytes 6 and 7 are currently 0, can be changed for later use

    int num_iters = out_jpg_len/CHUNK_SIZE + 1;
    int last_chunk_size = out_jpg_len % CHUNK_SIZE;
    uint8_t* outBuf = (uint8_t*)malloc(sizeof(uint8_t) * (CHUNK_SIZE + HEADER_SIZE + 100));

    int outSize = CHUNK_SIZE + HEADER_SIZE;

    outBuf[0] = 0x25;
    outBuf[1] = 0;
    outBuf[2] = (uint8_t) ((CHUNK_SIZE >> 24) & 255); 
    outBuf[3] = (uint8_t) ((CHUNK_SIZE >> 16) & 255);
    outBuf[4] = (uint8_t) ((CHUNK_SIZE >> 8) & 255);
    outBuf[5] = (uint8_t) ((CHUNK_SIZE >> 0) & 255);
    outBuf[6] = 0;
    outBuf[7] = 0;

    for (int i = 0; i < num_iters; i++) {

      outBuf[0] = 0x25;
      outBuf[1] = (uint8_t)i;
      if (i== num_iters -1) {
        outBuf[0] = 0x26;
        // Serial.println("SENDING FINAL CHUNK");
        outSize = last_chunk_size + HEADER_SIZE;
      } else { //need to write the jpeg image plus my packet header
        // Serial.println("SENDING OTHER CHUNK");
      }
      Serial.print("Outsize: ");
      Serial.println(outSize);
      memcpy(&outBuf[8], &out_jpg[i*CHUNK_SIZE], outSize - HEADER_SIZE);
      udp.writeTo(outBuf, outSize, screenIP, screenPort);

      // for (int i = 0; i < outSize; i++) {
      //   Serial.print(outBuf[i],HEX);
      //   Serial.print(" ");
      // }
      // Serial.println();

    }

      // for (int i = 0; i < out_jpg_len; i++) {
      //   Serial.print(out_jpg[i],HEX);
      //   Serial.print(" ");
      // }
      // Serial.println();

    // udp.writeTo(out_jpg, out_jpg_len, screenIP, screenPort);
    // Serial.print(out_jpg[0], HEX);Serial.print(out_jpg[1], HEX);Serial.print(" ");Serial.print(out_jpg[out_jpg_len-2], HEX);Serial.println(out_jpg[out_jpg_len-1], HEX);
    
    free(outBuf);
    free(out_jpg);
    Camera.free();
  } else {
    Serial.println("Cam NO image");

  }
}
