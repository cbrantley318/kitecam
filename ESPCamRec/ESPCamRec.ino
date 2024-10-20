/**************************************************
 * ESPNowCam video Receiver
 * by @hpsaturn Copyright (C) 2024
 * This file is part ESP32S3 camera tests project:
 * https://github.com/hpsaturn/esp32s3-cam
**************************************************/

#include <Arduino.h>
#include <ESPNowCam.h>

#include "WiFi.h"
#include "AsyncUDP.h"


const char *ssid = "KiteCam";
const char *password = "flykitesgB8iINPE_cDOaT6uarecool@";

const int PRINT_SERVER_TO_SERIAL = 1;
AsyncUDP udp;


#include <FS.h>
#include <SPIFFS.h>
// #include <Utils.h>
#define FORMAT_SPIFFS_IF_FAILED true


#include <JPEGDEC.h>

JPEGDEC jpeg;
const char *PHOTO_URI = "/album.jpg";
const int BUF_SIZE = 42000;


fs::File myfile;

#include <SPI.h>
#include <XPT2046_Bitbang.h>
#include <TFT_eSPI.h>


// ----------------------------
// Touch Screen pins
// ----------------------------

// The CYD touch uses some non default
// SPI pins

#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

XPT2046_Bitbang ts(XPT2046_MOSI, XPT2046_MISO, XPT2046_CLK, XPT2046_CS);

//setting calibration constants 
int xMin = 200;
int xMax = 3700;
int yMin = 340;
int yMax = 3800;

int screenHeight = TFT_HEIGHT;
int screenWidth = TFT_WIDTH;

TFT_eSPI tft = TFT_eSPI();
TFT_eSPI_Button key[2];
bool screenReady = false;


//image frame buffer and size
uint8_t *fb; 
uint32_t fb_i = 0;
int32_t dw, dh;
int frameCount = 0;

void setup() {
  Serial.begin(115200);

  createWifiAP();
  handleUDPClient();

  // BE CAREFUL WITH IT, IF JPG LEVEL CHANGES, INCREASE IT
  fb = (uint8_t*)  malloc(BUF_SIZE* sizeof( uint8_t ) ) ;
  fb[0] = 0;
  
  tft.init();
  tft.setRotation(1);
  tft.startWrite();
  Serial.println("TFT initted");

  if (tft.getRotation()%2 == 1) {   //switch the height and width values if rotated display
    screenHeight = screenHeight ^ screenWidth;    //a = axb
    screenWidth = screenHeight ^ screenWidth;     // b = axbxb = a
    screenHeight = screenHeight ^ screenWidth;    // a = axbxa = b
  } 
  tft.fillScreen(TFT_BLACK);

  int x = 320 / 2;  // center of display
  int y = 100;
  int fontSize = 2;
  tft.drawCentreString("Hello___Touch Screen to Start____There", x, y, fontSize);
  y = 130;

  char partiname[64];
  int spiffsize;
  int spiffusage;

  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
      Serial.println("SPIFFS Mount Failed");
      return;
  }

  tft.drawCentreString("General__Touch Screen to Start__Kenobi", x, y, fontSize);
  screenReady = true;

  // copyFileintoRAM(fb, SPIFFS, PHOTO_URI);
  // drawImagefromRAM((const char*) fb, BUF_SIZE);


  delay(1000);
}
int loopcount = 0;
void loop() {
  //adding in code to register button presses, possibly (unless I can do it with interrupts / callbacks)
  delay(10);
}

/*
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void copyFileintoRAM(uint8_t* dest, fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }
    Serial.println("- read from file:");
    if(file.available()){
        file.read(dest,BUF_SIZE);
    }
    Serial.println();
    file.close();
}

*/ //end of comment block 1

/*

  // void writeFile(fs::FS &fs, const char * path, const char * message){
  //     Serial.printf("Writing file: %s\r\n", path);

  //     File file = fs.open(path, FILE_WRITE);
  //     if(!file){
  //         Serial.println("- failed to open file for writing");
  //         return;
  //     }
  //     if(file.print(message)){
  //         Serial.println("- file written");
  //     } else {
  //         Serial.println("- write failed");
  //     }
  //     file.close();
  // }

  // void appendFile(fs::FS &fs, const char * path, const char * message){
  //     Serial.printf("Appending to file: %s\r\n", path);

  //     File file = fs.open(path, FILE_APPEND);
  //     if(!file){
  //         Serial.println("- failed to open file for appending");
  //         return;
  //     }
  //     if(file.print(message)){
  //         Serial.println("- message appended");
  //     } else {
  //         Serial.println("- append failed");
  //     }
  //     file.close();
  // }

  // void renameFile(fs::FS &fs, const char * path1, const char * path2){
  //     Serial.printf("Renaming file %s to %s\r\n", path1, path2);
  //     if (fs.rename(path1, path2)) {
  //         Serial.println("- file renamed");
  //     } else {
  //         Serial.println("- rename failed");
  //     }
  // }

  // void deleteFile(fs::FS &fs, const char * path){
  //     Serial.printf("Deleting file: %s\r\n", path);
  //     if(fs.remove(path)){
  //         Serial.println("- file deleted");
  //     } else {
  //         Serial.println("- delete failed");
  //     }
  // }

  // int drawImagefromFile(const char *imageFileUri) {
  //     unsigned long lTime = millis();
  //     lTime = millis();
  //     jpeg.open((const char *)imageFileUri, myOpen, myClose, myRead, mySeek, JPEGDraw);
  //     jpeg.setPixelType(1);
  //     int imagePosition = 320/2 - (150 / 2);
  //     // decode will return 1 on sucess and 0 on a failure
  //     int decodeStatus = jpeg.decode(imagePosition, 0, JPEG_SCALE_HALF);
  //     // jpeg.decode(45, 0, 0);
  //     jpeg.close();
  //     Serial.print("Time taken to decode and display Image (ms): ");
  //     Serial.println(millis() - lTime);

  //     return decodeStatus;
  // }

*/

int drawImagefromRAM(const char *imageBuffer, int size) {
    unsigned long lTime = millis();
    lTime = millis();
    jpeg.openRAM((uint8_t *)imageBuffer, size, JPEGDraw);
    jpeg.setPixelType(1);
    int imagePosition = 320/2 - (150 / 2);
    // decode will return 1 on sucess and 0 on a failure
    int decodeStatus = jpeg.decode(imagePosition, 0, JPEG_SCALE_HALF);
    // jpeg.decode(45, 0, 0);
    jpeg.close();
    // Serial.print("Time taken to decode and display Image (ms): ");
    // Serial.println(millis() - lTime);

    return decodeStatus;
}

int JPEGDraw(JPEGDRAW *pDraw) {
  // Stop further decoding as image is running off bottom of screen
  if (pDraw->y >= tft.height())
    return 0;
  tft.pushImage(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, pDraw->pPixels);
  return 1;
}

bool isValidJPEG(uint8_t* fb, uint32_t length) {
  //header is FFD8, footer is FFD9                  idx=0,      idx=1,     idx=len-2,      idx=len-1
  //                                              {1111 1111, 1101 1000,    1111 1111,     1101 1001}
  int magicNum = (fb[0] << 24) | (fb[1] << 16) | (fb[length-2] << 8) | (fb[length-1]);
  return (magicNum == -2555943);
}

void createWifiAP() {
    if (!WiFi.softAP(ssid, password)) {
      Serial.println("Whoops, we're screwed!");
      log_e("Soft AP creation failed.");
      while (1);
    }
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);

    delay(5000);

    int nums = WiFi.softAPgetStationNum();
    Serial.print("Number of stations: ");
    Serial.println(nums);
}

void handleUDPClient() {   //for use by the screen (client and Access Point)
  if (udp.connect(IPAddress(192, 168, 4, 2), 1234)) {
    Serial.println("UDP connected");
    udp.onPacket([](AsyncUDPPacket packet) {

      if (!screenReady) {
        return;
      }

      if (PRINT_SERVER_TO_SERIAL) { //this is just to clean up the serial monitor in case I don't want to print stuff
        Serial.print("UDP Packet Type: ");  Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
        Serial.print(", From: ");           Serial.print(packet.remoteIP());  Serial.print(":");        Serial.print(packet.remotePort());
        Serial.print(", To: ");             Serial.print(packet.localIP());   Serial.print(":");        Serial.print(packet.localPort());
        Serial.print(", Length: ");         Serial.print(packet.length());    
        //Serial.print(", Data: ");           Serial.write(packet.data(), packet.length());   
        Serial.println();
      }

      // Serial.print(frameCount); Serial.print("   ");
      // Serial.print(fb[0], HEX);Serial.print(fb[1], HEX);Serial.print(" ");Serial.print(fb[length-2], HEX);Serial.println(fb[length-1], HEX);

      IPAddress targetIP = packet.remoteIP();
      uint16_t targetPort = packet.remotePort();

      uint8_t* recvData = packet.data();
      int recvLength = packet.length();
      int idx = 0;
      uint32_t chunk_size = 0;


      if (recvLength > 4) { //just an arbitrary length, hope it doesn't bite me later
        switch(recvData[0]) {

          case 0x11:  //contains camera data (i.e. numPhotosTaken, current position, current mode photo/video)

          break;

          case 0x12: //contains other information from camera to the screen TBD


          break;

          case 0x25:  //this means we got one chunk of the data, loading it into frame buffer
            // Serial.println("CHUNK RECV'D");
            idx = recvData[1];  //seq num of this packet
            //reconstruct the chunk size from the packet header
            chunk_size = (recvData[2]<<24) | (recvData[3]<<16) | (recvData[4]<<8) | (recvData[5]); 
            // Serial.println(chunk_size);
            // if (idx*chunk_size != fb_i) {
            //   Serial.println("Chunk size mismatch");
            //   Serial.print(idx); Serial.print(" "); Serial.print(chunk_size); Serial.print(" "); Serial.print(fb_i);
            // }
            fb_i = idx * chunk_size;
            memcpy(&fb[fb_i], &recvData[8], chunk_size);  //chunk_size should be same as recvLength - 8 but not leaving anything to chance
          break;

          case 0x26: //last chunk of data was received, reset buffer pointer, display image
            // Serial.println("FINAL CHUNK");
            idx = recvData[1];  //seq num of this packet
            chunk_size = (recvData[2]<<24) | (recvData[3]<<16) | (recvData[4]<<8) | (recvData[5]); 
            fb_i = idx * chunk_size;
            memcpy(&fb[fb_i], &recvData[8], recvLength - 8);
            fb_i += recvLength - 8; //total length of image

            if (isValidJPEG(fb, fb_i)) {
              drawImagefromRAM((const char*) fb, fb_i);
            }
            tft.drawCentreString("got an image", 320/2, 195, 2);
            tft.drawNumber(frameCount, 320/2, 210);
            frameCount++;
            fb_i = 0;

          break;



          case 0xFF:  //this means it's probably a jpeg
            Serial.println("Proccsign JPEGGGG");

            for (int i = 0; i < recvLength; i++) {
              Serial.print(recvData[i],HEX);
              Serial.print(" ");
            }
            Serial.println();

            // Serial.print(recvData[0], HEX);Serial.print(recvData[1], HEX);Serial.print(" ");Serial.print(recvData[recvLength-2], HEX);Serial.println(recvData[recvLength-1], HEX);
            if (isValidJPEG(recvData, recvLength)) {
              Serial.println("IT WAS VALID");
              tft.drawCentreString("got an image", 320/2, 195, 2);
              tft.drawNumber(frameCount, 320/2, 210);
              frameCount++;
              memcpy(fb, recvData, recvLength);
              drawImagefromRAM((const char*) fb, recvLength);
            } 
          break;

          default:  //unknown packet type, don't respond
            Serial.println("Unknownpacket");
          break;

        } //end of switchcase
      } // if (recvLength > 4)

      if (packet.length() < 20) {

        //reply to the client
        // packet.printf("Got %u bytes of data", packet.length());
        uint8_t sendData = 0x03;
        udp.writeTo(&sendData, 0x01, IPAddress(192, 168, 4, 2), 1234);

      } else {
            // for (int i = 0; i < recvLength; i++) {
            //   Serial.print(recvData[i],HEX);
            //   Serial.print(" ");
            // }
            // Serial.println();
      }

    });
    // udp.print("Hello Server!");
  }
}



