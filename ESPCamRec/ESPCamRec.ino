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

const int PRINT_SERVER_TO_SERIAL = 0;
const int UDP_PORT = 1234;
AsyncUDP udp;

IPAddress targetIP;
uint16_t targetPort;


// #include <FS.h>
// #include <SPIFFS.h>
// #include <Utils.h>
// #define FORMAT_SPIFFS_IF_FAILED true


#include <JPEGDEC.h>

JPEGDEC jpeg;
const char *PHOTO_URI = "/album.jpg";
const int BUF_SIZE = 16384;


// fs::File myfile;

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
bool fbReady = false;
uint8_t *fb; 
uint8_t *writefb; 
uint32_t fb_i = 0;
uint32_t writefb_i = 0;
int32_t dw, dh;
int frameCount = 0;


//packet handling
uint8_t* recvData;
int recvLength;
int idx = 0;
uint32_t chunk_size = 0;
uint8_t ack = 0x01;



  //the list of buttons available (or joystick inputs) are as follows (not in order): 
  /*
    btn0: connect to gopro
    btn1: connect to iphone
    btn2: connect to android
    btn3: pan left
    btn4: pan right
    btn5: tilt up
    btn6: tilt down
    btn7: take photo
    btn8: turn off / sleep mode
    btn9: request image quality down
    btn10:

  */    
//control signals to transmit
//                     |    connect  |     direction     |  camera | espcontrol
uint8_t transCmds[] = {0x11,0x12,0x13,0x21,0x22,0x23,0x24,0x31,0x32,0x41     };
uint8_t tansCmdAmount = 11;


void setup() {
  Serial.begin(115200);

  createWifiAP();     //the screen is the wifi AP and the server
  handleUDPServer();  //the camera will be station and client (idk why but originally i did the reverse which was just stupid)

  // frame buffer for image, it's kinda big so 
  fb = (uint8_t*)  malloc(BUF_SIZE* sizeof( uint8_t ) ) ;
  fb[0] = 0;


  ts.begin();
  ts.setCalibration(xMin, xMax, yMin, yMax);
  // ts.setRotation(1);
  
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
  y +=30;

  // char partiname[64];
  // int spiffsize;
  // int spiffusage;

  // if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
  //     Serial.println("SPIFFS Mount Failed");
  //     return;
  // }
  // copyFileintoRAM(fb, SPIFFS, PHOTO_URI);
  // drawImagefromRAM((const char*) fb, BUF_SIZE);

  tft.drawCentreString("General__Touch Screen to Start__Kenobi", x, y, fontSize);
  drawButtons();
  screenReady = true;
  delay(1000);
} //setup


int loopcount = 0;
void loop() {

  if (fbReady) {      //all of the display commands will be done in this thread to avoid badness 
    fbReady = false;
    drawImagefromRAM((const char*) writefb, writefb_i);
    tft.drawCentreString("got an image", 320/2, 195, 2);
    tft.drawNumber(frameCount, 320/2, 210);
    frameCount++;
  }
  
  //adding in code to register button presses, possibly (unless I can do it with interrupts / callbacks)
  TouchPoint p = ts.getTouch();
  if (p.zRaw > 100) {
  printTouchToSerial(p);
  printTouchToDisplay(p);
  // key[0].drawButton(false);
  // key[1].drawButton(false);

  }
  for (uint8_t i = 0; i < 2; i++) {
    if (key[i].contains(p.x,p.y)) {
        key[i].press(true);
    } else {
        key[i].press(false);
    }
  }

  for (uint8_t i = 0; i < 2; i++) {
    if (key[i].justPressed()) {
      handleButtonPress(i);
      key[i].drawButton(true);
    } else if (key[i].justReleased()) {
      key[i].drawButton(false);
    }
  }

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
    int decodeStatus = jpeg.decode(0, 0, 0);                       //for full size display
    // int decodeStatus = jpeg.decode(imagePosition, 0, JPEG_SCALE_HALF);   //for one-quarter size (half in each dimension)
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
  //header is FFD8, footer is FFD9             {1111 1111, 1101 1000, 1111 1111, 1101 1001}
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

    delay(2000);

    int nums = WiFi.softAPgetStationNum();
    Serial.print("Number of stations: ");
    Serial.println(nums);
}

//this inits the udp server and has the packet handler code -> very important!!!
void handleUDPServer() {   //for use by the screen (client and Access Point) 
  if (udp.listen(WiFi.softAPIP(),UDP_PORT)) {
    Serial.print("UDP listening on IP: ");
    Serial.println(WiFi.softAPIP());
    udp.onPacket([](AsyncUDPPacket packet) {


      if (!screenReady) { //to prevent the thing from crashing / heap error / badness (also, depending on the model board used, the WiFi antenna can mess with the TFT init. my board is one such case)
        return;
      }
      //this means no devices will receive a response until the screen is ready

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


      recvData = packet.data();
      recvLength = packet.length();
      idx = 0;

        switch(recvData[0]) {

          case 0x01: //initial message from camera
            targetIP = packet.remoteIP();
            targetPort = packet.remotePort();
            udp.writeTo(&ack, 0x01, targetIP, targetPort);
          break;

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
              fbReady = true;
              writefb = fb;
              writefb_i = fb_i;
              // drawImagefromRAM((const char*) fb, fb_i);
            }
            // tft.drawCentreString("got an image", 320/2, 195, 2);
            // tft.drawNumber(frameCount, 320/2, 210);
            // frameCount++;
            fb_i = 0;

          break;


          default:  //unknown packet type, don't respond
            Serial.println("Unknownpacket");
          break;

        } //end of switchcase

      if (packet.length() < 20) {


        Serial.print("UDP Packet Type: ");  Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
        Serial.print(", From: ");           Serial.print(packet.remoteIP());  Serial.print(":");        Serial.print(packet.remotePort());
        Serial.print(", To: ");             Serial.print(packet.localIP());   Serial.print(":");        Serial.print(packet.localPort());
        Serial.print(", Length: ");         Serial.print(packet.length());    
        Serial.print(", Data: ");           Serial.write(packet.data(), packet.length());   
        Serial.println();

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

  }
}

//init the button positions (will likely keep them transparent in final UX but unsure, aka don't actually redraw them during loop)
void drawButtons() {
  uint16_t bWidth = 50;
  uint16_t bHeight = 50;
  // Generate buttons with different size X deltas
  for (int i = 0; i < 2; i++) {
    key[i].initButton(&tft,
                      bWidth * 3 / 2 + i * (screenWidth - 3 * bWidth),
                      bHeight * 3 / 2,
                      bWidth,
                      bHeight,
                      TFT_WHITE,  // Outline
                      TFT_GREEN,  // Fill
                      TFT_BLACK,  // Text
                      "Hello",
                      1);

    key[i].drawButton(false, String(i + 1));
  }
}

//for debugging purposes
void printTouchToDisplay(TouchPoint p) {

  // Clear screen first
  //tft.fillScreen(TFT_BLACK);
  tft.fillRect(screenWidth/4, screenHeight/3+5, screenWidth/2, screenHeight/2-10, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  int x = 320 / 2;  // center of display
  int y = 100;
  int fontSize = 2;

  String temp = "Pressure = " + String(p.zRaw);
  tft.drawCentreString(temp, x, y, fontSize);

  y += 16;
  temp = "X = " + String(p.x);
  tft.drawCentreString(temp, x, y, fontSize);

  y += 16;
  temp = "Y = " + String(p.y);
  tft.drawCentreString(temp, x, y, fontSize);

  y += 16;
  temp = "raw XY: " + String(p.xRaw) + ", " + String(p.yRaw);
  tft.drawCentreString(temp, x, y, fontSize);
}


void printTouchToSerial(TouchPoint p) {
  Serial.print("Pressure = ");
  Serial.print(p.zRaw);
  Serial.print(", x = ");
  Serial.print(p.x);
  Serial.print(", y = ");
  Serial.print(p.y);
  Serial.println();
}

//this takes in a number and transmits the proper signal to the controller (currently using UDP, might switch to TCP if necessary)
//will rely on each individual button to debounce itself (alternative is a global debounce timer)
void handleButtonPress(int button) {
  if (!screenReady) return;   //this shouldn't even be possible but just in case

  // //                |    connect  |     direction     |  camera | espcontrol
  // uint8_t transCmds[] = {0x11,0x12,0x13,0x21,0x22,0x23,0x24,0x31,0x32,0x41     };
  // uint8_t tansCmdAmount = 11;

 
  if (!(0 <= button < cmdAmount)) {
    Serial.println("Invalid button index passed");
    return;
  }

  uint8_t byteToSend = transCmds[button];

  //will send a 4-byte packet in the following format: (0xHH is the byteToSend)
  // {0x27,0x04,0xHH,0xHH}
  //duplicating the data val in case it's corrupted



}





