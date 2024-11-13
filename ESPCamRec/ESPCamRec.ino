
/**************************************************
 * Cameran video Receiver
 * by Carson Brantley, Dylan Nguyen 
 * Copyright (C) 2024
 **************************************************/

#include <Arduino.h>
//#include <ESPNowCam.h>

#include "WiFi.h"
#include "AsyncUDP.h"

const char *ssid = "KiteCam";
const char *password = "flykitesgB8iINPE_cDOaT6uarecool@";
#define AP_MODE true

const int PRINT_SERVER_TO_SERIAL = 0;
const int UDP_PORT = 1234;
AsyncUDP udp;

IPAddress targetIP = IPAddress(192,168,4,2);
uint16_t targetPort = 0;


#include <FS.h>
#include <SPIFFS.h>
//#include <Utils.h>
#define FORMAT_SPIFFS_IF_FAILED true


#include <JPEGDEC.h>

JPEGDEC jpeg;
const char *PHOTO_URI = "/carson.jpg";
const int BUF_SIZE = 16384;


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

const int NUM_BUTTONS = 6;
TFT_eSPI tft = TFT_eSPI();
TFT_eSPI_Button key[NUM_BUTTONS];
bool screenReady = false;

// recording variables - to handle logic for this more rigorously
bool isPhotoMode = true;
bool isRecording = false;
uint16_t shutterFill = TFT_WHITE;  // White for photo mode, Red for video mode
uint16_t shutterOutline = TFT_BLACK;  // Black for photo mode, Red for video mode

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

//control signals to transmit
//                     |    connect  |     direction     |  camera | espcontrol
// uint8_t transCmds[] = {0x11,0x12,0x13,0x21,0x22,0x23,0x24,0x31,0x32,0x41     };
// new
// ^: 0x21, v: 0x22, <: 0x23, >: 0x24, SHUTTER: 0x31, ENDVID: 0x32, PHOTO: 0x33, VIDEO: 0x34
uint8_t transCmds[] = {0x21, 0x22, 0x23, 0x24, 0x31, 0x32, 0x33, 0x34};
uint8_t transCmdAmount = 11;


void setup() {
  Serial.begin(115200);

  delay(500);


  if (AP_MODE) {
    //AP MODE 
    createWifiAP();     //the screen is the wifi AP and the server
    handleUDPServer();  //the camera will be station and client (idk why but originally i had this as AP and client which was just bad)
  } else {
    //STATION MODE (using another as relay)
    createWifiStation();
    handleUDPClient();
  }


  // frame buffer for image, it's kinda big so 
  fb = (uint8_t*)  malloc(BUF_SIZE* sizeof( uint8_t ) ) ;
  fb[0] = 0;


  // this is for initializing the on-chip file system (SPIFFS)
    char partiname[64];
    int spiffsize;
    int spiffusage;

    if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
        Serial.println("SPIFFS Mount Failed");
        return;
    }
    // copyFileintoRAM(fb, SPIFFS, PHOTO_URI);
    // drawImagefromRAM((const char*) fb, BUF_SIZE);
  

  //initialize the touch screen (will want to make this and the display into methods like wifi and UDP)
  ts.begin();
  ts.setCalibration(xMin, xMax, yMin, yMax);

  //initialize the display
  tft.init();
  tft.setRotation(1);
  tft.invertDisplay(0);   //only include if using the other brand of display
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
  // tft.drawCentreString("Hello___Touch Screen to Start____There", x, y, fontSize);
  y +=30;
  // tft.drawCentreString("General__Touch Screen to Start__Kenobi", x, y, fontSize);
  initButtons();
  screenReady = true;
  delay(1000);
} //setup


int loopcount = 0;
void loop() {
    if (fbReady) {
        fbReady = false;
        drawImagefromRAM((const char*)writefb, writefb_i); // Draws the photo
        frameCount++;
    }
  
    TouchPoint p = ts.getTouch();
    if (p.zRaw > 100) {
        // printTouchToSerial(p);
        // printTouchToDisplay(p);
    }

    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        if (p.zRaw > 100 && key[i].contains(p.x, p.y)) {
            key[i].press(true);
        } else {
            key[i].press(false);
        }
    }

    // Draw all buttons and circles every frame
    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        if (key[i].justPressed()) {
            handleButtonPress(i);
            Serial.print(i);
        }
        if (i == 4) { // shutter button
          key[i].drawFilledButton(key[i].isPressed()); // Draw button with current state
        } else {
          key[i].drawButton(key[i].isPressed()); // Draw button with current state
        }
    }

    delay(10);
}//loop()

void copyRAMintoFile(uint8_t* buf, int len, fs::FS &fs, const char * path) {
    // Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
      if(!file){
          Serial.println("- failed to open file for writing");
          return;
      }
      if(file.write(buf, len)){
          Serial.println("- file written");
      } else {
          Serial.println("- write failed");
      }
      file.close();
}

int drawImagefromRAM(const char *imageBuffer, int size) {
    unsigned long lTime = millis();
    lTime = millis();
    jpeg.openRAM((uint8_t *)imageBuffer, size, JPEGDraw);
    jpeg.setPixelType(1);
    int imagePositionX = 0;  // Start at the top-left corner
    int imagePositionY = 0;  // Start at the top-left corner
    // Decode the image at full size
    int decodeStatus = jpeg.decode(imagePositionX, imagePositionY, 0);  // 0 means no scaling
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
            }
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
        udp.writeTo(&sendData, 0x01, targetIP, targetPort);

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

void handleUDPClient() {   //for use by the screen (client and Access Point) 
  if (udp.connect(IPAddress(192, 168, 4, 1), UDP_PORT)) {
    Serial.println("UDP connected");
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
            }
            fb_i = 0;
          break;
          default:  //unknown packet type, don't respond
            Serial.println("Unknownpacket");
          break;

        } //end of switchcase
    });
  udp.write(0x01);
  }
}   //handleUDPClient


void initButtons() { 
  uint16_t bWidth = 50;
  uint16_t bHeight = 50;

  uint16_t sH = screenHeight;
  uint16_t sW = screenWidth;

  uint16_t LRw = 50;
  uint16_t LRh = 140;
  uint16_t UDw = 220;
  uint16_t UDh = 50;

  uint16_t mW = 50;
  uint16_t mH = 50;

  // Directional chevrons
  const char* chevrons[] = {"<", ">", "^", "v"}; // Left, Right, Up, Down
  uint16_t chevronX[] = {0, sW-LRw, 50, 50};
  uint16_t chevronY[] = {50, 50, 0, sH-UDh};
  uint16_t chevWidth[] = {50, 50, 220, 220};
  uint16_t chevHeight[] = {140, 140, 50, 50};


  // Draw directional buttons with chevrons
  for (int i = 0; i < 4; i++) {
    key[i].initButtonUL(&tft,
                        chevronX[i],
                        chevronY[i],
                        chevWidth[i],  // Width
                        chevHeight[i],  // Height
                        TFT_BLACK,  // No Outline
                        TFT_BLACK,  // Fill
                        TFT_WHITE,  // Text
                        (char*)chevrons[i],  // Cast to char*
                        2);
    key[i].drawButton(false, (char*)chevrons[i]);
  }

  // Circular button for shutter/record
  uint16_t shutterX = sW - bWidth - 10;  // Top-left corner of the square
  uint16_t shutterY = sH - bHeight - 10;  // Top-left corner of the square

  key[4].initButtonUL(&tft,
                      shutterX,
                      shutterY,
                      bWidth,
                      bHeight,
                      shutterOutline,  // No Outline
                      shutterFill,  // Fill
                      TFT_BLACK,  // Text (optional)
                      "",  // No text
                      1);
  key[4].drawFilledButton(false, ""); // Changed to drawFilledButton

  // Toggle button for photo/video mode
  uint16_t toggleX = 0;
  uint16_t toggleY = sH - bHeight;
  char* modeText = (char*)(isPhotoMode ? "PHOTO" : "VIDEO");

  key[5].initButtonUL(&tft,
                      toggleX,
                      toggleY,
                      bWidth,
                      bHeight,
                      TFT_BLACK,  // No Outline
                      TFT_BLACK,  // Fill
                      TFT_WHITE,  // Text
                      modeText,  // Cast to char*
                      1);
  key[5].drawButton(false, modeText);
}

//for debugging purposes
void printTouchToDisplay(TouchPoint p) {

  // Clear screen first
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
void handleButtonPress(uint8_t button) {
  if (!screenReady) return;

  uint8_t byteToSend;
  if (button < 4) {
    byteToSend = transCmds[button];
  } else if (button == 4) { // shutter button pressed
    if (isPhotoMode) {
      byteToSend = transCmds[4]; // shutter photo
    } else if (!isPhotoMode && isRecording) {
      isRecording = false;
      byteToSend = transCmds[5]; // end video
    } else if (!isPhotoMode && !isRecording) { // start recording
      isRecording = true;
      byteToSend = transCmds[4]; // start video
    }
  } else if (button == 5) { // photo/video mode toggle
    if (!isPhotoMode && !isRecording) {
      isPhotoMode = true;
      byteToSend = transCmds[6]; // switch to photo
    } else if (!isPhotoMode && isRecording) {
      isRecording = false;
      byteToSend = transCmds[5]; // end video
    } else if (isPhotoMode) {
      isPhotoMode = false;
      byteToSend = transCmds[7]; // switch to video
    }
    updateModeButton(); // Update the mode button text and redraw
  }

  updateShutterButton(); // Update the shutter button color and redraw

  uint8_t toSend[4] = {0x27, 0x04, byteToSend, byteToSend};
  udp.writeTo(toSend, 0x04, targetIP, targetPort);
}

void updateShutterButton() {
    shutterFill = (!isPhotoMode && isRecording) ? TFT_RED : TFT_WHITE;

    uint16_t shutterX = screenWidth - 50 - 10;  // Adjusted position
    uint16_t shutterY = screenHeight - 50 - 10; // Adjusted position

    uint16_t outlineColor = (!isPhotoMode) ? TFT_RED : TFT_BLACK;

    key[4].initButtonUL(&tft,
                        shutterX,
                        shutterY,
                        50,  // Width
                        50,  // Height
                        outlineColor,  // Outline color
                        shutterFill,  // New Fill Color
                        TFT_BLACK,  // Text (optional)
                        "",  // No text
                        1);
    key[4].drawFilledButton(false, ""); // Changed to drawFilledButton
}

void updateModeButton() {
  const char* modeText = isPhotoMode ? "PHOTO" : "VIDEO";
  
  // Reinitialize the button with the new label
  key[5].initButtonUL(&tft,
                      0,  // X position
                      screenHeight - 50,  // Y position
                      50,  // Width
                      50,  // Height
                      TFT_BLACK,  // No Outline
                      TFT_BLACK,  // Fill
                      TFT_WHITE,  // Text
                      (char*)modeText,  // Cast to char*
                      1);
  key[5].drawButton(false);
}



