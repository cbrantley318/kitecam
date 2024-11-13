#include <Arduino.h>
#include <ESPNowCam.h>
#include <drivers/CamAIThinker.h>
#include "BLEDevice.h"
#include <string.h>
#include <esp_task_wdt.h>
#include <ESP32Servo.h>

#include "WiFi.h"       //changing from ESPNow to bare UDP
#include "AsyncUDP.h"

const char *ssid = "KiteCam";
const char *password = "flykitesgB8iINPE_cDOaT6uarecool@";
const bool PRINT_CLIENT_TO_SERIAL = false;
const int UDP_PORT = 1234;
const int CHUNK_SIZE = 1024;  //fpr splitting the jpeg img
const int HEADER_SIZE = 8;
AsyncUDP udp;

CamAIThinker Camera;
const int JPEG_QUALITY = 49;
// const int JPEG_QUALITY = 12；

//IP stuff
IPAddress screenIP = IPAddress(192, 168, 4, 1);
uint16_t screenPort = 1234;
bool screenConnected = false;


// ground transceiver to camera signals
//                     |    connect  |     direction     |  camera | espcontrol
uint8_t transCmds[] = {0x11,0x12,0x13,0x21,0x22,0x23,0x24,0x31,0x32,0x41     };
uint8_t transCmdAmount = 11;



//for connecting to the external camera:
#define ServiceCharacteristic(S, C) ThisClient->getService(S)->getCharacteristic(C)

static BLEDevice *ThisDevice;
static BLEClient *ThisClient = ThisDevice->createClient();
BLEScan *ThisScan = ThisDevice->getScan();
static BLEUUID ServiceUUID((uint16_t)0xFEA6);
static BLEUUID CommandWriteCharacteristicUUID("b5f90072-aa8d-11e3-9046-0002a5d5c51b");
static BLEUUID CommandResponseCharacteristicUUID("b5f90073-aa8d-11e3-9046-0002a5d5c51b");
static bool ItsOn = false;
unsigned long TimeStamp;


//gopro control signals
char BLE_ShutterOn[] = {0x03, 0x01, 0x01, 0x01};
char BLE_ShutterOff[] = {0x03, 0x01, 0x01, 0x00};
char BLE_WiFiOn[] = {0x03, 17, 01, 01};
char BLE_WiFiOff[] = {0x03, 17, 01, 00};
char BLE_ModeVideo[] = {0x03, 0x02, 0x01, 0x00};
char BLE_ModePhoto[] = {0x03, 0x02, 0x01, 0x01};
char BLE_ModeMultiShot[] = {02, 01, 02};     //idk what this is but it seems cool
char BLE_Sleep[] = {0x01, 0x05};
char BLE_KeepAlive[] = {0x01, 0x5B};

//iphone control signals
char BLE_iPhoneShutter = 0xe9;
char BLE_iPhoneRelease = 0x00; //???

//android control signals
char BLE_androidReturn = 0x40;
char BLE_androidShutter = 0x80;

 //end of main bluetooth BLE stuff

//Servo inits
#define HAS_SERVOS false
const int vertServoPin = 12;               // GPIO pin connected to the servo signal pin
Servo vertServo;
int vertServoCurrentAngle = 90;            // Current servo angle (initially set to 90 degrees)
const int servoMin = 0;                 // Minimum servo angle
const int servoMax = 180;               // Maximum servo angle
const int horizServoPin = 13;
Servo horizServo;
int horizServoCurrentAngle = 90;
const int servoStep = 5;


//misc debugging / trying to fix dual antennae
const uint8_t CMD_NONE = 30;
uint8_t cmd = CMD_NONE;
bool cmdRecvd = false;

bool BLEhasAntenna = false;
bool tryScan = false;


//Serial global vars
uint8_t UdpCmds[] = {0x11, 0x21, 0x22, 0x23, 0x24, 0x31, 0x32, 0x33, 0x34, 0x40, 0x41};
char SerialCmds[] = {'c',  'l',  'r',  'u',   'd',  'n',  'f',  'p',  'v',   's', 'm'};
//                connect, left, right,up,  down, snap, endVid,photo,vidMode, sleep, M (data dump)


void setup() {
  Serial.begin(115200);

  if (HAS_SERVOS) {
    vertServo.attach(vertServoPin, 500, 2400);  // Attach servo with specified min and max pulse widths
    vertServo.write(vertServoCurrentAngle);      // Set initial servo position
    horizServo.attach(horizServoPin, 500, 2400);  // Attach servo with specified min and max pulse widths
    horizServo.write(horizServoCurrentAngle);      // Set initial servo position
    Serial.print("Servo initialized to ");
    Serial.print(vertServoCurrentAngle);
    Serial.println(" degrees.");
    Serial.println(horizServoCurrentAngle);
  }


  // Init Bluetooth Low Energy
  // ThisDevice->init("");
  // ThisDevice->setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  
  // CYD MAC address: 08:A6:F7:23:B0:10
  uint8_t macRecv[6] = {0x08,0xA6,0xF7,0x23,0xB0,0x10};
  createWifiStation();
  handleUDPClient();
  Camera.begin();
  

}

void loop() {             //this is my current workaround for resource collision


  if (screenConnected && !BLEhasAntenna) {
    processFrame();
  }
  if (cmdRecvd) {
    cmdRecvd = false;
    // actOnCommand(cmd);
    sendSerialCommand(cmd);
  }
  // if (tryScan) {
  //     // Serial.println("------------------------Starting scan---------------------------------------------------");
  //     tryScan = false;
  //     if (ScanAndConnect()) {
  //       // Serial.println("Connected!");
  //       ItsOn = true;
  //       delay(100);
  //       // Serial.println("done delay");
  //     } else {
  //       // Serial.println("Failed to connect");
  //     }
  //     BLEhasAntenna = false;
  // }

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

void handleUDPClient() {
  if (udp.connect(IPAddress(192, 168, 4, 1), UDP_PORT)) {
    // Serial.print("UDP Connected");
    udp.onPacket([](AsyncUDPPacket packet) {

      if (BLEhasAntenna) { //attempt #1 at sharing the antenna w/o conflict
        return;
      }

      screenIP = packet.remoteIP();
      screenPort = packet.remotePort();
      screenConnected = true;

      if (packet.length() < 4) return;

      if (PRINT_CLIENT_TO_SERIAL) { //this is just to clean up the serial monitor in case I don't want to print stuff
        Serial.print("UDP Packet Type: "); Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
        Serial.print(", From: ");   Serial.print(packet.remoteIP());  Serial.print(":");        Serial.print(packet.remotePort());
        Serial.print(", To: ");     Serial.print(packet.localIP());   Serial.print(":");        Serial.print(packet.localPort());
        Serial.print(", Length: "); Serial.print(packet.length());    Serial.print(", Data: "); Serial.write(packet.data(), packet.length()); Serial.println();
      }

      uint8_t* recvData = packet.data();
      int length = packet.length();

      if (length >= 4) {
        switch(recvData[0]) {

          case 0x03:
            packet.println("received number 3");
          break;

          case 0x04:
            packet.println("received number 4");
          break;
          case 0x27:  //command incoming from the ground
            if (recvData[1] != 0x04) {   //not a valid command
              packet.println("Invalid command sent from ground");
              return;
            }

            if (recvData[2] != recvData[3]) {
              udp.print("command wasnt duplicated");
              // udp.writeTo("command was not duplicated",screenIP,screenPort);
            }
            cmd = recvData[2];
            cmdRecvd = true;
            udp.printf("command 0x%x recv'd", cmd);
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
    udp.write(0x01);
    udp.print("Hello Server!");
  }
}

void processFrame() {

  if (BLEhasAntenna) return;
  if (Camera.get()) {
    uint8_t *out_jpg = NULL;
    size_t out_jpg_len = 0;
    frame2jpg(Camera.fb, JPEG_QUALITY, &out_jpg, &out_jpg_len);
    // radio.sendData(out_jpg, out_jpg_len);
    // udp.broadcast(out_jpg,out_jpg_len);
    // Serial.println("------------------------------------");

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
      // Serial.print("Outsize: ");
      // Serial.println(outSize);
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
    // Serial.println("Cam NO image");
  }
}

// bool ScanAndConnect(void) {
//   // vTaskSuskpendAll();
//   // Serial.println("Start of scanandconn");
//   ThisScan->clearResults();
//   // Serial.println("\n____________________________________________cleared results--------------\n");
//   ThisScan->start(3);
//   // Serial.println("ran the scan-------------------------------------<<<<<<<");
//   for (int i = 0; i < ThisScan->getResults()->getCount(); i++) {
//     delay(20);
//     if (ThisScan->getResults()->getDevice(i).haveServiceUUID() && ThisScan->getResults()->getDevice(i).isAdvertisingService(BLEUUID(ServiceUUID))) {
//       ThisScan->stop();
//       ThisClient->connect(new BLEAdvertisedDevice(ThisScan->getResults()->getDevice(i)));
//       Serial.println(ThisClient->toString());
//       Serial.println("About to return from scan");

//       Serial.println(WiFi.isConnected());
//       Serial.println("Just checking");
//       return true;
//     }
//   }
//   return false;
// }


// void actOnCommand(uint8_t cmd) {

//   if (cmd == CMD_NONE) {
//     return;
//   }
//     if (cmd < 11) {
//       BLEhasAntenna = true;
//     }

//     switch(cmd) {
    
//     case 0x11: //Connect the camera (gopro)
//       tryScan = true;
//       BLEhasAntenna = true;
//     break;
//     case 0x33: //set to photo mode
//         if (ItsOn) ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue(BLE_ModePhoto, 4); //sending video mode
//     break;
//     case 0x34: //set to video mode
//         if (ItsOn) ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue(BLE_ModeVideo, 4); //sending video mode
//     break;
//     case 0x40: //tell it to sleep?
//         if (ItsOn) ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue(BLE_Sleep, 2); //sending sleep
//     break;
//     case 0x31: //shutter on (start the photo/video)
//         if (ItsOn) ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue(BLE_ShutterOn, 4);
//     break;
//     case 0x32: //shutter off (end video)
//         if (!ItsOn) return; 
//         ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue(BLE_ShutterOff, 4);
//         while (millis() - TimeStamp < 5000)
//           yield();
//         ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue(BLE_Sleep, 2); //sending sleep
//         ItsOn = false;
//     break;

//     //start of commands for moving servos / stepper (TODO)
//     case 0x21:   //left
//       adjustServoAngle(horizServo, horizServoCurrentAngle, -servoStep);
      
//     break;
//     case 0x22:   //right
//       adjustServoAngle(horizServo, horizServoCurrentAngle, servoStep);
//     break;
//     case 0x23:   //up
//       adjustServoAngle(vertServo, vertServoCurrentAngle, -servoStep);
//     break;
//     case 0x24:   //down
//       adjustServoAngle(vertServo, vertServoCurrentAngle, servoStep);
//     break;
//     //end of commands for moving motors     

//     case 0x41:  //requesting a data dump be transmitted back to the ground (TODO)

//     break;    
//     default:
//       Serial.println("Not a recognized command");
//     break;
//   }

//   BLEhasAntenna = false;
// }

void sendSerialCommand(uint8_t cmd) {


  int idx = indexOf(cmd, UdpCmds, transCmdAmount);
  char toSend = SerialCmds[idx];
  Serial.write(toSend);
  Serial.write(toSend);


}

uint8_t indexOf(uint8_t target, uint8_t* arr, uint8_t size) {
  for (int i = 0; i < size; i++) {
    if (target == arr[i]) return i;
  }
  return -1;
}

// void adjustServoAngle(Servo& myServo, int &servoCurrentAngle, int delta) {
//   int newAngle = servoCurrentAngle + delta;

//   // Ensure the new angle is within the defined limits
//   if (newAngle < servoMin) {
//     newAngle = servoMin;
//     Serial.println("Servo angle reached MIN limit.");
//   } else if (newAngle > servoMax) {
//     newAngle = servoMax;
//     Serial.println("Servo angle reached MAX limit.");
//   }

//   // Update servo position if angle has changed
//   if (newAngle != servoCurrentAngle) {
//     servoCurrentAngle = newAngle;
//     myServo.write(servoCurrentAngle);
//     Serial.print("Servo angle adjusted to: ");
//     Serial.print(servoCurrentAngle);
//     Serial.println(" degrees.");
//   }
// }
