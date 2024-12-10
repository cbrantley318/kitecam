#include <Arduino.h>
#include <ESPNowCam.h>
#define CAM_IS_AI_THINKER
#ifdef CAM_IS_AI_THINKER
  #include <drivers/CamAIThinker.h>
#else
  #include <drivers/CamFreenoveWR.h>
#endif

#include "BLEDevice.h"
#include <string.h>
#include <esp_task_wdt.h>
#include <ESP32Servo.h>

// #define COMM_USING_UDP

#include "WiFi.h"       //changing from ESPNow to bare UDP
#include "AsyncUDP.h"

const char *ssid = "KiteCam";
const char *password = "flykitesgB8iINPE_cDOaT6uarecool@";
const bool PRINT_CLIENT_TO_SERIAL = false;
const int UDP_PORT = 1234;
const int CHUNK_SIZE = 240;  //fpr splitting the jpeg img
const int HEADER_SIZE = 8;
AsyncUDP udp;
#ifdef CAM_IS_AI_THINKER
  CamAIThinker Camera;
#else
  CamFreenoveWR Camera;
#endif
const int JPEG_QUALITY = 70;
// const int JPEG_QUALITY = 12;

//IP stuff
IPAddress screenIP = IPAddress(192, 168, 4, 1);
uint16_t screenPort = 1234;
bool screenConnected = false;

//ESP-NOW stuff
// uint8_t broadcastAddress[] = {0x94, 0x54, 0xc5, 0xe7, 0x3c, 0xe0};
uint8_t broadcastAddress[] = {0x08, 0xA6, 0xF7, 0x23, 0xB0, 0x10};
esp_now_peer_info_t peerInfo;
bool success = true;



// ground transceiver to camera signals
//                     |    connect  |     direction     |  camera | espcontrol
uint8_t transCmds[] = {0x11,0x12,0x13,0x21,0x22,0x23,0x24,0x31,0x32,0x40,0x41, 0x55};
uint8_t transCmdAmount = 12;



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
uint8_t UdpCmds[] = {0x11, 0x21, 0x22, 0x23, 0x24, 0x31, 0x32, 0x33, 0x34, 0x40, 0x41, 0x55};
char SerialCmds[] = {'c',  'l',  'r',  'u',   'd',  'n',  'f',  'p',  'v',  'b', 'm', 's'};
//                connect, left, right,up,  down, snap, endVid,photo,vidMode, sleep, M (data dump), Stop motors (s)


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
  

  #ifdef COMM_USING_UDP
    //UDP over WiFi communication
    createWifiStation();
    handleUDPClient();
  #else
    //ESP-NOW communication
    Serial.println("ESP-NOW initing");
      WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }

    esp_now_register_send_cb(OnDataSent);
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    // esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    esp_now_register_recv_cb(OnDataRecv);


  #endif



  Camera.begin();
  sensor_t * s = esp_camera_sensor_get();
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  
}

void loop() {             //this is my current workaround for resource collision

  #ifdef COMM_USING_UDP
  if (!WiFi.isConnected()) {
    WiFi.reconnect();
    delay(3000);
    if (!WiFi.isConnected()) {
      WiFi.begin(ssid, password);
    }
  }

  if (screenConnected && !BLEhasAntenna) {
    processFrame();
  }
  #else
    processFrame();
  #endif


  if (cmdRecvd) {
    cmdRecvd = false;
    // actOnCommand(cmd);
    sendSerialCommand(cmd);
  }
  delay(20);
}

void createWifiStation() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while (1) {
      delay(10000);
      if (WiFi.isConnected()) {
        break;
      }
      WiFi.begin(ssid,password);
    }
  }
  Serial.println("WiFi Conected");
  Serial.println(WiFi.localIP());
}

void handleUDPClient() {
  if (udp.connect(IPAddress(192, 168, 4, 1), UDP_PORT)) {
    Serial.print("UDP Conected");
    udp.onPacket([](AsyncUDPPacket packet) {

      if (BLEhasAntenna) { //attempt #1 at sharing the antenna w/o conflict
        return;
      }

      screenIP = packet.remoteIP();
      screenPort = packet.remotePort();
      screenConnected = true;

      if (packet.length() < 4) return;

      if (PRINT_CLIENT_TO_SERIAL) {
        Serial.print("UDP Packet Type: "); Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
        Serial.print(", From: ");   Serial.print(packet.remoteIP());  Serial.print(":");        Serial.print(packet.remotePort());
        Serial.print(", To: ");     Serial.print(packet.localIP());   Serial.print(":");        Serial.print(packet.localPort());
        Serial.print(", Length: "); Serial.print(packet.length());    Serial.print(", Data: "); Serial.write(packet.data(), packet.length()); Serial.println();
      }

      uint8_t* recvData = packet.data();
      int length = packet.length();

      if (length >= 4) {
        switch(recvData[0]) {

          // case 0x03:
          //   packet.println("received number 3");
          // break;

          // case 0x04:
          //   packet.println("received number 4");
          // break;
          case 0x27:  //command incoming from the ground
            if (recvData[1] != 0x04) {   //not a valid command
              packet.println("Invalid command sent from ground");
              return;
            }

            if (recvData[2] != recvData[3]) {
              udp.print("comand wasnt duplicated");
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

void OnDataRecv(const esp_now_recv_info_t* messageInfo, const uint8_t* incomingData, int len) {
  uint8_t* recvData = (uint8_t*)incomingData;
  int length = len;

  if (len >= 4) {
    switch(recvData[0]) {

      // case 0x03:
      //   packet.println("received number 3");
      // break;
      // case 0x04:
      //   packet.println("received number 4");
      // break;
      case 0x27:  //command incoming from the ground
        if (recvData[1] != 0x04) {   //not a valid command
          return;
        }

        if (recvData[2] != recvData[3]) {
          // udp.writeTo("command was not duplicated",screenIP,screenPort);
        }
        cmd = recvData[2];
        cmdRecvd = true;
      break;
      default:
      break;
    }
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  return;
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = true;
  }
  else{
    success = false;
  }
}

void processFrame() {

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

      #ifndef COMM_USING_UDP
        while (!success) {
          delay(5);
        }
        delay(5);

      #endif

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

      #ifdef COMM_USING_UDP
        udp.writeTo(outBuf, outSize, screenIP, screenPort);
      #else
        esp_err_t result = esp_now_send(broadcastAddress, outBuf, outSize);
        //send using esp-now
      #endif

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

    // Serial.print(out_jpg[0], HEX);Serial.print(out_jpg[1], HEX);Serial.print(" ");Serial.print(out_jpg[out_jpg_len-2], HEX);Serial.println(out_jpg[out_jpg_len-1], HEX);
    
    free(outBuf);
    free(out_jpg);
    Camera.free();
  } else {
    // Serial.println("Cam NO image");
  }
}

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