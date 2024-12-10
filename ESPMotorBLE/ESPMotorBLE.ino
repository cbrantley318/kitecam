// this is a sketch for the esp module that will:
/*
  Control the GoPro via bluetooth (just make it autoconnect on power up, and if it's not connected then have it scan until it is)
  Listen to ALL the commands received via serial and execute them. This includes the following:
    Camera controls (for now have it ignore 'connect'. implement it the way described above)
    Pan motor controls: this is left/right and will involve telling the motor to move and then setting a timeout
      Conversely, it could mean telling the motors to move until we receive the stop command but that's harder and also bad
    Set gimbal position (this is for adjusting pitch and technically roll if we want). This will write pwm values to the gimbal controller
  Possible additional functionality: have a pin that is pulled high to turn on a relay or something in order to ensure the other components are turned off if no power (this is an anti-explosion measure)

*/

//    WIRING SPECIFICATION:
/*
  Run the pwm and dc motor control pins through the level shifter first. Power gimbal off 12V boost. Power panning motor off 12V for now (can adjust lower)
  Might want to add a 5A fuse (or 10A), might want to add a relay that kills the motors if espp32 is off

*/

#include "BLEDevice.h"
#include <ESP32Servo.h>


// ------------- Bluetooth Low Energy config ----------------------

#define ServiceCharacteristic(S, C) ThisClient->getService(S)->getCharacteristic(C)

static BLEDevice *ThisDevice;
static BLEClient *ThisClient = ThisDevice->createClient();
static BLEAdvertisedDevice* ClientAdvDev; 
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


// -------------------- DC Motor config ---------------------------

const int motor1pin1 = 14;
const int motor1pin2 = 27;
const int motor1pin2follower = 26; //this is a software fix to a hardware bug (soldered two pins together so forcing 26 to follow 27)
long onTime = 0;
bool panning = false;
const int PAN_TIMEOUT = 250;
const uint8_t MOTOR_SPEED = 10; //a number in range 0 - 255;
// -------------------- Gimbal config -----------------------------
const int rollServoPin = 24;
Servo rollServo;
int rollServoCurrentAngle = 90; 

const int pitchServoPin = 25;
Servo pitchServo;
int pitchServoCurrentAngle = 90; 

const int servoMin = 0;
const int servoMax = 180;
const int stepSize = 2; //this is how much the pwm value changes in us

char lastInbound = 0;

void setup() {
  Serial.begin(115200);    //Hardware Serial of ESP32
  while (!Serial) {
    delay(10); // Wait for serial port to connect (only needed for native USB)
  }

    //init the motor pins
    pinMode(motor1pin1, OUTPUT);
    pinMode(motor1pin2, OUTPUT);
    pinMode(motor1pin2follower, OUTPUT);

    //init the gimbal controller (treat it like a servo)
    pinMode(pitchServoPin, OUTPUT); //this is prbably not needed
    pitchServo.attach(pitchServoPin, 500, 2400);  // Attach servo with specified min and max pulse widths
    pitchServo.write(pitchServoCurrentAngle);      // Set initial servo position
    rollServo.attach(rollServoPin, 500, 2400);  // Attach servo with specified min and max pulse widths
    // rollServo.write(rollServoCurrentAngle);      // Set initial servo position

    delay(100);

    // Init Bluetooth Low Energy
    ThisDevice->init("");
    ThisDevice->setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);

    delay(200);

    if (ScanAndConnect()) {
      ItsOn = true;
    }
}

void loop() {

  if (panning && (millis() - onTime) > PAN_TIMEOUT) { //sanity check, stop panning after a certain time unless we receive a new time update
    // Serial.println("s_");
    handleSerialCommand('s');
  }

  if (Serial.available()>=1) {
    char inbound = Serial.read();
    // Serial.println(inbound);
    if (inbound == lastInbound) {
      // Serial.println(inbound);
      handleSerialCommand(lastInbound);
      inbound = 0;
      delay(5);
    }
    lastInbound = inbound;
  }
}

//this handles servo directions, camera mode switch, and camera shutter on/off
void handleSerialCommand(char cmd) {
  switch(cmd) {
    //motor controls / pan and tilt
    case 'l':
      analogWrite(motor1pin1, 100);
      analogWrite(motor1pin2, 0);
      analogWrite(motor1pin2follower, 0);
      onTime = millis();
      panning = true;
      break;
    case 'r':
      analogWrite(motor1pin1, 0);
      analogWrite(motor1pin2, 100);
      analogWrite(motor1pin2follower, 100);
      onTime = millis();
      panning = true;
      break;
    case 's':
      analogWrite(motor1pin1, 0);
      analogWrite(motor1pin2, 0);
      analogWrite(motor1pin2follower, 0);
      panning = false;
      break;
    case 'u':   //tilt up
      adjustServoAngle(pitchServo, pitchServoCurrentAngle, 1*stepSize);
      break;
    case 'd':   //tilt down
      adjustServoAngle(pitchServo, pitchServoCurrentAngle, -1*stepSize);
      break;

    //gopro cmds
    case 'n': //shutter on (take photo)
      if (ItsOn) ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue(BLE_ShutterOn, 4);
      break;
    case 'f':
      if (ItsOn) ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue(BLE_ShutterOff, 4);
      break;
    case 'p':
      if (ItsOn) ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue(BLE_ModePhoto, 4);
      break;
    case 'v':
      if (ItsOn) ServiceCharacteristic(ServiceUUID, CommandWriteCharacteristicUUID)->writeValue(BLE_ModeVideo, 4);
      break;
    
    //connect / disconnect from gopro
    case 'c': //connect to the device if not already
      if (ThisClient == 0) {
        ItsOn = ScanAndConnect();
        break;
      }
      if (ItsOn && !ThisClient->isConnected()) {
        if (ThisClient->connect(ClientAdvDev)) {
          Serial.write('w');
          Serial.write('w');
        } else {            //sending back whether or not we successfully connected (ESPcamTrans needs to then forward this along)
          Serial.write('x');
          Serial.write('x');
        }
      }
    break;
    case 'b': //disconnect to make room for the iPhone to connect to goPro
      if (ItsOn && ThisClient && ThisClient->isConnected()) {
        ThisClient->disconnect();
        Serial.write('x');
        Serial.write('x');
      }
      break;
    default:
      break;
  } //end of switch-case

}

bool ScanAndConnect(void) {
  ThisScan->clearResults();
  ThisScan->start(3);
  for (int i = 0; i < ThisScan->getResults()->getCount(); i++) {
    delay(20);
    if (ThisScan->getResults()->getDevice(i).haveServiceUUID() && ThisScan->getResults()->getDevice(i).isAdvertisingService(BLEUUID(ServiceUUID))) {
      ThisScan->stop();
      ClientAdvDev = new BLEAdvertisedDevice(ThisScan->getResults()->getDevice(i));
      ThisClient->connect(ClientAdvDev);
      //Serial.println(ThisClient->toString());
      return true;
    }
  }
  return false;
}

void adjustServoAngle(Servo& myServo, int &servoCurrentAngle, int delta) {
  int newAngle = servoCurrentAngle + delta;

  // Ensure the new angle is within the defined limits
  if (newAngle < servoMin) {
    newAngle = servoMin;
    // Serial.println("Servo angle reached MIN limit.");
  } else if (newAngle > servoMax) {
    newAngle = servoMax;
    // Serial.println("Servo angle reached MAX limit.");
  }

  // Update servo position if angle has changed
  if (newAngle != servoCurrentAngle) {
    servoCurrentAngle = newAngle;
    myServo.write(servoCurrentAngle);
    // Serial.print("Servo angle adjusted to: ");
    // Serial.print(servoCurrentAngle);
    // Serial.println(" degrees.");
  }
}








