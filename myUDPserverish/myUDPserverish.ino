#include "WiFi.h"
#include "AsyncUDP.h"

const char *ssid = "KiteCam";
const char *password = "flykitesgB8iINPE_cDOaT6uarecool@";
const int PRINT_CLIENT_TO_SERIAL = 1;
AsyncUDP udp;

void setup() {
  Serial.begin(115200);

  createWifiStation();
  handleUDPServer() {

  
}

void loop() {
  delay(5000);
  //Send broadcast
  udp.broadcast("Anyone here? I'm server");
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
