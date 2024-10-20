#include "WiFi.h"
#include "AsyncUDP.h"


//constant declarations (I prefer this to #DEFINE but whatever)

const int PRINT_SERVER_TO_SERIAL = 1;
const char *ssid = "KiteCam";
const char *password = "flykitesgB8iINPE_cDOaT6uarecool@";
AsyncUDP udp;

void setup() {
  Serial.begin(115200);
  
  createWifiAP()
  handleUDPClient();

}

void loop() {
  delay(5000);
  //Send broadcast on port 1234
  udp.broadcastTo("It's client time", 1234);
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

void handleUDPClient() {   //for use by the camera (client and Access Point)
  if (udp.connect(IPAddress(192, 168, 4, 2), 1234)) {
    Serial.println("UDP connected");
    udp.onPacket([](AsyncUDPPacket packet) {

      if (PRINT_SERVER_TO_SERIAL) { //this is just to clean up the serial monitor in case I don't want to print stuff
        Serial.print("UDP Packet Type: ");  Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
        Serial.print(", From: ");           Serial.print(packet.remoteIP());  Serial.print(":");        Serial.print(packet.remotePort());
        Serial.print(", To: ");             Serial.print(packet.localIP());   Serial.print(":");        Serial.print(packet.localPort());
        Serial.print(", Length: ");         Serial.print(packet.length());    Serial.print(", Data: "); Serial.write(packet.data(), packet.length()); Serial.println();

      }
      //reply to the client
      packet.printf("Got %u bytes of data", packet.length());
    });
    //Send unicast
    udp.print("Hello Server!");
  }
}
