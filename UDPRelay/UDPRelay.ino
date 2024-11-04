#include "WiFi.h"
#include "AsyncUDP.h"

const char *ssid = "KiteCam";
const char *password = "flykitesgB8iINPE_cDOaT6uarecool@";
const int PRINT_SERVER_TO_SERIAL = 0;
AsyncUDP udp;


IPAddress client1IP = IPAddress(192,168,4,2);
uint16_t client1Port = 1024;
IPAddress client2IP = IPAddress(192,168,4,3);
uint16_t client2Port = 1024;
bool isCLient1 = true;  //this isn't needed
const int UDP_PORT = 1234;

IPAddress targetIP;
uint16_t targetPort;
uint8_t ack = 0x01;


//goal is client 1 connects, it saves the client 1 IP and port, and then when client 2 connects it does the other
void setup() {
  Serial.begin(115200);

  createWifiAP();
  handleUDPServer();

}

void loop() {
  delay(100);
  if (Serial.available() > 0) {
    Serial.readString();
    Serial.print("Client 1: "); Serial.print(client1IP); Serial.print(", "); Serial.println(client1Port);
    Serial.print("Client 2: "); Serial.print(client2IP); Serial.print(", "); Serial.println(client2Port);
  }
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

// void handleUDPServer() {
//   if (udp.listen(1234)) {
//     Serial.print("UDP Listening on IP: ");
//     Serial.println(WiFi.localIP());
//     udp.onPacket([](AsyncUDPPacket packet) {

//       if (PRINT_CLIENT_TO_SERIAL) { //this is just to clean up the serial monitor in case I don't want to print stuff
//         Serial.print("UDP Packet Type: "); Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
//         Serial.print(", From: ");   Serial.print(packet.remoteIP());  Serial.print(":");        Serial.print(packet.remotePort());
//         Serial.print(", To: ");     Serial.print(packet.localIP());   Serial.print(":");        Serial.print(packet.localPort());
//         Serial.print(", Length: "); Serial.print(packet.length());    Serial.print(", Data: "); Serial.write(packet.data(), packet.length()); Serial.println();
//       }

//       //reply to the client
//       packet.printf("Got %u bytes of data", packet.length());
//     });
//   }
// }


void handleUDPServer() {   //this is the only server, it will accept packets from each client and it just forwards them to the other client
  if (udp.listen(WiFi.softAPIP(),UDP_PORT)) {
    Serial.print("UDP listening on IP: ");
    Serial.println(WiFi.softAPIP());
    udp.onPacket([](AsyncUDPPacket packet) {

      if (PRINT_SERVER_TO_SERIAL) { //this is just to clean up the serial monitor in case I don't want to print stuff
        Serial.print("UDP Packet Type: ");  Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
        Serial.print(", From: ");           Serial.print(packet.remoteIP());  Serial.print(":");        Serial.print(packet.remotePort());
        Serial.print(", To: ");             Serial.print(packet.localIP());   Serial.print(":");        Serial.print(packet.localPort());
        Serial.print(", Length: ");         Serial.print(packet.length());    
        Serial.print(", Data: ");           Serial.write(packet.data(), packet.length());   
        Serial.println();
      }

        if (packet.remoteIP() != client1IP) { //if it doesn't match client 1, we set it to client 2 and transmit to client 1. otherwise do opposite 
          client2IP = packet.remoteIP();
          client2Port = packet.remotePort();
          targetIP = client1IP;
          targetPort = client1Port;
        } else {
          client1IP = packet.remoteIP();
          client1Port = packet.remotePort();
          targetIP = client2IP;
          targetPort = client2Port;
        }

      if (packet.data()[0] == 0x01) {  //respond to the one that sent it
        udp.writeTo(&ack, 0x01, packet.remoteIP(), packet.remotePort());
        return;
      }

      udp.writeTo(packet.data(), packet.length(), targetIP, targetPort);  //all this does is forward the packets to the other guy

    });

  }
}

