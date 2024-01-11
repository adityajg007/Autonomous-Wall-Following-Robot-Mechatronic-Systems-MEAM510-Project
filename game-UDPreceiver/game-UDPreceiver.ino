/*
 * receives udp text packets and print packets to monitor
 *  using station mode (needs router) and static IP
 */
#include <WiFi.h>
#include <WiFiUdp.h>

#define UDPPORT 2510 // port for game obj transmission
WiFiUDP UDPServer;
IPAddress myIPaddress(192, 168, 1, 204); // Andy Xiao's IP

// uncomment the router SSID and Password that is being used 

//const char* ssid     = "TP-Link_05AF";
//const char* password = "47543454";

const char* ssid     = "TP-Link_E0C8";
const char* password = "52665134";

//const char* ssid     = "TP-Link_FD24"; 
//const char* password = "65512111";

void handleUDPServer() {
   const int UDP_PACKET_SIZE = 14; // can be up to 65535          
   uint8_t packetBuffer[UDP_PACKET_SIZE];

   int cb = UDPServer.parsePacket(); // if there is no message cb=0
   if (cb) {
      int x,y;
      packetBuffer[13]=0; // null terminate string

    UDPServer.read(packetBuffer, UDP_PACKET_SIZE);
      x = atoi((char *)packetBuffer+3); // ##,####,#### 2nd indexed char
      y = atoi((char *)packetBuffer+8); // ##,####,#### 7th indexed char
      Serial.print("From Team ");
      Serial.println((char *)packetBuffer);
      Serial.println(x);
      Serial.println(y);
   }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  WiFi.config( myIPaddress,        // Device IP address
      IPAddress(192, 168, 1, 1),   // gateway (not important for 5100)
      IPAddress(255, 255, 255, 0)); // net mask 
  
  UDPServer.begin(UDPPORT);  // 2510 forgame  arbitrary UDP port# need to use same one   
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);    Serial.print(".");
  }
  Serial.printf("WiFi connected to %s\n", ssid);
  Serial.print("Using static IP "); Serial.print(myIPaddress); 
  Serial.print(" and UDP port "); Serial.println(UDPPORT);
}

void loop() {
  handleUDPServer();
  delay(10);
}
