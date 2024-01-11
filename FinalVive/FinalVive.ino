/*
 * broadcasts data packets to port 2510 
 * Demonstrates high precision vive localization and displays on RGB LED
 * 
 */
#include "vive510.h"
#include <WiFi.h>
#include <WiFiUdp.h>

#define RGBLED 2 // for ESP32S2 Devkit pin 18, for M5 stamp=2
#define SIGNALPIN1 19 // pin receiving signal from Vive circuit
#define UDPPORT 2510 // For GTA 2022C game 
#define STUDENTIP 204 // Andy's IP number
#define teamNumber 31 
#define FREQ 1 // in Hz

const char* ssid     = "TP-Link_E0C8";
const char* password = "52665134";

Vive510 vive1(SIGNALPIN1);

WiFiUDP UDPTestServer;
IPAddress ipTarget(192, 168, 1, 255); // 255 => broadcast

void UdpSend(int x, int y) {
  char udpBuffer[20];
  sprintf(udpBuffer, "%02d:%4d,%4d",teamNumber,x,y);                                                
  UDPTestServer.beginPacket(ipTarget, UDPPORT);
  UDPTestServer.println(udpBuffer);
  UDPTestServer.endPacket();
  Serial.println(udpBuffer);
}
               
void setup() {
  int i = 0;
  Serial.begin(115200);

  WiFi.mode(WIFI_AP_STA);
  WiFi.config(IPAddress(192, 168, 1, STUDENTIP), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);

  Serial.printf("team  #%d ", teamNumber); 
  Serial.print("Connecting to ");  Serial.println(ssid);
  while(WiFi.status()!=WL_CONNECTED && i++ < 20){
    delay(500);   Serial.print(".");
  }
  if (i < 19) {
    Serial.println("WiFi connected as "); Serial.print(WiFi.localIP());
  } else {
    Serial.printf("Could not connect err: %d ",i); 
  }
  UDPTestServer.begin(UDPPORT);
  vive1.begin();
  Serial.println("Vive trackers started");
}
                                 
void loop() {  
  static long int ms = millis();
  static uint16_t x,y;

  if (millis()-ms>1000/FREQ) {
    ms = millis();
    if (WiFi.status()==WL_CONNECTED)
        neopixelWrite(RGBLED,255,255,255);  // full white
    UdpSend(x,y);
  }
  
  if (vive1.status() == VIVE_RECEIVING) {
    x = vive1.xCoord();
    y = vive1.yCoord();
    Serial.printf("X %d, Y %d\n", vive1.xCoord(), vive1.yCoord());
    neopixelWrite(RGBLED, 0, x/200, y/200);  // blue to greenish
  }
  else {
    x=0;
    y=0; 
    switch (vive1.sync(5)) {
      break;
      case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
        neopixelWrite(RGBLED,64,32,0);  // yellowish
      break;
      default:
      case VIVE_NO_SIGNAL: // nothing detected     
        neopixelWrite(RGBLED,128,0,0);  // red
    }
  }
    
  delay(20);
}
