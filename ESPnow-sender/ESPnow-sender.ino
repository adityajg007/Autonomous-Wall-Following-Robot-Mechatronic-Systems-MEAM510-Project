/**
 * Simple sender to only a receiver ESPNOW with known MAC hard coded to MAC_RECV
*/

#include <esp_now.h>
#include <WiFi.h> 

esp_now_peer_info_t peer1 = {
  .peer_addr = {0x84,0xF7,0x03,0xA8,0xBE,0x30}, // receiver MAC address (last digit should be even for STA)
  .channel = 1, // channel can be 1 to 14, channel 0 means current channel.  
  .encrypt = false,
};

// callback when data is sent 
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) Serial.println ("Success ");
  else Serial.println("Fail "); 
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);  
  Serial.print("Sending MAC: "); Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) {
    Serial.println("init failed"); ESP.restart();
  }
  if (esp_now_add_peer(&peer1) != ESP_OK) {
    Serial.println("Pair failed"); // ERROR should not happen
  }
}

void loop() {
  static int count;
  uint8_t message[200]; // Max ESPnow packet is 250 byte data

  // put some message together to send
  sprintf((char *) message, "sender %d ", count++);
  
  if (esp_now_send(peer1.peer_addr, message, sizeof(message))==ESP_OK) 
    Serial.printf("Sent '%s' to %x:%x:%x:%x:%x:%x \n",message, peer1.peer_addr[0],peer1.peer_addr[1],peer1.peer_addr[2],peer1.peer_addr[3],peer1.peer_addr[4],peer1.peer_addr[5]);   
  else Serial.println("Send failed");
  
  delay(500); // ESPNow max sending rate (with default speeds) is about 50Hz  
}
