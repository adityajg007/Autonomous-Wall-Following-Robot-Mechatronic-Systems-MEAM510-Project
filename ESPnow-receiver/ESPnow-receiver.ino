/**
 *  ESP NOW testing generic receiver no sending
 *  Simple setup, print packets as received.
**/

#include <esp_now.h>
#include <WiFi.h>

// callback on receive
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  Serial.printf(" Recv from: %02x:%02x:%02x:%02x:%02x:%02x ",mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]); 
  Serial.print(" Data: "); 
  if (isascii(data[0])) Serial.println( (char *)data);
  else {
    for (int i=0; i<data_len; i++ ){
      Serial.printf("%x",data[i]); if (i%3==0) Serial.print(" ");
    }
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.print("ESPNow Receiving MAC: ");  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
  esp_now_register_recv_cb(OnDataRecv);    
}

void loop() {
}
