// Slimmed down ESP-NOW Sender for ESP32-WROOM with PS3 Controller Input
#include <esp_now.h>
#include <WiFi.h>
#include <Ps3Controller.h>

// Receiver MAC Address
uint8_t broadcastAddress[] = {0x08, 0xD1, 0xF9, 0xE0, 0x6C, 0x24};

// Nested direction struct
struct Direction {
  int x;
  int y;
};

bool startToggle=false;
struct struct_message {
  Direction left;
  Direction right;
  bool startToggle;
};



struct_message myData;   // Global data to send

esp_now_peer_info_t peerInfo;

// Send data function
void sendData() {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  Serial.println(result == ESP_OK ? "Sent successfully" : "Error sending data");
}

// Callback: joystick moved
void notify() {
  bool dataToSend = false;

  if (abs(Ps3.event.analog_changed.stick.lx) > 2 || abs(Ps3.event.analog_changed.stick.ly) > 2) {
    myData.left.x = Ps3.data.analog.stick.lx;
    myData.left.y = Ps3.data.analog.stick.ly;
    dataToSend = true;
  }

  if (abs(Ps3.event.analog_changed.stick.rx) > 2 || abs(Ps3.event.analog_changed.stick.ry) > 2) {
    myData.right.x = Ps3.data.analog.stick.rx;
    myData.right.y = Ps3.data.analog.stick.ry;
    dataToSend = true;
  }

  if (Ps3.event.button_down.start) {
    startToggle = !startToggle;                      // Toggle
    myData.startToggle = startToggle;                // Update struct
    Serial.print("Start toggled to: ");
    Serial.println(startToggle ? "true" : "false");
    dataToSend = true;
  }

  if (dataToSend) {
    sendData();
  }
}

// ESP-NOW send callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nSend Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);

  Ps3.attach(notify);
  Ps3.begin("00:0b:e4:46:3a:29");

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("Ready.");
}

void loop() {
  // Loop does nothing; PS3 events handled via callback
}