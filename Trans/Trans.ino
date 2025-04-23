// ESP32 GPR Transmitter (Mobile Unit) with LED Indicator
// This code runs on the battery-powered ESP32

#include <WiFi.h>
#include <esp_wifi.h>

// WiFi network credentials for direct communication
const char* ssid = "ESP32_GPR_NETWORK";
const char* password = "gpr12345";

// LED pin for status indications
const int ledPin = 2;  // ESP32 onboard LED

// UDP setup for communication
WiFiUDP udp;
IPAddress receiverIP(192, 168, 4, 2); // Fixed IP for the receiver
unsigned int localPort = 2390;  
unsigned int receiverPort = 2391;

// GPR pulse generation settings
const int pulsePin = 25;         // GPIO pin used for signal output
const int pulseDuration = 1;     // Duration of pulse in microseconds
const int pulsePeriod = 1000;    // Time between pulses in microseconds
const int samplesPerScan = 256;  // Number of samples per scan

// LED blink patterns
unsigned long lastLedToggle = 0;
bool ledState = false;

// Setup function
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 GPR Transmitter Starting");
  
  // Configure the LED pin
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);  // Turn on LED during setup
  
  // Configure the pulse pin as output
  pinMode(pulsePin, OUTPUT);
  digitalWrite(pulsePin, LOW);
  
  // Create WiFi network
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  
  // Start UDP
  udp.begin(localPort);
  Serial.println("UDP server started");
  
  // Optimize WiFi for lower latency
  esp_wifi_set_ps(WIFI_PS_NONE);
  
  // Setup complete, blink LED 3 times
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, LOW);  // LED off
    delay(100);
    digitalWrite(ledPin, HIGH);  // LED on
    delay(100);
  }
  digitalWrite(ledPin, LOW);  // LED off after setup
}

// Main loop
void loop() {
  // Send sync signal to receiver
  sendSyncPacket();
  
  // Quick blink to indicate sync packet sent
  digitalWrite(ledPin, HIGH);
  
  // Generate a series of pulses for one complete scan
  for (int i = 0; i < samplesPerScan; i++) {
    // Generate pulse
    digitalWrite(pulsePin, HIGH);
    delayMicroseconds(pulseDuration);
    digitalWrite(pulsePin, LOW);
    
    // Wait for the specified period before next pulse
    delayMicroseconds(pulsePeriod - pulseDuration);
  }
  
  // Turn LED off after scan completed
  digitalWrite(ledPin, LOW);
  
  // Wait between scans
  delay(100);
}

// Send synchronization packet to receiver
void sendSyncPacket() {
  udp.beginPacket(receiverIP, receiverPort);
  uint8_t syncByte = 0xFF;
  udp.write(&syncByte, 1);
  udp.endPacket();
}