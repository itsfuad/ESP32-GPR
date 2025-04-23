// ESP32 GPR Receiver (Connected to Laptop) with LED Indicator
// This code runs on the ESP32 connected to a laptop via USB

#include <WiFi.h>
#include <esp_wifi.h>
#include <driver/adc.h>

// WiFi credentials to connect to transmitter
const char* ssid = "ESP32_GPR_NETWORK";
const char* password = "gpr12345";

// LED pin for status indications
const int ledPin = 2;  // ESP32 onboard LED
unsigned long lastLedBlinkTime = 0;
bool ledState = false;
int ledBlinkMode = 0;  // 0=off, 1=slow blink (waiting), 2=fast blink (receiving)

// UDP setup
WiFiUDP udp;
unsigned int localPort = 2391;
unsigned int transmitterPort = 2390;

// GPR signal reception settings
const int receivePin = 34;       // GPIO pin with ADC for receiving
const int samplesPerScan = 256;  // Number of samples per scan

// Use separate buffer to ensure data isn't corrupted during sending
uint16_t scanData[samplesPerScan];
const int maxScansToBuffer = 10; // Maximum number of scans to buffer
uint16_t scanBuffer[maxScansToBuffer][samplesPerScan]; // Buffer for multiple scans
int currentScanIndex = 0;

// Timing control
unsigned long lastScanTime = 0;
const unsigned long scanTimeout = 5000; // ms
const int samplingDelay = 10; // microseconds between samples

// ADC calibration
int baselineValue = 0;
bool calibrated = false;

// Setup function
void setup() {
  Serial.begin(115200);
  delay(1000); // Give time for serial to initialize
  Serial.println("ESP32 GPR Receiver Starting");
  
  // Configure LED pin
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);  // Turn on LED during setup
  
  // Configure ADC for better performance
  analogReadResolution(12); // 12-bit resolution
  analogSetAttenuation(ADC_11db); // Full range
  analogSetPinAttenuation(receivePin, ADC_11db);
  
  // Connect to WiFi network created by transmitter
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  // Wait for connection with timeout
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && 
         millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
    digitalWrite(ledPin, !digitalRead(ledPin));  // Blink LED while connecting
  }
  
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Start UDP
    udp.begin(localPort);
    Serial.println("UDP started");
    
    // Optimize WiFi for lower latency
    esp_wifi_set_ps(WIFI_PS_NONE);
    
    // Connection success - blink LED 3 times
    for (int i = 0; i < 3; i++) {
      digitalWrite(ledPin, LOW);  // LED off
      delay(100);
      digitalWrite(ledPin, HIGH);  // LED on
      delay(100);
    }
    ledBlinkMode = 1;  // Set to slow blink mode (waiting for data)
  } else {
    Serial.println("");
    Serial.println("WiFi connection failed! Running in demo mode.");
    
    // Connection failed - blink LED 5 times quickly
    for (int i = 0; i < 5; i++) {
      digitalWrite(ledPin, LOW);  // LED off
      delay(50);
      digitalWrite(ledPin, HIGH);  // LED on
      delay(50);
    }
    ledBlinkMode = 1;  // Set to slow blink mode (demo mode)
  }
  digitalWrite(ledPin, LOW);  // Turn off LED

  // Initial calibration
  calibrateADC();
}

// Update LED status based on current mode
void updateLED() {
  unsigned long currentTime = millis();
  
  switch(ledBlinkMode) {
    case 0:  // LED off
      digitalWrite(ledPin, LOW);
      break;
      
    case 1:  // Slow blink (waiting for data/demo mode)
      if (currentTime - lastLedBlinkTime > 1000) {
        ledState = !ledState;
        digitalWrite(ledPin, ledState);
        lastLedBlinkTime = currentTime;
      }
      break;
      
    case 2:  // Fast blink (actively receiving)
      if (currentTime - lastLedBlinkTime > 100) {
        ledState = !ledState;
        digitalWrite(ledPin, ledState);
        lastLedBlinkTime = currentTime;
      }
      break;
      
    case 3:  // Solid on (sending data to laptop)
      digitalWrite(ledPin, HIGH);
      break;
  }
}

void calibrateADC() {
  Serial.println("Calibrating ADC...");
  long sum = 0;
  const int calibrationSamples = 100;
  
  // Take multiple readings to get baseline
  for(int i = 0; i < calibrationSamples; i++) {
    sum += analogRead(receivePin);
    delay(5);
  }
  
  baselineValue = sum / calibrationSamples;
  Serial.print("Baseline value: ");
  Serial.println(baselineValue);
  calibrated = true;
}

// Main loop
void loop() {
  bool scanTriggered = false;
  
  // Update LED based on current mode
  updateLED();
  
  // Check for sync packets from transmitter
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Received sync packet, record the scan
    uint8_t syncByte;
    udp.read(&syncByte, 1);
    if (syncByte == 0xFF) {
      ledBlinkMode = 2;  // Fast blink - receiving data
      recordScan();
      scanTriggered = true;
    }
  }
  
  // If WiFi connection failed or no sync packet received for a while, run in demo mode
  if (!scanTriggered && (WiFi.status() != WL_CONNECTED || millis() - lastScanTime > scanTimeout)) {
    ledBlinkMode = 1;  // Slow blink - demo mode
    generateDemoScan();
    scanTriggered = true;
  }
  
  // Check if we should send data to laptop
  if (currentScanIndex >= maxScansToBuffer || 
      (scanTriggered && currentScanIndex > 0)) {
    ledBlinkMode = 3;  // Solid on - sending data
    sendDataToLaptop();
    currentScanIndex = 0;
    ledBlinkMode = 1;  // Return to slow blink after sending
  }
  
  // Small delay to prevent overwhelming the CPU
  delay(10);
}

// Record one scan of actual data
void recordScan() {
  // Create some variation in each reading
  static int offsetVal = 0;
  offsetVal = (offsetVal + 7) % 100; // Change the offset slightly each scan
  
  for (int i = 0; i < samplesPerScan; i++) {
    // Read analog value from receive pin with noise reduction
    int rawValue = 0;
    // Take average of multiple readings for noise reduction
    const int numReadings = 3;
    for (int j = 0; j < numReadings; j++) {
      rawValue += analogRead(receivePin);
      delayMicroseconds(samplingDelay);
    }
    rawValue /= numReadings;
    
    // Store value with offset from baseline to show variation
    scanBuffer[currentScanIndex][i] = rawValue;
    
    // Small delay between samples
    delayMicroseconds(samplingDelay);
  }
  
  lastScanTime = millis();
  currentScanIndex++;
}

// Generate demo scan with simulated reflections when no real data is available
void generateDemoScan() {
  static int position = 0;
  position = (position + 1) % 50; // Move "position" to simulate movement
  
  // Create a scan with simulated objects
  for (int i = 0; i < samplesPerScan; i++) {
    // Base signal (decaying with depth)
    int value = 2048 - (i * 4);
    
    // Add simulated reflections that move with position
    if (i > 50 && i < 70) {
      value += 500 * exp(-0.1 * abs(i - 60 - (position % 10)));
    }
    
    // Add second deeper reflection
    if (i > 120 && i < 150) {
      value += 300 * exp(-0.1 * abs(i - 135 - (position % 15)));
    }
    
    // Add some random noise
    value += random(-100, 100);
    
    // Ensure value is within ADC range
    value = constrain(value, 0, 4095);
    
    scanBuffer[currentScanIndex][i] = value;
  }
  
  lastScanTime = millis();
  currentScanIndex++;
  
  // Simple indicator
  Serial.print("Demo scan generated (position: ");
  Serial.print(position);
  Serial.println(")");
}

// Send collected data to laptop via serial
void sendDataToLaptop() {
  digitalWrite(ledPin, HIGH);  // Solid ON while sending data
  
  Serial.println("BEGIN_GPR_DATA");
  Serial.print(currentScanIndex);
  Serial.print(",");
  Serial.println(samplesPerScan);
  
  // Send all buffered scans
  for (int scan = 0; scan < currentScanIndex; scan++) {
    for (int i = 0; i < samplesPerScan; i++) {
      Serial.print(scanBuffer[scan][i]);
      if (i < samplesPerScan - 1) {
        Serial.print(",");
      }
    }
    Serial.println();
  }
  
  Serial.println("END_GPR_DATA");
}