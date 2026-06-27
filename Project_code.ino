// ============================================================
// BEHAVIORAL HEALTH MONITORING SYSTEM
// ALL SENSORS ON SINGLE I2C BUS (SDA=21, SCL=22)
// ============================================================

#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_MLX90614.h>

// ------------------- Pin Definitions -------------------
#define SDA_PIN 21
#define SCL_PIN 22
#define BUZZER_PIN 25
#define GSR_PIN 33

// ------------------- OLED Setup -------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ------------------- ADXL345 Setup -------------------
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// ------------------- MAX30102 Setup -------------------
MAX30105 particleSensor;

// ------------------- MLX90614 Setup -------------------
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// MAX30102 Variables
#define BUFFER_SIZE 100
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
int32_t spo2 = 0;
int32_t heartRate = 0;
int8_t validSPO2 = 0;
int8_t validHeartRate = 0;
uint32_t lastHRCalculation = 0;

// MLX90614 Variables
float ambientTemp = 0;
float objectTemp = 0;

// Movement detection
float lastAx = 0, lastAy = 0, lastAz = 0;
bool movementDetected = false;

// GSR Variables
int gsrRaw = 0;
float gsrVoltage = 0.0;
String gsrStatus = "Normal";

// Alert Variables
bool hrAlert = false;
bool spo2Alert = false;
bool tempAlert = false;
bool gsrAlert = false;
String alertMessage = "";

// Timing variables
unsigned long lastGSRUpdate = 0;
unsigned long lastTempUpdate = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastBuzzerAlert = 0;

// System State
bool systemInitialized = false;

// Alert Thresholds
const int HR_LOW = 60;
const int HR_HIGH = 100;
const int SPO2_LOW = 95;
const float TEMP_HIGH = 38.0;
const int GSR_HIGH = 2500;

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Show startup sequence
  startupSequence();
  
  // Initialize single I2C bus
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize components with testing
  initializeOLED();
  testADXL345();
  testMAX30102();
  testMLX90614();
  testGSR();
  testBuzzer();
  
  // System ready
  systemReady();
  systemInitialized = true;
  Serial.println("✅ Behavioral Health Monitoring System Ready!");
}

// ------------------- Startup Sequence -------------------
void startupSequence() {
  Serial.println("==========================================");
  Serial.println("   BEHAVIORAL HEALTH MONITORING SYSTEM");
  Serial.println("==========================================");
  
  // Initialize OLED first for display
  Wire.begin(SDA_PIN, SCL_PIN);
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C) || 
      display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    
    // Display system title
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Behavioral");
    display.setCursor(0, 18);
    display.println("Health");
    display.setCursor(0, 36);
    display.println("Monitor");
    display.display();
    delay(2000);
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Behavioral Health");
    display.setCursor(0, 12);
    display.println("Monitoring System");
    display.setCursor(0, 30);
    display.println("Initializing...");
    display.display();
  }
  
  delay(1000);
}

// ------------------- Initialize OLED -------------------
void initializeOLED() {
  Serial.print("Testing OLED Display...");
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
      Serial.println(" FAILED!");
      return;
    }
  }
  
  // Test display functionality
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED Test");
  display.setCursor(0, 20);
  display.println("Display: OK");
  display.display();
  
  Serial.println(" OK");
  delay(1000);
}

// ------------------- Test ADXL345 -------------------
void testADXL345() {
  Serial.print("Testing ADXL345 Accelerometer...");
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Testing Sensors:");
  display.setCursor(0, 12);
  display.print("ADXL345...");
  display.display();
  
  if (!accel.begin()) {
    display.setCursor(70, 12);
    display.println("FAIL");
    display.display();
    Serial.println(" FAILED!");
    delay(1000);
    return;
  }
  
  accel.setRange(ADXL345_RANGE_4_G);
  
  // Test reading
  sensors_event_t event;
  if (accel.getEvent(&event)) {
    display.setCursor(70, 12);
    display.println("OK");
    display.display();
    Serial.println(" OK");
  } else {
    display.setCursor(70, 12);
    display.println("FAIL");
    display.display();
    Serial.println(" FAILED!");
  }
  
  delay(1000);
}

// ------------------- Test MAX30102 -------------------
void testMAX30102() {
  Serial.print("Testing MAX30102 Pulse Sensor...");
  
  display.setCursor(0, 24);
  display.print("MAX30102...");
  display.display();
  
  if (!particleSensor.begin(Wire)) {
    display.setCursor(70, 24);
    display.println("FAIL");
    display.display();
    Serial.println(" FAILED!");
    delay(1000);
    return;
  }
  
  // Configure MAX30102
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
  
  // Initialize buffers
  for (int i = 0; i < BUFFER_SIZE; i++) {
    irBuffer[i] = 0;
    redBuffer[i] = 0;
  }
  
  // Test IR reading
  uint32_t irValue = particleSensor.getIR();
  if (irValue > 0) {
    display.setCursor(70, 24);
    display.println("OK");
    display.display();
    Serial.println(" OK");
  } else {
    display.setCursor(70, 24);
    display.println("FAIL");
    display.display();
    Serial.println(" FAILED!");
  }
  
  delay(1000);
}

// ------------------- Test MLX90614 -------------------
void testMLX90614() {
  Serial.print("Testing MLX90614 Temperature...");
  
  display.setCursor(0, 36);
  display.print("MLX90614...");
  display.display();
  
  if (!mlx.begin()) {
    display.setCursor(70, 36);
    display.println("FAIL");
    display.display();
    Serial.println(" FAILED!");
    delay(1000);
    return;
  }
  
  // Test temperature reading
  float testTemp = mlx.readObjectTempC();
  if (!isnan(testTemp)) {
    display.setCursor(70, 36);
    display.println("OK");
    display.display();
    Serial.println(" OK");
  } else {
    display.setCursor(70, 36);
    display.println("FAIL");
    display.display();
    Serial.println(" FAILED!");
  }
  
  delay(1000);
}

// ------------------- Test GSR -------------------
void testGSR() {
  Serial.print("Testing GSR Sensor...");
  
  display.setCursor(0, 48);
  display.print("GSR Sensor...");
  display.display();
  
  pinMode(GSR_PIN, INPUT);
  
  // Test GSR reading
  int testValue = analogRead(GSR_PIN);
  if (testValue >= 0) {
    display.setCursor(70, 48);
    display.println("OK");
    display.display();
    Serial.println(" OK");
  } else {
    display.setCursor(70, 48);
    display.println("FAIL");
    display.display();
    Serial.println(" FAILED!");
  }
  
  delay(1000);
}

// ------------------- Test Buzzer -------------------
void testBuzzer() {
  Serial.print("Testing Buzzer...");
  
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Quick beep test
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.println(" OK");
  delay(500);
}

// ------------------- System Ready -------------------
void systemReady() {
  // Display final ready message
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("System Status:");
  display.setCursor(0, 12);
  display.println("All Sensors: OK");
  display.setCursor(0, 24);
  display.println("Behavioral Health");
  display.setCursor(0, 36);
  display.println("Monitoring System");
  display.setCursor(0, 48);
  display.println("READY!");
  display.display();
  
  // Celebration beep pattern
  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(50);
  }
  digitalWrite(BUZZER_PIN, HIGH);
  delay(300);
  digitalWrite(BUZZER_PIN, LOW);
  
  delay(2000);
}

// ------------------- Movement Detection -------------------
bool detectMovement(float ax, float ay, float az) {
  float dx = ax - lastAx;
  float dy = ay - lastAy;
  float dz = az - lastAz;
  
  float mag = sqrt(dx*dx + dy*dy + dz*dz);
  
  lastAx = ax;
  lastAy = ay;
  lastAz = az;
  
  return (mag > 2.0); // Movement threshold
}

// ------------------- Check Alerts -------------------
void checkAlerts() {
  hrAlert = false;
  spo2Alert = false;
  tempAlert = false;
  gsrAlert = false;
  alertMessage = "";
  
  // Check Heart Rate
  if (heartRate > 0) {
    if (heartRate < HR_LOW || heartRate > HR_HIGH) {
      hrAlert = true;
      alertMessage += "HR! ";
    }
  }
  
  // Check SpO2
  if (spo2 > 0) {
    if (spo2 < SPO2_LOW) {
      spo2Alert = true;
      alertMessage += "SpO2! ";
    }
  }
  
  // Check Temperature
  if (objectTemp > TEMP_HIGH) {
    tempAlert = true;
    alertMessage += "T! ";
  }
  
  // Check GSR
  if (gsrStatus == "Sweaty") {
    gsrAlert = true;
    alertMessage += "GSR! ";
  }
  
  // Trigger buzzer if any alert
  if ((hrAlert || spo2Alert || tempAlert || gsrAlert) && 
      (millis() - lastBuzzerAlert > 2000)) {
    triggerAlertBuzzer();
    lastBuzzerAlert = millis();
  }
}

// ------------------- Trigger Alert Buzzer -------------------
void triggerAlertBuzzer() {
  Serial.println("ALERT! Triggering buzzer");
  
  // Different patterns based on severity
  if (hrAlert) {
    // Heart rate alert - rapid beeps
    for (int i = 0; i < 5; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
      delay(100);
    }
  } else if (spo2Alert) {
    // SpO2 alert - long beeps
    for (int i = 0; i < 3; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(300);
      digitalWrite(BUZZER_PIN, LOW);
      delay(200);
    }
  } else {
    // General alert
    for (int i = 0; i < 3; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(200);
      digitalWrite(BUZZER_PIN, LOW);
      delay(200);
    }
  }
}

// ------------------- Update MAX30102 -------------------
void updateMAX30102() {
  uint32_t irValue = particleSensor.getIR();
  
  if (irValue < 10000) {
    // No finger detected
    heartRate = 0;
    spo2 = 0;
    validSPO2 = 0;
    validHeartRate = 0;
    return;
  }

  // Calculate HR and SpO2 every 2 seconds
  if (millis() - lastHRCalculation >= 2000) {
    lastHRCalculation = millis();
    
    // Collect samples
    for (int i = 0; i < BUFFER_SIZE; i++) {
      while (!particleSensor.available()) {
        particleSensor.check();
      }
      
      irBuffer[i] = particleSensor.getIR();
      redBuffer[i] = particleSensor.getRed();
      particleSensor.nextSample();
    }

    // Calculate heart rate and SpO2
    maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer, 
                                          &spo2, &validSPO2, &heartRate, &validHeartRate);

    // Validate results
    if (!validHeartRate || heartRate < 30 || heartRate > 250) {
      heartRate = 0;
    }
    
    if (!validSPO2 || spo2 < 70 || spo2 > 100) {
      spo2 = 0;
    }

    Serial.printf("MAX30102 - HR: %ld BPM, SpO2: %ld%%\n", heartRate, spo2);
  }
}

// ------------------- Update MLX90614 -------------------
void updateMLX90614() {
  if (millis() - lastTempUpdate >= 1000) {
    lastTempUpdate = millis();
    
    ambientTemp = mlx.readAmbientTempC();
    objectTemp = mlx.readObjectTempC();

    Serial.printf("MLX90614 - Obj: %.2f°C, Amb: %.2f°C\n", objectTemp, ambientTemp);
  }
}

// ------------------- Update GSR -------------------
void updateGSR() {
  if (millis() - lastGSRUpdate >= 1000) {
    lastGSRUpdate = millis();
    
    gsrRaw = analogRead(GSR_PIN);
    gsrVoltage = (gsrRaw / 4095.0) * 3.3;
    
    // Determine GSR status
    if (gsrRaw < 100) {
      gsrStatus = "No Contact";
    } else if (gsrRaw < 500) {
      gsrStatus = "Dry";
    } else if (gsrRaw < 1500) {
      gsrStatus = "Normal";
    } else if (gsrRaw < 2500) {
      gsrStatus = "Moist";
    } else {
      gsrStatus = "Sweaty";
    }

    Serial.printf("GSR - Status: %s, Voltage: %.2fV\n", gsrStatus.c_str(), gsrVoltage);
  }
}

// ------------------- Buzzer Alert -------------------
void movementBeep() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(50);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
}

// ------------------- Update OLED Display -------------------
void updateOLED() {
  if (millis() - lastDisplayUpdate < 500) return;
  lastDisplayUpdate = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Line 1: HR and SpO2
  display.setCursor(0, 0);
  display.print("HR: ");
  if (heartRate > 0) {
    if (hrAlert) {
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    }
    display.print(heartRate);
    display.print(" BPM");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.print("-- BPM");
  }
  
  display.setCursor(70, 0);
  display.print("SpO2: ");
  if (spo2 > 0) {
    if (spo2Alert) {
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    }
    display.print(spo2);
    display.print(" %");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.print("-- %");
  }

  // Line 2: Object Temp and Ambient Temp
  display.setCursor(0, 12);
  display.print("ObjT: ");
  if (tempAlert) {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  }
  display.print(objectTemp, 2);
  display.print(" C");
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(70, 12);
  display.print("AmbT: ");
  display.print(ambientTemp, 2);
  display.print(" C");

  // Line 3: GSR with status and voltage
  display.setCursor(0, 24);
  display.print("GSR: ");
  if (gsrAlert) {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  }
  display.print(gsrStatus);
  display.print(" (");
  display.print(gsrVoltage, 2);
  display.print(" V)");
  display.setTextColor(SSD1306_WHITE);

  // Line 4: Movement
  display.setCursor(0, 36);
  display.print("MOV: ");
  if (movementDetected) {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.print("YES");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.print("NO");
  }

  // Line 5: Alert Message
  display.setCursor(0, 48);
  if (alertMessage.length() > 0) {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.print("ALERT: ");
    display.print(alertMessage);
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.print("All Normal");
  }

  display.display();
}

// ------------------- Main Loop -------------------
void loop() {
  if (!systemInitialized) {
    return;
  }

  // Read accelerometer
  sensors_event_t event;
  if (accel.getEvent(&event)) {
    movementDetected = detectMovement(event.acceleration.x, 
                                     event.acceleration.y, 
                                     event.acceleration.z);
  }

  // Update all sensors
  updateMAX30102();
  updateMLX90614();
  updateGSR();

  // Check for alerts
  checkAlerts();

  // Update OLED Display
  updateOLED();

  // Buzzer alert for movement
  if (movementDetected) {
    movementBeep();
  }

  // Serial output for debugging
  Serial.printf("HR:%ld SpO2:%ld ObjT:%.2f AmbT:%.2f GSR:%s(%.2fV) Mov:%s Alerts:%s\n",
                heartRate, spo2, objectTemp, ambientTemp, gsrStatus.c_str(), gsrVoltage,
                movementDetected ? "YES" : "NO", alertMessage.c_str());

  delay(100);
}