#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_VL53L0X.h"

// Pin Definitions
#define TRIG_PIN 2
#define ECHO_PIN 3
#define RADAR_PIN 4
#define VIBRATION_LEFT_PIN 8  
#define VIBRATION_RIGHT_PIN 9  
#define BUZZER_PIN 7
#define WATER_SENSOR A0

// Sensor Objects
Adafruit_MPU6050 mpu;
Adafruit_VL53L0X lidar;

// Radar Settings (Fast Object Detection)
const int RADAR_COOLDOWN = 10000;             // 10-second cooldown
const int RADAR_MIN_DETECTION_TIME = 2000;    // 5s minimum (fast vehicles)
const int RADAR_MAX_DETECTION_TIME = 5000;    // 7s maximum (slow vehicles)
const int WALKING_BLOCK_TIME = 1000;          // 3s blocking after walking
unsigned long lastRadarTime = 0;
unsigned long lastWalkingTime = 0;

// Walking Detection
const float WALKING_THRESHOLD = 2.8;
bool isWalking = false;

// Stair Detection (70-150mm range)
const int STAIR_MIN_HEIGHT = 200;
const int STAIR_MAX_HEIGHT = 300;
unsigned long lastStairTime = 0;

// Serial Output Control
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 400;

// Vibration Constants
const int SHORT_VIBE = 150;
const int MED_VIBE = 300;
const int LONG_VIBE = 500;
const int DANGER_VIBE = 1000;
const int TILT_VIBE = 350;
const int STAIR_VIBE = 450;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println(F("\n=== Smart Navigation Shoe ==="));
  Serial.println(F("Initializing sensors..."));

  // Initialize I/O
  pinMode(VIBRATION_LEFT_PIN, OUTPUT);
  pinMode(VIBRATION_RIGHT_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RADAR_PIN, INPUT);

  // Initialize I2C Sensors
  Wire.begin();
  if (!mpu.begin() || !lidar.begin()) {
    Serial.println(F("ERROR: Sensor initialization failed!"));
    while(1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.println(F("System Ready"));
  Serial.println(F("----------------------------------------"));
  Serial.println(F("| Time (ms) | Hazard       | Distance   |"));
  Serial.println(F("----------------------------------------"));
}

void loop() {
  updateWalkingState();

  if (!isWalking) {
    if (checkWater()) return;
    if (checkPit()) return;
    checkStairs();
    checkWalls();
    checkRadar();
  } else {
    lastWalkingTime = millis();
  }
  checkTilt();
  
  printSensorData();
  delay(50);
}

// ==================== IMPROVED RADAR ====================
void checkRadar() {
  static bool radarActive = false;
  static unsigned long detectionStart = 0;

  // Skip if in cooldown or recently walking
  if (millis() - lastRadarTime < RADAR_COOLDOWN || 
      millis() - lastWalkingTime < WALKING_BLOCK_TIME) {
    radarActive = false;
    return;
  }

  bool currentDetection = digitalRead(RADAR_PIN);

  // Detection logic
  if (currentDetection) {
    if (!radarActive) {
      radarActive = true;
      detectionStart = millis();
    }
    
    // Check if detection duration is in vehicle range (5-7s)
    unsigned long detectionDuration = millis() - detectionStart;
    if (detectionDuration >= RADAR_MIN_DETECTION_TIME && 
        detectionDuration <= RADAR_MAX_DETECTION_TIME) {
      printDetection("VEHICLE", 0);
      
      // Emergency alert pattern
      for (int i = 0; i < 3; i++) {
        tone(BUZZER_PIN, 1800, 200);
        safeVibrate(300, 300, 1);
        delay(400);
        noTone(BUZZER_PIN);
        delay(300);
      }
      
      lastRadarTime = millis();
      radarActive = false;
    }
  } else {
    radarActive = false;
  }
}

// ==================== SENSOR FUNCTIONS ====================
void updateWalkingState() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  isWalking = (abs(a.acceleration.y) > WALKING_THRESHOLD);
}

bool checkWater() {
  int waterLevel = analogRead(WATER_SENSOR);
  if (waterLevel > 200) {
    printDetection("WATER", 0);
    triggerAlert(3, LONG_VIBE, true);
    return true;
  }
  return false;
}

bool checkPit() {
  VL53L0X_RangingMeasurementData_t measure;
  lidar.rangingTest(&measure, false);
  if (measure.RangeStatus != 4 && measure.RangeMilliMeter > 300) {
    printDetection("DEEP PIT", measure.RangeMilliMeter);
    triggerAlert(2, DANGER_VIBE, true);
    return true;
  }
  return false;
}

void checkStairs() {
  if (millis() - lastStairTime < 2000) return;

  VL53L0X_RangingMeasurementData_t measure;
  lidar.rangingTest(&measure, false);
  if (measure.RangeStatus == 4) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (measure.RangeMilliMeter >= STAIR_MIN_HEIGHT && 
      measure.RangeMilliMeter <= STAIR_MAX_HEIGHT &&
      a.acceleration.z > 7.0 && 
      abs(a.acceleration.y) < 2.0) {
    printDetection("STAIR", measure.RangeMilliMeter);
    safeVibrate(STAIR_VIBE, STAIR_VIBE, 1);
    lastStairTime = millis();
  }
}

void checkWalls() {
  int dist = getUltrasonicDistance();
  if (dist > 40 && dist <= 100) {
    printDetection("WALL", dist);
    int vibeTime = map(dist, 20, 80, LONG_VIBE, SHORT_VIBE);
    safeVibrate(vibeTime, vibeTime, 1);
  }
}

void checkTilt() {
  if (isWalking) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  if (a.acceleration.x > 3.0) {
    printDetection("TILT RIGHT", a.acceleration.x * 10);
    safeVibrate(TILT_VIBE, 0, 1);
  } 
  else if (a.acceleration.x < -3.0) {
    printDetection("TILT LEFT", a.acceleration.x * 10);
    safeVibrate(0, TILT_VIBE, 1);
  }
}

// ==================== UTILITY FUNCTIONS ====================
void printDetection(const char* hazard, int value) {
  Serial.print("| ");
  Serial.print(millis());
  Serial.print(" | ");
  Serial.print(hazard);
  Serial.print("\t| ");
  if (value != 0) Serial.print(value);
  Serial.println("\t|");
}

void printSensorData() {
  if (millis() - lastPrintTime >= PRINT_INTERVAL) {
    Serial.print("| ");
    Serial.print(millis());
    Serial.print(" | STATUS\t| US: ");
    Serial.print(getUltrasonicDistance());
    Serial.print("cm LiDAR: ");
    
    VL53L0X_RangingMeasurementData_t measure;
    lidar.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
      Serial.print(measure.RangeMilliMeter);
      Serial.print("mm");
    } else {
      Serial.print("N/A");
    }
    
    Serial.print(" Walk: ");
    Serial.print(isWalking ? "Yes" : "No");
    Serial.println("\t|");
    
    lastPrintTime = millis();
  }
}

int getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2;
}

void triggerAlert(int bursts, int duration, bool useBuzzer) {
  for (int i = 0; i < bursts; i++) {
    if (useBuzzer) tone(BUZZER_PIN, 1500, duration);
    safeVibrate(duration, duration, 1);
    delay(duration + 100);
    if (useBuzzer) noTone(BUZZER_PIN);
  }
}

void vibratePattern(int pulses, int duration) {
  safeVibrate(duration, duration, pulses);
}

void safeVibrate(int leftDur, int rightDur, int count) {
  static unsigned long lastVib = 0;
  if (millis() - lastVib < 300) return;
  
  for (int i = 0; i < count; i++) {
    if (leftDur > 0) {
      digitalWrite(VIBRATION_LEFT_PIN, HIGH);
      delay(leftDur);
      digitalWrite(VIBRATION_LEFT_PIN, LOW);
    }
    if (rightDur > 0) {
      digitalWrite(VIBRATION_RIGHT_PIN, HIGH);
      delay(rightDur);
      digitalWrite(VIBRATION_RIGHT_PIN, LOW);
    }
    if (i < count - 1) delay(100);
  }
  lastVib = millis();
}