//This code connects the ESP to Jaime's app and displays our metrics




#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Create the BNO055 object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // Sensor ID and I2C address

// Force Sensor Pin
const int FORCE_SENSOR_PIN = 5;

// Sampling interval in milliseconds
#define SAMPLING_INTERVAL_MS 50 // 50ms for ~20Hz sampling rate

// Force Detection Threshold (Adjust based on calibration)
#define FORCE_THRESHOLD 0.05 // Newtons

// Debounce Delay to prevent multiple counts for a single punch (in ms)
#define DEBOUNCE_DELAY_MS 500

unsigned long lastSampleTime = 0;

// Punch Detection Variables
bool punchDetected = false;
unsigned long punchStartTime = 0;
float forceSum = 0.0;
int forceSamples = 0;
int punchCount = 0;
unsigned long lastPunchTime = 0;

void setup() {
  // Initialize Serial communication at 115200 baud
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for Serial Monitor to open
  }

  Serial.println("Initializing BNO055 and Force Sensor...");

  // Initialize digital pins as per your working setup
  // Set pin 1 as output and set it LOW
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);

  // Set pin 3 as output and set it HIGH
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);

  // Set pin 11 as output and set it LOW
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);

  // Initialize ADC resolution
  analogReadResolution(12); // 12-bit ADC resolution (0-4095)

  // Short delay for stabilization
  delay(100);

  // Initialize I2C with SDA on pin 18 and SCL on pin 17
  Wire.begin(18, 17); // SDA on GPIO18, SCL on GPIO17

  // Initialize the BNO055 sensor
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check your wiring or I2C address.");
    while (1); // Halt execution if sensor not found
  }

  Serial.println("BNO055 detected and initialized!");

  // Use external crystal for better accuracy
  

  // Brief delay to allow sensor to stabilize
  delay(1000);

  // Print initial message
  Serial.println("Punch Metrics Display Initialized.");
}

void loop() {
  unsigned long currentTime = millis();

  // Check if it's time to take a new sample
  if (currentTime - lastSampleTime >= SAMPLING_INTERVAL_MS) {
    lastSampleTime = currentTime;

    // Read force sensor value
    int adcValue = analogRead(FORCE_SENSOR_PIN);
    float voltage = (adcValue / 4095.0) * 3.3; // Convert ADC value to voltage (assuming 3.3V reference)
    float force = calculateForce(voltage); // Convert voltage to force

    // Punch Detection Logic
    if (force >= FORCE_THRESHOLD && !punchDetected && (currentTime - lastPunchTime > DEBOUNCE_DELAY_MS)) {
      // Start of a new punch
      punchDetected = true;
      punchStartTime = millis();

      // Reset accumulation variables
      forceSum = 0.0;
      forceSamples = 0;

      // Increment punch count
      punchCount++;

      // Optionally, print "Punch detected!" message
      Serial.println("Punch detected!");
    }

    if (force >= FORCE_THRESHOLD && punchDetected) {
      // Accumulate force data
      forceSum += force;
      forceSamples++;
    }

    if (force < FORCE_THRESHOLD && punchDetected) {
      // End of the current punch
      punchDetected = false;

      // Calculate average force
      float avgForce = (forceSamples > 0) ? (forceSum / forceSamples) : 0.0;

      // Fetch acceleration data in m/s²
      imu::Vector<3> accelData = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      float accelX = accelData.x();
      float accelY = accelData.y();
      float accelZ = accelData.z();

      // Get gyroscope data in rad/s
      imu::Vector<3> gyroData = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      float gyroX = gyroData.x();
      float gyroY = gyroData.y();
      float gyroZ = gyroData.z();

      // Create a data package string with labels
      String dataPackage = "PunchCount: " + String(punchCount) + ", Force: " + String(avgForce, 2) + " N, " +
                           "AccelX: " + String(accelX, 2) + " m/s², " +
                           "AccelY: " + String(accelY, 2) + " m/s², " +
                           "AccelZ: " + String(accelZ, 2) + " m/s², " +
                           "GyroX: " + String(gyroX, 2) + " rad/s, " +
                           "GyroY: " + String(gyroY, 2) + " rad/s, " +
                           "GyroZ: " + String(gyroZ, 2) + " rad/s";

      // Send data to Serial Monitor
      Serial.println(dataPackage);

      // Update the last punch time for debounce
      lastPunchTime = currentTime;

      // Optional: Add a small delay to prevent immediate re-detection
      delay(200);
    }
  }

  // Small delay to prevent overwhelming the loop (optional)
  delay(1);
}

// Function to calculate force based on voltage
float calculateForce(float voltage) {
  // Replace this with your calibration formula
  // Example formula:
  // force (N) = (voltage - V_offset) * scaling_factor
  float V_offset = 0.5;                    // Voltage offset corresponding to 0 N (adjust based on calibration)
  float scaling_factor = 1000.0 / 3.0;     // Scaling factor to convert voltage to force (adjust based on calibration)

  float force = (voltage - V_offset) * scaling_factor;

  // Ensure that force is not negative
  if (force < 0) {
    force = 0;
  }

  return force;
}


