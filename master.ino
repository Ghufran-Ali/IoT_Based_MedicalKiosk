#include <WiFi.h>          
#include <Firebase_ESP_Client.h>
#include "HX711.h"
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_MLX90614.h>

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
// #define WIFI_SSID "PTCL-Flash Fiber"
// #define WIFI_PASSWORD "Pakistan@9214#"
#define WIFI_SSID "EIE"
#define WIFI_PASSWORD "EIE@4321"

// Insert Firebase project API Key
#define API_KEY "AIzaSyBmYCvYq3opk-dHL5iAlRGkm3VdFynDEMs"

// Insert RTDB URL
#define DATABASE_URL "https://iot-based-mk-default-rtdb.asia-southeast1.firebasedatabase.app/" 

// Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
bool signupOK = false; // since we are doing an anonymous sign in

MAX30105 particleSensor;

// Variables to hold sensor readings
uint32_t irBuffer[100]; // Infrared LED sensor data
uint32_t redBuffer[100]; // Red LED sensor data

// Variables for the algorithm
int32_t bufferLength; // Number of samples in the buffer
int32_t spo2; // Calculated SpO2 value
int8_t validSPO2; // Indicator for valid SpO2
int32_t heartRate; // Calculated heart rate value
int8_t validHeartRate; // Indicator for valid heart rate

// Diastolic Blood Pressure Estimation Variables
int diastolicPressure = 80; // Default diastolic value for estimation

// Create sensor object
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

#define LOADCELL_DOUT_PIN 32  // DT pin
#define LOADCELL_SCK_PIN 25   // SCK pin

#define TRIG_PIN 12
#define ECHO_PIN 14

const int sensorHeight = 199.39;  // Fixed height of the ultrasonic sensor above the ground in cm

HX711 scale;



void setup() {
  Serial.begin(115200);       // Serial monitor
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // UART on TX2 (16) and RX2 (17)

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.tare();  // Tare the scale

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize MLX90614 sensor
    Wire.begin(); // Default SDA (GPIO 21), SCL (GPIO 22) for ESP32
    Wire.setClock(100000);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1);
  }

  // Initialize sensor settings
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); // Turn off Green LED

  // Initial buffer length
  bufferLength = 100;

  // Read first 100 samples, and determine the signal range
  for (int i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false) { // Check if new data is available
      particleSensor.check(); // Check the sensor for new data
    }

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // Move to next sample
  }

  // Calculate SpO2 and heart rate
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX90614 sensor. Please check wiring.");
    while (1);  // Stop execution
  }
  Serial.println("MLX90614 sensor initialized successfully!");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Assign the api key (required)
  config.api_key = API_KEY;

  // Assign the RTDB URL (required)
  config.database_url = DATABASE_URL;

  // Sign up
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  // Assign the callback function for the long running token generation task
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  long duration, distance, objectHeight;
  int heightInFeet = 0;      // Declare variables outside the block
  float heightInInches = 0;
  float weight = 0;
  float bmi = 0;

  // Trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000);  // Timeout in 30 ms
  if (duration > 0) {
    distance = duration * 0.034 / 2;
    objectHeight = sensorHeight - distance;

    heightInFeet = objectHeight / 30.48;
    float totalInches = objectHeight / 2.54;
    heightInInches = totalInches - (heightInFeet * 12);

    if (scale.is_ready()) {
      long reading = scale.read();
      float calibration_factor = 12261.27;  // Adjust as per your setup
      weight = abs(1.121 * (reading / calibration_factor) - 9.9);

      float heightInMeters = objectHeight / 100.0;
      bmi = weight / (heightInMeters * heightInMeters);

      // Send the data to the slave via UART
      String data = "Height: " + String(heightInFeet) + " ft " + String(heightInInches, 1) + " in\n"
                    "Weight: " + String(weight, 2) + " kg\n"
                    "BMI: " + String(bmi, 2) + "\n";
      Serial2.print(data);  // Send data to slave

      // Debugging on Serial Monitor
      Serial.println(data);
    } else {
      Serial.println("HX711 not ready. Check wiring.");
    }
  } else {
    Serial.println("No object detected or out of range.");
  }

float bodyTemp = 0.0;
  // Read body temperature in Celsius
  float objectTempC = mlx.readObjectTempC();

  // Check for valid reading
  if (isnan(objectTempC)) {
    Serial.println("Error reading temperature.");
  } else {
    // Convert temperature to Fahrenheit
    bodyTemp = objectTempC * 9 / 5 + 32;

    // Debugging
    Serial.print("Body Temperature: ");
    Serial.print(bodyTemp);
    Serial.println(" F");

    // Send temperature data to the slave
    String dataToSend = "Body_Temp:" + String(bodyTemp) + "F;";
    Serial2.println(dataToSend);  // Send via UART
  }
  
  // Shift data in the buffer to make room for new samples
  for (int i = 25; i < 100; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  // Read 25 new samples
  for (int i = 75; i < 100; i++) {
    while (particleSensor.available() == false) {
      particleSensor.check();
    }

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // Check if a valid finger is placed (based on IR signal)
  bool validFinger = isValidFingerPresent(irBuffer);

  if (validFinger) {
    // Calculate SpO2 and heart rate only if a valid finger is detected
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    // Detect Diastolic Blood Pressure (DBP)
    estimateDiastolicBloodPressure(irBuffer, bufferLength);

    // Prepare the message to send
    String message = "Heart Rate: ";
    message += (validHeartRate) ? String(heartRate) + " bpm" : "Invalid bpm";
    message += ", SpO2: ";
    message += (validSPO2) ? String(spo2) + " %" : "Invalid %";
    message += ", DBP: " + String(diastolicPressure);

    // Print and send the message
    Serial.println(message); // Debugging
    Serial2.println(message); // Send data to slave ESP32 via UART
  } else {
    // Send "Invalid" for all readings if no valid finger is detected
    String message = "Heart Rate: Invalid bpm, SpO2: Invalid %, DBP: Invalid";
    Serial.println(message); // Debugging
    Serial2.println(message); // Send data to slave ESP32
  }
  if (Serial2.available() > 0) {
    String receivedData = Serial2.readStringUntil(';'); // Use delimiter to ensure full message
    if (receivedData.length() > 0) {
      // Parsing logic here
    }
}

  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();

    // Send data to Firebase
    if (Firebase.RTDB.setInt(&fbdo, "Sensors/HeightInFeet", heightInFeet)) {
      Serial.println("Height (Feet) upload PASSED");
    } else {
      Serial.println("Height (Feet) upload FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setFloat(&fbdo, "Sensors/HeightInInches", heightInInches)) {
      Serial.println("Height (Inches) upload PASSED");
    } else {
      Serial.println("Height (Inches) upload FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setFloat(&fbdo, "Sensors/Weight", weight)) {
      Serial.println("Weight upload PASSED");
    } else {
      Serial.println("Weight upload FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setFloat(&fbdo, "Sensors/BMI", bmi)) {
      Serial.println("BMI upload PASSED");
    } else {
      Serial.println("BMI upload FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (validHeartRate) {
  if (Firebase.RTDB.setInt(&fbdo, "Sensors/Heart_Rate", heartRate)) {
    Serial.println("Heart Rate upload PASSED");
  } else {
    Serial.println("Heart Rate upload FAILED: " + fbdo.errorReason());
  }
} else {
  Serial.println("Heart Rate is invalid, skipping upload.");
}

if (validSPO2) {
  if (Firebase.RTDB.setInt(&fbdo, "Sensors/SpO2", spo2)) {
    Serial.println("SpO2 upload PASSED");
  } else {
    Serial.println("SpO2 upload FAILED: " + fbdo.errorReason());
  }
} else {
  Serial.println("SpO2 is invalid, skipping upload.");
}

// Diastolic Pressure is always calculated, no validity check needed
if (Firebase.RTDB.setInt(&fbdo, "Sensors/DBP", diastolicPressure)) {
  Serial.println("DBP upload PASSED");
} else {
  Serial.println("DBP upload FAILED: " + fbdo.errorReason());
}

if (Firebase.RTDB.setFloat(&fbdo, "Sensors/Body_Temp", bodyTemp)) {
      Serial.println("Body Temp upload PASSED");
    } else {
      Serial.println("Body Temp upload FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
  }

  delay(2000);
}

// Function to Check if Finger is Placed (Valid IR signal)
bool isValidFingerPresent(uint32_t irBuffer[]) {
  uint32_t minIr = irBuffer[0];
  uint32_t maxIr = irBuffer[0];

  // Check the min and max IR values to determine if there's a valid signal
  for (int i = 1; i < 100; i++) {
    if (irBuffer[i] < minIr) minIr = irBuffer[i];
    if (irBuffer[i] > maxIr) maxIr = irBuffer[i];
  }

  // If the max IR value is too low, no finger is placed
  if (maxIr - minIr < 5000) { // Threshold to detect the signal strength (can be adjusted)
    return false; // No valid signal, so no finger detected
  }

  return true; // Finger detected (valid signal)
}

// Function to Estimate Diastolic Blood Pressure (DBP)
void estimateDiastolicBloodPressure(uint32_t irBuffer[], byte bufferLength) {
  int diastolicPeak = irBuffer[0];

  // Find the minimum value (diastolic peak)
  for (int i = 1; i < bufferLength; i++) {
    if (irBuffer[i] < diastolicPeak) {
      diastolicPeak = irBuffer[i];
    }
  }

  // For estimation purposes, we map the diastolic peak value to a blood pressure range
  // The mapping here is simplified and needs calibration
  diastolicPressure = map(diastolicPeak, 0, 100000, 60, 90);  // Mapping IR peak to diastolic pressure range (simplified)
}