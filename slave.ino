#include <SPI.h>
#include "Adafruit_GFX.h"
#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>

#include <regex>  // Include regex library

// Pin definitions for TFT
#define LCD_CS 15
#define LCD_CD 2
#define LCD_WR 4
#define LCD_RD 5
#define LCD_RESET 23

MCUFRIEND_kbv tft;

#define MINPRESSURE 200
#define MAXPRESSURE 40000

const int XP = 27, XM = 15, YP = 4, YM = 14; // Touchscreen pins
const int TS_LEFT = 570, TS_RT = -2640, TS_TOP = 775, TS_BOT = -2670;

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

#define RXp2 16
#define TXp2 17

String height = "";
String weight = "";
String bmi = "";
int heartRate = 0;
int spo2 = 0;
int dbp = 0; // DBP value
float bodyTemp = 0.0;

Adafruit_GFX_Button on_btn, stop_btn;

int pixel_x, pixel_y;
bool screenChanged = false;

bool Touch_getXY(void) {
  TSPoint p = ts.getPoint();
  pinMode(YP, OUTPUT);
  pinMode(XM, OUTPUT);
  digitalWrite(YP, HIGH);
  digitalWrite(XM, HIGH);
  p.z = abs(p.z);
  bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);
  if (pressed) {
    pixel_x = map(p.x, TS_LEFT, TS_RT, 0, tft.width());
    pixel_y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
    Serial.print("Touch detected at: ");
    Serial.print(pixel_x);
    Serial.print(", ");
    Serial.println(pixel_y);
  }
  return pressed;
}

#define BLACK 0x0000
#define WHITE 0xFFFF
#define CYAN 0x07FF
#define RED 0xF800
#define YELLOW 0xFFE0
#define BLUE 0x001F

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXp2, TXp2); // Initialize UART on pins 16 (RX2) and 17 (TX2)

  // Initialize TFT
  uint16_t identifier = tft.readID();
  tft.begin(identifier);
  tft.setRotation(1); // Landscape mode
  tft.fillScreen(BLACK);  // Initial black screen

  // Draw START button at center
  on_btn.initButton(&tft, tft.width()/2, tft.height()/2, 180, 80, WHITE, CYAN, BLACK, "START", 4);
  on_btn.drawButton(false); // Initial button state

  // Flag to ensure health data is not displayed initially
  screenChanged = false;
}

// Declare a new state variable to track the readings page
bool isOnReadingsPage = false;

void loop() {
  bool down = Touch_getXY();

  // Handle button press
  if (!screenChanged) {
    on_btn.press(down && on_btn.contains(pixel_x, pixel_y));
    if (on_btn.justReleased()) {
      on_btn.drawButton();
    }
    if (on_btn.justPressed()) {
      Serial.println("START button pressed!");
      on_btn.drawButton(true);  // Highlight the button
      tft.fillScreen(BLACK);  // Clear the screen before moving to the readings page
      displayHealthData();     // Display the health data
      screenChanged = true;    // Prevent the button from showing again
      isOnReadingsPage = true; // Set the flag indicating the readings page is active
    }
  }

  if (Serial2.available()) {
    //String receivedData = Serial2.readString();  // Receive data
    String receivedData = Serial2.readStringUntil('\n');  // Read until newline
    Serial.println(receivedData);               // Debugging

    // Parse and display the received data
    if (receivedData.indexOf("Height:") >= 0) {
      int heightStart = receivedData.indexOf("Height:") + 8;
      //int heightEnd = receivedData.indexOf("in");
      //String height = receivedData.substring(heightStart, heightEnd);
      height = receivedData.substring(heightStart);

      //tft.print("inches");
      // Only update the display if we are on the readings page
      if (isOnReadingsPage) {
        updateHealthData();
      }
    }

    if (receivedData.indexOf("Weight:") >= 0) {
      int weightStart = receivedData.indexOf("Weight:") + 8;
      int weightEnd = receivedData.indexOf("kg");
      //String weight = receivedData.substring(weightStart, weightEnd);
      weight = receivedData.substring(weightStart, weightEnd);

      // Only update the display if we are on the readings page
      if (isOnReadingsPage) {
        updateHealthData();
      }
    }

    if (receivedData.indexOf("BMI:") >= 0) {
      int bmiStart = receivedData.indexOf("BMI:") + 5;
      //String bmi = receivedData.substring(bmiStart);
      bmi = receivedData.substring(bmiStart);

      // Only update the display if we are on the readings page
      if (isOnReadingsPage) {
        updateHealthData();
      }
    }

    // Parse the received data to extract Heart Rate, SpO2, and DBP
    if (receivedData.indexOf("Heart Rate:") >= 0 &&
        receivedData.indexOf("SpO2:") >= 0 &&
        receivedData.indexOf("DBP:") >= 0) {
      int hrStart = receivedData.indexOf("Heart Rate:") + 12;
      int hrEnd = receivedData.indexOf(" bpm");
      heartRate = receivedData.substring(hrStart, hrEnd).toInt();

      int spo2Start = receivedData.indexOf("SpO2:") + 6;
      int spo2End = receivedData.indexOf(" %", spo2Start);
      spo2 = receivedData.substring(spo2Start, spo2End).toInt();

      int dbpStart = receivedData.indexOf("DBP:") + 5;
      int dbpEnd = receivedData.indexOf(" mmHg", dbpStart);
      dbp = receivedData.substring(dbpStart, dbpEnd).toInt();

      // Only update the display if we are on the readings page
      if (isOnReadingsPage) {
        updateHealthData();
      }
    }

    if (receivedData.indexOf("Body Temperature:") >= 0) {
      Serial.println("Data matched, parsing...");
      
      // Parsing logic
      int startIdx = receivedData.indexOf("Body Temperature:") + 17;
      int endIdx = receivedData.indexOf("F", startIdx);
      if (startIdx > 0 && endIdx > startIdx) {
        String tempStr = receivedData.substring(startIdx, endIdx);
        bodyTemp = tempStr.toFloat();
        //Serial.print("Parsed Body Temp: ");
        //Serial.println(bodyTemp);
        Serial.println(bodyTemp);

        if (isOnReadingsPage) {
          updateHealthData();  // Update TFT display
        }
      } else {
        Serial.println("Error parsing body temperature.");
      }
    } else {
      Serial.println("Data format mismatch, skipping...");
    }
  }
}

// Function to display the labels for health data
void displayHealthData() {
  tft.setTextColor(YELLOW);
  tft.fillScreen(RED);  // Red background only for health data page
  tft.setTextSize(3);

  int yPosition = 10;
  int lineHeight = 45; // Adjust line height for font size 3

  // Display the health-related data labels
  displayData("Body Temp: ", 10, yPosition);
  yPosition += lineHeight;
  displayData("SpO2: ", 10, yPosition);
  yPosition += lineHeight;
  displayData("Heart Rate: ", 10, yPosition);
  yPosition += lineHeight;
  displayData("DBP: ", 10, yPosition);
  yPosition += lineHeight;
  displayData("Height: ", 10, yPosition);
  yPosition += lineHeight;
  //displayData("Height(in): ", 10, yPosition);
  //yPosition += lineHeight;
  displayData("Weight: ", 10, yPosition);
  yPosition += lineHeight;
  displayData("BMI: ", 10, yPosition);
}

// Function to update Heart Rate, SpO2, and DBP data on the TFT screen
void updateHealthData() {
  tft.setTextColor(WHITE, RED); // White text on a red background to overwrite previous values

  // Clear the area where the temperature is displayed
  tft.fillRect(200, 10, 150, 30, RED);  // Clear the region
  tft.setCursor(200, 10);
  
    // Display the updated temperature with two decimal places
    tft.printf("%.2f F", bodyTemp);
    
    // Clear area and update SPO2 value
    tft.fillRect(110, 55, 100, 30, RED); // Clear the area where DBP is displayed
    tft.setCursor(110, 55);
    tft.print(spo2);
    tft.print(" %");

  // Clear area and update Heart Rate value
    tft.fillRect(220, 100, 140, 30, RED); // Clear the area where DBP is displayed
    tft.setCursor(220, 100);
    tft.print(heartRate);
    tft.print(" bpm");

    // Clear area and update DBP value
    tft.fillRect(90, 145, 140, 30, RED); // Clear the area where Heart Rate is displayed
    tft.setCursor(90, 145);
    tft.print(dbp);
    tft.print(" mmHg");

  // Clear area and update Height value
    tft.fillRect(140, 190, 260, 30, RED);  // Clear previous value
    tft.setCursor(140, 190);
    tft.print(height);
    //tft.print("in");

  // Clear area and update Weight value
    tft.fillRect(140, 235, 150, 30, RED);  // Clear previous value
    tft.setCursor(140, 235);
    tft.print(weight);
    tft.print("kg");

  // Clear area and update BMI value
    tft.fillRect(90, 280, 260, 30, RED);  // Clear previous value
    tft.setCursor(90, 280);
    tft.print(bmi);
    tft.print(" kg/m2");

}

// Helper function to print each data label
void displayData(String label, int x, int y) {
  tft.setCursor(x, y);
  tft.print(label);
}
