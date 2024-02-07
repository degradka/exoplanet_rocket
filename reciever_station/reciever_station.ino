#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// SCL is connected to A5
// SDA is connected to A4
const int SET_RADIO_PIN = A3;
const int CS_RADIO_PIN = A2;
const int JOY_VRX_PIN = A1;

const int JOY_R_THRESHOLD = 0;
const int JOY_L_THRESHOLD = 600;
const int PAGE_CHANGE_TIME = 1000; // 1 second
const int UPDATE_INTERVAL = 1000;

String incomingData = "";

LiquidCrystal_I2C lcd(0x3F, 16, 2);

int currentPage = 0;
unsigned long pageChangeStartTime = 0;
unsigned long lastUpdateTime = 0;

// Rocket data variables
String rocketID = "NN"; // ID
unsigned long rocketTimestamp = 69000; // T
float rocketVoltage = 0.23; // V
float rocketAltitude = 250.5; // Alt
float rocketAcceleration = -20.2; // A
int rocketStartPoint = 0; // S
int rocketApogeePoint = 0; // H
int rocketActivatePoint = 0; // R
int rocketParachutePoint = 0; // P
int rocketLandingPoint = 0; // F

void setup() {
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Exoplanet Rocket");
  lcd.setCursor(0, 1);
  lcd.print("Radio Receiver");
  
  // Sleep mode: CS: 0, SET: Doesn't matter (Probably 0)
  // Working mode: CS: 1, SET: 1
  // Setting mode: CS: 1, SET: 0
  pinMode(CS_RADIO_PIN, INPUT);
  pinMode(SET_RADIO_PIN, INPUT);
}

void loop() {
    int joyValue = analogRead(JOY_VRX_PIN);

  if (joyValue == JOY_R_THRESHOLD) {
    if (millis() - pageChangeStartTime > PAGE_CHANGE_TIME) {
      changePage(1);
      pageChangeStartTime = millis();
    }
  } else if (joyValue > JOY_L_THRESHOLD) {
    if (millis() - pageChangeStartTime > PAGE_CHANGE_TIME) {
      changePage(-1);
      pageChangeStartTime = millis();
    }
  } else {
    pageChangeStartTime = millis(); // Reset the timer when the joystick is not moved
  }

  // Update the display every second
  if (millis() - lastUpdateTime >= UPDATE_INTERVAL) {
    if (currentPage != 0) {
      updatePage();
      lastUpdateTime = millis();
    }
  }

    while (Serial.available() > 0) {
        char incomingChar = Serial.read();

        if (incomingChar == '\n') {  // Assuming newline is the end of the data
            parseRocketData();
            Serial.println(incomingData);
            incomingData = "";  // Reset the String for the next data
        } else {
            incomingData += incomingChar;  // Append the character to the String
        }
    }
}

void changePage(int direction) {
  lcd.clear();
  currentPage += direction;

  if (currentPage == -1) {
    currentPage = 2; // Wrap around to the last page
  } else if (currentPage == 3) {
    currentPage = 0; // Wrap around to the first page
  }

  if (currentPage == 0) {
    lcd.setCursor(0, 0);
    lcd.print("Exoplanet Rocket");
    lcd.setCursor(0, 1);
    lcd.print("Radio Receiver");
  } else if (currentPage == 1) {
    lcd.setCursor(0, 0);
    lcd.print("ID: " + String(rocketID)); // rocketID is already a string, but just in case
    lcd.setCursor(7, 0);
    lcd.print("V: " + String(rocketVoltage, 2));
    lcd.setCursor(0, 1);
    lcd.print("T: " + String(rocketTimestamp));
  } else if (currentPage == 2) {
    lcd.setCursor(0, 0);
    lcd.print("Alt: " + String(rocketAltitude));
    lcd.setCursor(0, 1);
    lcd.print("A: " + String(rocketAcceleration));
    lcd.setCursor(12, 0);
    lcd.print("S" + String(rocketStartPoint));
    lcd.setCursor(14, 0);
    lcd.print("H" + String(rocketApogeePoint));
    lcd.setCursor(10, 1);
    lcd.print("R" + String(rocketActivatePoint));
    lcd.setCursor(12, 1);
    lcd.print("P" + String(rocketParachutePoint));
    lcd.setCursor(14, 1);
    lcd.print("F" + String(rocketLandingPoint));
  }
}

void updatePage() {
  changePage(0);
}

void parseRocketData() {
  // Find the index of the semicolon after the team ID
  size_t semicolonIndex = incomingData.indexOf(';');

  // Check if a semicolon is found and if there's enough room for a valid team ID
  if (semicolonIndex != -1 && semicolonIndex + 2 <= incomingData.length()) {
    // Extract the team ID portion from the incoming data
    rocketID = incomingData.substring(0, semicolonIndex);

    // Update the rest of the variables as usual
    char dataBuffer[100];
    incomingData.substring(semicolonIndex + 1).toCharArray(dataBuffer, sizeof(dataBuffer));

    char* token = strtok(dataBuffer, ";");

    if (token != NULL) rocketTimestamp = atol(token);

    token = strtok(NULL, ";");
    if (token != NULL) rocketVoltage = atof(token);

    token = strtok(NULL, ";");
    if (token != NULL) rocketAltitude = atof(token);

    token = strtok(NULL, ";");
    if (token != NULL) rocketAcceleration = atof(token);

    token = strtok(NULL, ";");
    if (token != NULL) rocketStartPoint = atoi(token);

    token = strtok(NULL, ";");
    if (token != NULL) rocketApogeePoint = atoi(token);

    token = strtok(NULL, ";");
    if (token != NULL) rocketActivatePoint = atoi(token);

    token = strtok(NULL, ";");
    if (token != NULL) rocketParachutePoint = atoi(token);

    token = strtok(NULL, ";");
    if (token != NULL) rocketLandingPoint = atoi(token);
  }
}
