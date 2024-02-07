#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

const int SET_RADIO_PIN = A3;
const int CS_RADIO_PIN = A2;

String incomingData = "";

//LiquidCrystal_I2C lcd(0x3F, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
  Serial.begin(9600);

  //lcd.begin(16, 2);
  
  // Sleep mode: CS: 0, SET: Doesn't matter (Probably 0)
  // Working mode: CS: 1, SET: 1
  // Setting mode: CS: 1, SET: 0
  pinMode(CS_RADIO_PIN, INPUT);
  pinMode(SET_RADIO_PIN, INPUT);
}

void loop() {
    while (Serial.available() > 0) {
        char incomingChar = Serial.read();

        if (incomingChar == '\n') {  // Assuming newline is the end of your data
            //Serial.print("I received: ");
            Serial.println(incomingData);
            //lcd.clear();
            //lcd.setCursor(0, 0);
            //lcd.print(incomingData);
            incomingData = "";  // Reset the String for the next data
        } else {
            incomingData += incomingChar;  // Append the character to the String
        }
    }
}