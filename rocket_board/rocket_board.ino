#include <Wire.h>
#include <Servo.h>
#include <VB_MPU9250.h>
#include <VB_BMP280.h>

// Constants

#define MPU_ADDRESS 0x68
#define MAGNETOMETER_ADDRESS 0x76
const int READY_LED = 5;
const int SPEAKER_PIN = A1;
const int SERVO_PIN = 9;
const int VOLTAGE_PIN = A0;
const int BUTTON_PIN = A3;
const int SET_RADIO_PIN = 7;
const int CS_RADIO_PIN = 4;
bool servoEnabled = false;
const float GRAVITATIONAL_ACCELERATION = 9.78;

Servo servo;

// Acceleration class

class Acceleration {
private:
  float offsetX, offsetY, offsetZ;

public:
  bool isCalibrated; 
  float x;
  float y;
  float z;

  Acceleration() : x(0), y(0), z(0), isCalibrated(false) {}

  void calibrate(VB_MPU9250& mpu, int numSamples, int delayTime) {
    float sumX = 0, sumY = 0, sumZ = 0;

    for (int i = 0; i < numSamples; ++i) {
      mpu.read();
      sumX += mpu.ax;
      sumY += mpu.ay;
      sumZ += mpu.az;
      delay(delayTime);
    }

    offsetX = sumX / numSamples;
    offsetY = sumY / numSamples;
    offsetZ = sumZ / numSamples; // - gravitationalAcceleration (can be added back if needed)

    setOffsets(offsetX, offsetY, offsetZ);

    isCalibrated = true;
    Serial.println("Accelerometer calibration complete:");
    Serial.print("Offset X: "); Serial.println(offsetX);
    Serial.print("Offset Y: "); Serial.println(offsetY);
    Serial.print("Offset Z: "); Serial.println(offsetZ);
  }

  void update(float rawX, float rawY, float rawZ) {
    x = rawX - offsetX;
    y = rawY - offsetY;
    z = rawZ - offsetZ;
  }

  void setOffsets(float offsetX, float offsetY, float offsetZ) {
    this->offsetX = offsetX;
    this->offsetY = offsetY;
    this->offsetZ = offsetZ;
  }

  float magnitude() const {
    return sqrt(sq(x) + sq(y) + sq(z));
  }
};

// Rocket class

class Rocket {
public:
  Acceleration acceleration;
  float barometerPressure;
  float altitude;
  float previousAltitude;
  float temperature;
  float voltage;

  bool startPoint;
  bool apogeePoint;
  bool activatePoint;
  bool parachutePoint;
  bool landingPoint;

  Rocket() : mpuConnection(false), barometerConnection(false), hasRocketBeenReady(false), altitudeOffset(0.0), startPoint(false), apogeePoint(false), activatePoint(false), parachutePoint(false), landingPoint(false) {}

  void initialize() {
    setupSensors();
    acceleration.calibrate(mpu, 100, 10);
    servo.write(65);
    while (!isReady());
    onReady();
  }

  bool isReady() {
    return (mpuConnection & barometerConnection & acceleration.isCalibrated);
  }

  void readSensors() {
    mpu.read();
    barometer.read();
  }

  void processSensorData() {
    barometerPressure = pascalToMmHg(barometer.pres);
    previousAltitude = altitude;
    altitude = barometer.alti - altitudeOffset;
    temperature = barometer.temp;
    acceleration.update(mpu.ax, mpu.ay, mpu.az);
    voltage = getArduinoVoltage();
  }

void updateStatus() {
  if (!startPoint) {
    startPoint = (acceleration.magnitude() > 1 && acceleration.y > 0);
  }

  if (startPoint && !apogeePoint) {
    apogeePoint = (acceleration.magnitude() <= 1 && altitude < previousAltitude || acceleration.y < 0);
  }

  if (apogeePoint && !activatePoint) {
    activatePoint = (altitude < previousAltitude); 
  }

  if (activatePoint && !parachutePoint) {
    parachutePoint = true;
    servo.write(180);
  }

  if (startPoint && apogeePoint && !landingPoint) {
    landingPoint = (acceleration.magnitude() < 0.1 && abs(acceleration.y) < 0.1 && altitude == previousAltitude);
  }
}

  void outputData() {
    /*
    Serial.println("===== Rocket Data =====");

    Serial.print("Altitude: ");
    Serial.print(altitude);
    Serial.println(" meters");

    Serial.print("Voltage: ");
    Serial.print(voltage);
    Serial.println(" V");

    Serial.print("Acceleration: ");
    Serial.print(acceleration.magnitude());
    Serial.println(" m/s^2");

    Serial.print("Barometer Pressure: ");
    Serial.print(barometerPressure);
    Serial.println(" mmHg");

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.println("=======================");
    */

    String dataString = String("1A;") +
                        String(millis()) + ";" +
                        String(voltage, 2) + ";" +
                        String(altitude, 1) + ";" +
                        String(acceleration.magnitude(), 1) + ";" +
                        String(startPoint) + ";" +
                        String(apogeePoint) + ";" +
                        String(activatePoint) + ";" +
                        String(parachutePoint) + ";" +
                        String(landingPoint) + "\n";

    // Print the data to Serial and Serial1
    Serial.println(dataString);
    Serial1.println(dataString);
  }
private:
  VB_BMP280 barometer;
  VB_MPU9250 mpu;

  bool mpuConnection;
  bool barometerConnection;
  bool isAccelerometerCalibrated;
  bool hasRocketBeenReady; // Needed to call the onReady() function once
  float altitudeOffset;

  void setupSensors() {
    if (mpu.begin() != MPU_OK) {
      handleSensorConnectionFailure("MPU");
    }

    mpuConnection = true;
    Serial.println("MPU connected successfully.");

    if (barometer.begin() != true) {
      handleSensorConnectionFailure("Barometer");
    }

    barometerConnection = true;
    Serial.println("Barometer connected successfully.");
  }

  void handleSensorConnectionFailure(const char* sensorName) {
    while (true) {
      Serial.print(sensorName);
      Serial.println(" connection failed. Attempting again in 2 seconds.");
      delay(2000);
    }
  }

  void onReady() {
    if (!hasRocketBeenReady) {
      hasRocketBeenReady = true;
      Serial.println("The rocket is ready.");
      digitalWrite(READY_LED, HIGH);
      tone(SPEAKER_PIN, 10000, 500);
    }
  }

  float pascalToMmHg(float pressureInPascals) {
    const float pascalsInOneMmHg = 133.322;
    return pressureInPascals / pascalsInOneMmHg;
  }

  float getArduinoVoltage() {
    int sensorValue = analogRead(VOLTAGE_PIN);
    return sensorValue * (5.0 / 1023.0);
  }
};

Rocket rocket;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  //while (!Serial);

  pinMode(READY_LED, OUTPUT);
  digitalWrite(READY_LED, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Sleep mode: CS: 0, SET: Doesn't matter (Probably 0)
  // Working mode: CS: 1, SET: 1
  // Setting mode: CS: 1, SET: 0
  pinMode(CS_RADIO_PIN, INPUT);
  pinMode(SET_RADIO_PIN, INPUT);

  Wire.begin();
  servo.attach(SERVO_PIN);

  rocket.initialize();
}

void loop() {
  while (!rocket.isReady());

  servoTest();

  rocket.readSensors();
  rocket.processSensorData();
  //sendCommandToRadio();
  rocket.updateStatus();
  rocket.outputData();
}

void servoTest() {
  if (digitalRead(BUTTON_PIN) == HIGH) {
    servoEnabled = !servoEnabled;
    while (digitalRead(BUTTON_PIN) == HIGH);

    if (!servoEnabled) {
      servo.write(65);
    } else {
      Serial.print("Setting servo angle to: ");
      Serial.println(0);
      Serial1.println(0);
      servo.write(0);
    }
  }
}

/*
void setRadioToSetting() {
  delay(2000);
  pinMode(SET_RADIO_PIN, OUTPUT);
}

void sendCommandToRadio() {
  char command[] = {0xAA, 0xFA, 0x01};
  for (int i = 0; i < sizeof(command); i++) {
    Serial1.write(command[i]);
  }
}
*/
