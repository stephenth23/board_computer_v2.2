#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <SimpleKalmanFilter.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>

// Kalman Filter
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

// Sensors accel & barometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor * bmp_pressure = bmp.getPressureSensor();

// Useful pins
Servo Servo1;
const int servoPin = 9;
const int buzzer = 5;
const int ledPin = 10;

int state = 1;
const short ground_idle = 1;
const short powered_flight = 2;
const short unpowered_flight = 3;
const short balistic_descent = 4;
const short chute_descent = 5;
const short landing_idle = 6;

// Functions
void initialize_ADXL345();
void initialize_BMP280();
void error();

// ============================================ SETUP ============================================

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(buzzer, OUTPUT);

  Servo1.attach(servoPin);
  Servo1.write(30);

  Serial.begin(9600);

  // user interface -> startup
  for (int i = 6; i <= 9; i++) {
    tone(buzzer, i * 100);
    delay(100);
  }

  noTone(buzzer);
  delay(500);

  initialize_ADXL345();
  initialize_BMP280();

  tone(buzzer, 600);
  delay(100);
  tone(buzzer, 1000);
  delay(100);
  noTone(buzzer);
}

// ============================================ LOOP ============================================

int currentRead = 0, counter = 1;
float estimated_alt_first, estimated_alt_second = 0;

void loop() {
  sensors_event_t event;
  accel.getEvent( & event);

  switch (state) {
    
    // GROUND_IDLE STATE - the caraft is in a waiting state for a spike in acceleration. the spike will be twice veriffied
    case ground_idle: {
      Serial.println(event.acceleration.x);
      currentRead = event.acceleration.x;

      if (currentRead >= 15) // de ce am pus aici 14.. trb sa verific
      {
        if (counter == 2) {
          Serial.println("checked 2 times the x acceleration for a spike in acceleration, going to next state");
          state += 1;
          break;
        }
        delay(500);
        counter++;
      } else {
        counter = 1;
      }
      break;
    }

    // POWERED_FLIGHT STATE - the rocket is in flight, powered by the motor. the BC will wait and check for the burnout 
    case powered_flight: {
      tone(buzzer, 500);
      if (event.acceleration.x <= 2) {
        if (counter == 3) {
          Serial.println("checked 2 times the x acceleration for unpowered flight, going to next state");
          state += 1;
          break;
        }
        delay(500);
        counter++;
      } else {
        counter = 2;
      }
      break;
    }

    // UNPOWERED_FLIGHT STATE - the rocket reached the burnout. it is in a costing (decelerating) state. waiting and checking for apogee
    case unpowered_flight: {
      tone(buzzer, 700);
      estimated_alt_second = pressureKalmanFilter.updateEstimate(bmp.readAltitude());

      if (estimated_alt_first > estimated_alt_second) {
        if (counter == 4 ) {
          Serial.println("checked 2 times the x acceleration for unpowered flight, going to next state");
          state += 1;
          break;
        }
        delay(1000);
        counter++;
      }
      else {
        counter = 3;
        estimated_alt_first = estimated_alt_second;
      }
      break;
    }

    // BALISTIC_DESCENT STATE - the rocket is descending balistically. waiting for optimal altitude to deploy parachute
    case balistic_descent: {
      tone(buzzer, 900);
      float estimated_alt = pressureKalmanFilter.updateEstimate(bmp.readAltitude());
      Serial.println(estimated_alt);
      if (estimated_alt <= 470.0f) {
        state += 1;
        Servo1.write(65); // deployment servo angle
        delay(100);
        Servo1.write(30);
        delay(100);
        Servo1.write(65); // deployment servo angle
        delay(100);
        Servo1.write(30);
        delay(100);
      }
      break;
    }

    // CHUTE_DESCENT STATE - the rocket is descending under parachute. waiting to land and to be recovered
    case chute_descent: {
      while (1)
      {
        tone(buzzer, 600);
        delay(100);
        tone(buzzer, 1000);
        delay(100);
        noTone(buzzer);
      }
    }

  }
}

// ============================================ FUNCTIONS ============================================

void initialize_ADXL345() {
  if (!accel.begin()) {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    error();
  }

  accel.setRange(ADXL345_RANGE_16_G);
  accel.printSensorDetails();
}

void initialize_BMP280() {
  // barometer sensor setup
  Serial.println(F("BMP280 Sensor event test"));
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    error();
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_500);

  bmp_pressure -> printSensorDetails();
}

void error() {
  while (1) {
    digitalWrite(ledPin, HIGH);
    tone(buzzer, 800);
    delay(100);

    digitalWrite(ledPin, LOW);
    delay(100);

    digitalWrite(ledPin, HIGH);
    tone(buzzer, 600);
    delay(100);

    digitalWrite(ledPin, LOW);
    delay(100);
  }
}