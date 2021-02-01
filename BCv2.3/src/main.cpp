#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <SimpleKalmanFilter.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>

// Kalman Filter
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;
float baseline = 0; // baseline pressure

// Sensors accel & barometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// Useful pins
Servo Servo1;
const int servoPin = 9;
const int buzzer = 5;
const int SDcard = 4;
const int ledPin = 10;

int state = 0;

const int ground_idle = 1;
const int powered_flight = 2;
const int unpowered_flight = 3;
const int balistic_descent = 4;
const int chute_descent = 5;
const int landing_idle = 6;

// Functions
void initialize_SDcard();
void initialize_ADXL345();
void initialize_BMP280();

void error();

// ============================================ SETUP ============================================

void setup()
{
  pinMode(ledPin, OUTPUT);
  pinMode(buzzer, OUTPUT);

  Servo1.attach(servoPin);
  Servo1.write(35);

  Serial.begin(9600);

  // user interface -> startup
  for (int i = 1; i <= 3; i++)
  {
    digitalWrite(ledPin, HIGH);
    tone(buzzer, 1000);
    delay(250);

    digitalWrite(ledPin, LOW);
    delay(250);

    digitalWrite(ledPin, HIGH);
    tone(buzzer, 600);
    delay(250);

    digitalWrite(ledPin, LOW);
    delay(250);
  }

  digitalWrite(ledPin, HIGH);
  noTone(buzzer);
  delay(500);

  // initialize_SDcard();
  initialize_ADXL345();
  initialize_BMP280();

  // getting baseline alt
  for (int i = 1; i <= 5; i++)
  {
    baseline = baseline + bmp.readAltitude();
    delay(100);
  }
  baseline = baseline / 5;

  digitalWrite(ledPin, LOW);
}

// ============================================ LOOP ============================================

void loop()
{
  while (1)
  {
    Servo1.write(35);
    delay(2000);
    Servo1.write(100);
    delay(1000);
  }

  sensors_event_t event;
  accel.getEvent(&event);

  switch (state)
  {
    case ground_idle:
    {
      if (event.acceleration.x >= 14)
      {
        delay(100);
        if (event.acceleration.x >= 14)
        {
          state += 1;
          // sdcard -> "vehicle launch"
          // sdcard -> time
        }
      }
      break;
    }

    case powered_flight:
    {
      if (event.acceleration.x <= 2)
      {
        delay(100);
        if (event.acceleration.x <= 2)
        {
          state += 1;
          // sdcard -> "meco"
          // sdcard -> time
        }
      }
      break;
    }

    case unpowered_flight:
    {
      float estimated_alt_first = pressureKalmanFilter.updateEstimate(bmp.readAltitude());
      delay(500);
      float estimated_alt_second = pressureKalmanFilter.updateEstimate(bmp.readAltitude());

      if (estimated_alt_first >= estimated_alt_second)
      {
        state += 1;
        // sdcard -> "passed apogee"
        // sdcard -> time
      }
      break;
    }

    case balistic_descent:
    {
      float estimated_alt = pressureKalmanFilter.updateEstimate(bmp.readAltitude());

      if (estimated_alt <= 25.0f)
      {
        state += 1;
        Servo1.write(45); // deployment angle
        // sdcard -> "parachute deployed"
        // sdcard -> time
      }
      break;
    }

    case chute_descent:
    {
      // sdcard -> "chute_descent"
      // sdcard -> time

      break;
    }

    case landing_idle:
    {

      break;
    }
  }

  // if (millis() > refresh_time) {
  //   Serial.print(bmp.readAltitude(), 6);
  //   Serial.print(", ");
  //   Serial.print(pressureKalmanFilter.updateEstimate(bmp.readAltitude()), 6);
  //   Serial.println();
  //   refresh_time=millis()+SERIAL_REFRESH_TIME;
  // }

  // Kalman Filter Pressure

  // Serial.print(F("Temperature = "));
  // Serial.print(bmp.readTemperature());
  // Serial.println(" *C");

  // Serial.print(F("Pressure = "));
  // Serial.print(bmp.readPressure()/100); //displaying the Pressure in hPa, you can change the unit
  // Serial.println(" hPa");

  // Serial.print(F("Approx altitude = "));
  // Serial.print(bmp.readAltitude(1029)); //The "1013.25" is the pressure(hPa) at sea level in day in your region
  // Serial.println(" m");                    //If you don't know it, modify it until you get your current altitude

  // Serial.println();
  // delay(2000);
}

void initialize_SDcard()
{
  Serial.print("Initializing SD card...");
  pinMode(SDcard, OUTPUT);

  if (!SD.begin())
  {
    Serial.println("Card failed, or not present");
    error();
  }

  Serial.println("card initialized.");
  File dataFile = SD.open("datalog.txt");
}

void initialize_ADXL345()
{
  if (!accel.begin())
  {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    error();
  }

  accel.setRange(ADXL345_RANGE_16_G);
  accel.printSensorDetails();
}

void initialize_BMP280()
{
  // barometer sensor setup

  Serial.println(F("BMP280 Sensor event test"));
  if (!bmp.begin(0x76))
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    error();
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  bmp_temp->printSensorDetails();
}

void error()
{
  while (1)
  {
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