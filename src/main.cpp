#include <Arduino.h>
#include <moto_NeoPixel.h>
#define DEBUG 0
#define BLACK_LINE false
moto_NeoPixel strip13 = moto_NeoPixel(2, 13, NEO_GRB + NEO_KHZ800);


#define rightMotor 4
#define rightMotorPWM 6
#define leftMotor 7
#define leftMotorPWM 5

const uint8_t SensorCount = 5;
const uint8_t SensorPins[SensorCount] = { A1, A2, A3, A4, A5 };
const uint16_t samplesPerSensor = 4;
const uint16_t maxValue = 1023;

uint16_t maximum[SensorCount];
uint16_t minimum[SensorCount];
uint16_t sensorValues[SensorCount] = { 0 };
uint16_t referenceValues = 200;
uint16_t  Hspeed = 100;
uint16_t  Mspeed = 80;
uint16_t  Lspeed = 70;
uint16_t Dtime = 25;

bool calibrate_initialized = false;

// put function declarations here:
void colorled13(int number, String c);
void calibrate();
void read();
void readCalibrated();
void forward(int speed);
void backward(int speed);
void right(int speed);
void left(int speed);
void side_right(int speed);
void side_left(int speed);
void wait();

void setup() {
  // put your setup code here, to run once:
  pinMode(12, INPUT);
  pinMode(7, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  // 每個部位的最大值與最小值更新
  for (uint8_t i = 0; i < SensorCount;i++)
  {
    maximum[i] = 0;
    minimum[i] = maxValue;
  }

  Serial.begin(9600);
  while ((digitalRead(12) == HIGH)) {
  }
#if DEBUG
  Serial.println("calibrate start");
#endif
  for (int j = 0;j < 1;j++) {
    for (int i = 0; i < 100;i++) {
      if (i < 25 || i >= 75) {
        right(50);
      }
      else
      {
        left(50);
      }
      calibrate();
      delay(30);
    }
  }
  wait();
#if DEBUG
  Serial.println("calibrate start");
  for (int i = 0;i < SensorCount;i++) {
    Serial.println("maximum");
    Serial.println(maximum[i]);
    Serial.println("minimum");
    Serial.println(minimum[i]);
  }
#endif
  while ((digitalRead(12) == HIGH)) {
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  readCalibrated();
#if DEBUG
  for (int i = 1; i <= SensorCount; i = i + 1) {
    Serial.println((sensorValues[(i - 1)]));
  }
  delay(500);
#endif
  if (sensorValues[1] > referenceValues && sensorValues[2] < referenceValues && sensorValues[3] > referenceValues) {
    forward(Mspeed);
    delay(Dtime);
  }
  else if (sensorValues[1] < referenceValues && sensorValues[2] > referenceValues && sensorValues[3] > referenceValues) {
    side_right(Mspeed);
    delay(Dtime);
  }
  else if (sensorValues[1] > referenceValues && sensorValues[2] > referenceValues && sensorValues[3] < referenceValues) {
    side_left(Mspeed);
    delay(Dtime);
  }
  else if (sensorValues[1] < referenceValues && sensorValues[2] < referenceValues && sensorValues[3] > referenceValues) {
    side_right(Hspeed);
    delay(Dtime);
  }
  else if (sensorValues[1] > referenceValues && sensorValues[2] < referenceValues && sensorValues[3] < referenceValues) {
    side_left(Hspeed);
    delay(Dtime);
  }
  else if (sensorValues[0] < referenceValues && sensorValues[1] < referenceValues && sensorValues[2] < referenceValues && sensorValues[3] < referenceValues && sensorValues[4] < referenceValues) {
    forward(Lspeed);
    colorled13(1, "#ffffff");
    colorled13(2, "#ffffff");
    delay(200);
    colorled13(1, "#000000");
    colorled13(2, "#000000");
    forward(Mspeed);
  }
  if (sensorValues[0] > referenceValues && sensorValues[1] > referenceValues && sensorValues[2] > referenceValues && sensorValues[3] > referenceValues && sensorValues[4] > referenceValues) {
  }
}

void colorled13(int number, String c)
{
  long rgb_number = strtol(&c[1], NULL, 16);
  int r = rgb_number >> 16;
  int g = rgb_number >> 8 & 0xFF;
  int b = rgb_number & 0xFF;
  if (number <= 0) number = 0;
  else  number--;
  strip13.setPixelColor(number, strip13.Color(r, g, b));
  strip13.show();
}


// put function definitions here:
void calibrate() {
  uint16_t maxSensorValues[SensorCount];
  uint16_t minSensorValues[SensorCount];
  // 若SensorPins為null則返回
  if (SensorPins == nullptr) { return; }

  calibrate_initialized = true;

  for (uint8_t j = 0; j < 10; j++)
  {
    read();
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      if ((j == 0) || (sensorValues[i] > maxSensorValues[i]))
      {
        maxSensorValues[i] = sensorValues[i];
      }

      if ((j == 0) || sensorValues[i] < minSensorValues[i])
      {
        minSensorValues[i] = sensorValues[i];
      }
    }
  }

  for (uint8_t i = 0; i < SensorCount;i++) {
    if (minSensorValues[i] < minimum[i])
    {
      minimum[i] = minSensorValues[i];
    }
    if (maxSensorValues[i] > maximum[i])
    {
      maximum[i] = maxSensorValues[i];
    }
  }
}

void read() {
  // 初始化sensorValues
  for (uint8_t i = 0;i < SensorCount; i++) {
    sensorValues[i] = 0;
    pinMode(SensorPins[i], INPUT);
  }



  for (uint8_t j = 0; j < samplesPerSensor; j++)
  {
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      // 加總所有感測器讀值
      sensorValues[i] += analogRead(SensorPins[i]);
      delayMicroseconds(10);
    }
  }

  for (uint8_t i = 0; i < SensorCount;i++)
  {
    // 將讀值取平均值後四捨五入
    sensorValues[i] = (sensorValues[i] + (samplesPerSensor >> 1)) / samplesPerSensor;
  }
}

void readCalibrated() {
  read();
  for (uint8_t i = 0; i < SensorCount; i++) {
    uint16_t calmin, calmax;
    calmax = maximum[i];
    calmin = minimum[i];


    uint16_t denominator = calmax - calmin;
    int16_t value = 0;
    if (denominator != 0)
    {
      value = (((int32_t)sensorValues[i]) - calmin) * 1000 / denominator;
    }

    if (value < 0) { value = 0; }
    else if (value > 1000) { value = 1000; }


    sensorValues[i] = value;
  }
  if (BLACK_LINE) {
      for (int i = 0; i < SensorCount; i++)
      {
        sensorValues[i] = 1000 - sensorValues[i];
      }
    }
}

void forward(int speed) {
  digitalWrite(leftMotor, LOW);
  analogWrite(leftMotorPWM, speed);
  digitalWrite(rightMotor, HIGH);
  analogWrite(rightMotorPWM, speed);
}

void backward(int speed) {
  digitalWrite(leftMotor, HIGH);
  analogWrite(leftMotorPWM, speed);
  digitalWrite(rightMotor, LOW);
  analogWrite(rightMotorPWM, speed);
}

void right(int speed) {
  digitalWrite(leftMotor, LOW);
  analogWrite(leftMotorPWM, speed);
  digitalWrite(rightMotor, LOW);
  analogWrite(rightMotorPWM, speed);
}

void left(int speed) {
  digitalWrite(leftMotor, HIGH);
  analogWrite(leftMotorPWM, speed);
  digitalWrite(rightMotor, HIGH);
  analogWrite(rightMotorPWM, speed);
}

void side_right(int speed) {
  digitalWrite(leftMotor, LOW);
  analogWrite(leftMotorPWM, speed);
  digitalWrite(rightMotor, LOW);
  analogWrite(rightMotorPWM, 0);
}

void side_left(int speed) {
  digitalWrite(leftMotor, HIGH);
  analogWrite(leftMotorPWM, 0);
  digitalWrite(rightMotor, HIGH);
  analogWrite(rightMotorPWM, speed);
}

void wait() {
  analogWrite(leftMotorPWM, 0);
  analogWrite(rightMotorPWM, 0);
}