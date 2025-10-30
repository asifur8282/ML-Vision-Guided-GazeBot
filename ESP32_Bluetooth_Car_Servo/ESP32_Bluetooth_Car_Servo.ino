#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include <ESP32Servo.h>
#include <BluetoothSerial.h> // Include the BluetoothSerial library

BluetoothSerial SerialBT;

Servo udservo;  // create servo object to control a servo
Servo lrservo;  // create servo object to control a servo

//Right motor
int enableRightMotor=2;
int rightMotorPin1=26;
int rightMotorPin2=27;
//Left motor
int enableLeftMotor=4;
int leftMotorPin1=14;
int leftMotorPin2=12;

#define MAX_MOTOR_SPEED 255

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;
int poslr = 90;
int posud=90;
int lr = 19;
int ud = 18;

int currentPosud = 90;
int currentPoslr = 90;



bool dabbleServoControl = false; 
unsigned long lastDabbleServoTime = 0;
const unsigned long dabbleServoTimeout = 500; //ms

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  } else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }

  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin1, LOW);
  }

  ledcWrite(rightMotorPWMSpeedChannel, abs(rightMotorSpeed));
  ledcWrite(leftMotorPWMSpeedChannel, abs(leftMotorSpeed));
}

void setUpPinModes() {
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  //Set up PWM for speed
  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel);

  rotateMotor(0, 0);
}

void setup() {
  setUpPinModes();
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  udservo.setPeriodHertz(50);     // standard 50 hz servo
  lrservo.setPeriodHertz(50);     // standard 50 hz servo
  lrservo.attach(lr, 1000, 2000); // attaches the servo on pin 19 to the servo object
  udservo.attach(ud, 1000, 2000); // attaches the servo on pin 18 to the servo object

  Dabble.begin("404 Bots Not Found");
  SerialBT.begin("404BNF"); // Start Bluetooth server with the name "ESP32-BT"
  Serial.println("Bluetooth Started! Ready to pair.");
}

void loop() {
  int rightMotorSpeed = 0;
  int leftMotorSpeed = 0;
  Dabble.processInput();

  // Motor control from Dabble
  if (GamePad.isRightPressed()) {
    rightMotorSpeed = MAX_MOTOR_SPEED;
    leftMotorSpeed = MAX_MOTOR_SPEED;
  }
  if (GamePad.isLeftPressed()) {
    rightMotorSpeed = -MAX_MOTOR_SPEED;
    leftMotorSpeed = -MAX_MOTOR_SPEED;
  }
  if (GamePad.isUpPressed()) {
    rightMotorSpeed = MAX_MOTOR_SPEED;
    leftMotorSpeed = -MAX_MOTOR_SPEED;
  }
  if (GamePad.isDownPressed()) {
    rightMotorSpeed = -MAX_MOTOR_SPEED;
    leftMotorSpeed = MAX_MOTOR_SPEED;
  }
  rotateMotor(rightMotorSpeed, leftMotorSpeed);

  // Servo control from Dabble
  if (GamePad.isCirclePressed()) {
    poslr = max(0, poslr - 1);
    dabbleServoControl = true;
    lastDabbleServoTime = millis();
  }
  if (GamePad.isSquarePressed()) {
    poslr = min(180, poslr + 1);
    dabbleServoControl = true;
    lastDabbleServoTime = millis();
  }
  if (GamePad.isCrossPressed()) {
    posud = min(180, posud + 1);
    dabbleServoControl = true;
    lastDabbleServoTime = millis();
  }
  if (GamePad.isTrianglePressed()) {
    posud = max(0, posud - 1);
    dabbleServoControl = true;
    lastDabbleServoTime = millis();
  }
  if (GamePad.isStartPressed()) {
    posud = 90;
    poslr = 90;
    dabbleServoControl = true;
    lastDabbleServoTime = millis();
  }

  // Check for timeout of Dabble servo control
  if (dabbleServoControl && (millis() - lastDabbleServoTime > dabbleServoTimeout)) {
    dabbleServoControl = false;
  }

  // Read Bluetooth data for servo control if Dabble is not actively controlling
  if (SerialBT.available() && !dabbleServoControl) {
    String data = SerialBT.readStringUntil('\n');
    if (data.startsWith("S") && data.length() == 7) {
      int newLrPos = data.substring(1, 4).toInt();
      int newUdPos = data.substring(4, 7).toInt();
      poslr = constrain(newLrPos, 0, 180);
      posud = constrain(newUdPos, 0, 180);
    }
  }
  

  if (dabbleServoControl) {
    if (currentPosud < posud) 
      currentPosud++;
    else if (currentPosud > posud) 
      currentPosud--;

    if (currentPoslr < poslr) 
      currentPoslr++;
    else if (currentPoslr > poslr) 
      currentPoslr--;

    udservo.write(180 - currentPosud);
    lrservo.write(currentPoslr);
    } 
    else {
      currentPosud = posud;
      currentPoslr = poslr;
      udservo.write(180 - posud);
      lrservo.write(poslr);
    }


  delay(1);
}