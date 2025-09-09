/*
  Differential-Drive Robot ROS Node

  This Arduino node performs the following functions:

  1. Reads wheel encoders to calculate odometry (x, y, theta) of the robot.
  2. Reads MPU6050 gyro to estimate robot orientation (yaw).
  3. Applies FIR filtering to wheel velocities to reduce noise.
  4. Publishes robot pose and filtered wheel velocities to ROS topics:
     - "robot_pos" (geometry_msgs::Point)
     - "velocities" (geometry_msgs::Point)
  5. Subscribes to PWM commands from ROS ("motor_pwm") to drive motors.
  6. Controls a servo motor to capture images based on ROS commands ("image").
  
  Hardware connections:
  - Two DC motors with encoders (pins defined below)
  - MPU6050 IMU for orientation
  - Servo motor on pin 7
*/

#include <Wire.h>
#include <MPU6050_light.h>
#include <ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int64.h>
#include <Arduino.h>
#include <Servo.h>

// ==============================
// Robot Parameters
// ==============================
const float WHEEL_RADIUS = 0.0335;  // meters
const float WHEEL_BASE = 0.181;     // meters
const int TICKS_PER_REV = 190;      // encoder ticks per wheel revolution

// ==============================
// Encoder Pins
// ==============================
const int L_A = 3, L_B = 2;
const int R_A = 18, R_B = 19;

// ==============================
// Motor Control Pins
// ==============================
const int ENA = 10, IN1 = 8, IN2 = 9;
const int ENB = 5,  IN3 = 11, IN4 = 12;

// ==============================
// Encoder Counters
// ==============================
volatile long leftTicks = 0, rightTicks = 0;
long prevLeftTicks = 0, prevRightTicks = 0;

// ==============================
// Robot Pose
// ==============================
float x = 0, y = 0;
float theta = 0; // radians

// ==============================
// MPU6050 and Orientation Filter
// ==============================
MPU6050 mpu(Wire);
float yaw = 0, yawRate = 0;
float alpha = 0.98; // complementary filter coefficient
float rapidChangeThreshold = 100.0;
bool isRapidChange = false;
unsigned long rapidChangeStartTime = 0;
const unsigned long rapidChangeMaxDuration = 1000;
unsigned long prevTime = 0;

// ==============================
// FIR Filter for wheel velocities
// ==============================
#define FILTER_LEN 43
double filter[FILTER_LEN] = {
  0.0000119596432413, 0.0000480139397014, 0.0001375046888079, 0.0003251890000293,
  0.0006762558885572, 0.0012775572985148, 0.0022357564563538, 0.0036713053047777,
  0.0057077825462884, 0.0084569610996958, 0.0120008287066771, 0.0163726769008519,
  0.0215400553470459, 0.0273926023208945, 0.0337375666561611, 0.0403050236279273,
  0.0467635094453458, 0.0527452755672683, 0.0578787525536911, 0.0618244850678606,
  0.0643099873837576, 0.0651588406343320, 0.0643099873837576, 0.0618244850678606,
  0.0578787525536911, 0.0527452755672683, 0.0467635094453458, 0.0403050236279273,
  0.0337375666561611, 0.0273926023208945, 0.0215400553470459, 0.0163726769008519,
  0.0120008287066771, 0.0084569610996958, 0.0057077825462884, 0.0036713053047777,
  0.0022357564563538, 0.0012775572985148, 0.0006762558885572, 0.0003251890000293,
  0.0001375046888079, 0.0000480139397014, 0.0000119596432413
};
float bufferLeft[FILTER_LEN] = {0};
float bufferRight[FILTER_LEN] = {0};
int bufferIndexLeft = 0, bufferIndexRight = 0;

float filterSignal(float newSample, float* buffer, int& bufferIndex) {
  buffer[bufferIndex] = newSample;
  double result = 0.0;
  int idx = bufferIndex;
  for (int i = 0; i < FILTER_LEN; i++) {
    result += filter[i] * buffer[idx];
    idx = (idx - 1 + FILTER_LEN) % FILTER_LEN;
  }
  bufferIndex = (bufferIndex + 1) % FILTER_LEN;
  return result;
}

// ==============================
// ROS Node Setup
// ==============================
ros::NodeHandle nh;
geometry_msgs::Point pos_msg;
ros::Publisher pos_pub("robot_pos", &pos_msg);
geometry_msgs::Point vel_msg;
ros::Publisher vel_pub("velocities", &vel_msg);

// ==============================
// Motor PWM Subscriber
// ==============================
void pwmCallback(const geometry_msgs::Point& msg) {
  int pwm1 = constrain((int)msg.x, -255, 255);
  int pwm2 = constrain((int)msg.y, -255, 255);

  // Left motor
  if (pwm1 >= 0) { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); } 
  else { digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH); pwm1=-pwm1; }
  analogWrite(ENA, pwm1);

  // Right motor
  if (pwm2 >= 0) { digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); } 
  else { digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH); pwm2=-pwm2; }
  analogWrite(ENB, pwm2);
}
ros::Subscriber<geometry_msgs::Point> pwm_sub("motor_pwm", pwmCallback);

// ==============================
// Servo Control Subscriber
// ==============================
Servo imageServo;
int currentServoAngle = 90;

void imageCallback(const std_msgs::Int64& msg) {
  int targetAngle = constrain((int)msg.data, 0, 180);
  if (targetAngle == currentServoAngle) return;

  int step = (targetAngle > currentServoAngle) ? 10 : -10;
  for (int angle = currentServoAngle + step; (step>0)?angle<=targetAngle:angle>=targetAngle; angle+=step) {
    imageServo.write(angle);
    delay(15);
  }
  currentServoAngle = targetAngle;
}
ros::Subscriber<std_msgs::Int64> image_sub("image", imageCallback);

// ==============================
// Encoder ISRs
// ==============================
void leftEncoderISR()  { leftTicks  += (digitalRead(L_A)==digitalRead(L_B)) ? 1 : -1; }
void rightEncoderISR() { rightTicks += (digitalRead(R_A)==digitalRead(R_B)) ? 1 : -1; }

// ==============================
// Utility
// ==============================
float normalizeAngle(float angle) {
  while (angle < -180) angle += 360;
  while (angle >= 180) angle -= 360;
  return angle;
}

// ==============================
// Arduino Setup
// ==============================
void setup() {
  // Motor pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Encoder pins
  pinMode(L_A, INPUT); pinMode(L_B, INPUT);
  pinMode(R_A, INPUT); pinMode(R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(L_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_A), rightEncoderISR, CHANGE);

  // MPU6050 initialization
  Wire.begin();
  while (mpu.begin() != 0) delay(1000);
  mpu.calcGyroOffsets();
  mpu.update();
  prevTime = millis();

  // Servo setup
  imageServo.attach(7);
  imageServo.write(currentServoAngle);

  // ROS setup
  nh.initNode();
  nh.advertise(pos_pub);
  nh.advertise(vel_pub);
  nh.subscribe(pwm_sub);
  nh.subscribe(image_sub);
}

// ==============================
// Main Loop
// ==============================
void loop() {
  mpu.update();
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime)/1000.0;
  prevTime = currentTime;

  // Update yaw & complementary filter
  yawRate = mpu.getGy

