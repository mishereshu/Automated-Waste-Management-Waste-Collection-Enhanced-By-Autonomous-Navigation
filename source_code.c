//color sensor initialization
int red = 0;  
int green = 0;  
int blue = 0;  

#define IR_SENSOR_RIGHT 11
#define IR_SENSOR_LEFT 12
#define MOTOR_SPEED 150

const int TRIG_PIN = 30; 
const int ECHO_PIN1 = 28; 
const int LED_PIN  = 22; 
const int DISTANCE_THRESHOLD = 7; 


float duration_us, distance_cm;

// Right motor
int enableRightMotor = 6;
int rightMotorPin1 = 7;
int rightMotorPin2 = 8;

// Left motor
int enableLeftMotor = 5;
int leftMotorPin1 = 9;
int leftMotorPin2 = 10;

// Ultrasonic sensor
#define TRIGGER_PIN 3
#define ECHO_PIN 4
#define MAX_DISTANCE 200 // Reduced maximum distance for testing

#include <NewPing.h>

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void stopMotors() {
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
}

void setup() {
  Serial.begin(750); // Initialize serial communication
  // color sensor pin config
  pinMode(32, OUTPUT);  
  pinMode(33, OUTPUT);  
  pinMode(34, OUTPUT);  
  pinMode(35, OUTPUT);  
  pinMode(36, INPUT);  
  
  digitalWrite(32, HIGH);  
  digitalWrite(33, HIGH); 

  pinMode(TRIG_PIN, OUTPUT); 
  pinMode(ECHO_PIN1, INPUT);   
  pinMode(LED_PIN, OUTPUT); 
  Serial.println("Setup complete.");
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
}

void loop() {
  //color sensor debug config 
  digitalWrite(34, LOW);  
  digitalWrite(35, LOW);  
  red = pulseIn(36, digitalRead(36) == HIGH ? LOW : HIGH);  
  digitalWrite(35, HIGH);  
  blue = pulseIn(36, digitalRead(36) == HIGH ? LOW : HIGH);  
  digitalWrite(34, HIGH);  
  green = pulseIn(36, digitalRead(36) == HIGH ? LOW : HIGH);   
 
  if (red < blue && red < green && red < 15)
  {  
    Serial.println("Red Color Detected");  
  }  
  else if (blue < red && blue < green)   
  {  
    Serial.println("Blue Color Detected");  
  }  
  else if (green < red && green < blue)  
  {  
    Serial.println("Green Color Detected");  
  }  
  
  delay(500);   


   digitalWrite(TRIG_PIN, HIGH);//for level sensing
   delayMicroseconds(10);
   digitalWrite(TRIG_PIN, LOW);

  
  duration_us = pulseIn(ECHO_PIN1, HIGH);
  
  distance_cm = 0.017 * duration_us;

  if(distance_cm < DISTANCE_THRESHOLD)
    digitalWrite(LED_PIN, HIGH); 
  else
    digitalWrite(LED_PIN, LOW);  

 
  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  delay(500);

  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  // Read distance from ultrasonic sensor
  int distance = sonar.ping_cm();

  // Print distance to serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Rest of the code
  // Stop the motors if an object is detected within 10 cm
  if (distance > 0 && distance <= 30 || (red < blue && red < green && red < 15)) {
    Serial.println("Collision detected!");
    stopMotors(); // Stop the motors
  } else {
    // If none of the sensors detects a black line, then go straight
    if (rightIRSensorValue == LOW && leftIRSensorValue == LOW) {
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
    }
    // If right sensor detects a black line, then turn right
    else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW) {
      stopMotors(); // Stop the motors
      rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
    }
    // If left sensor detects a black line, then turn left  
    else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH) {
      stopMotors(); // Stop the motors
      rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
    }
    // If both sensors detect a black line, then stop 
    else {
      stopMotors(); // Stop the motors
      rotateMotor(0, 0);
    }
  }
}

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
    digitalWrite(leftMotorPin2, LOW);
  }
  
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}
