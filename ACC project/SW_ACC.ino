#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>

// Define ultrasonic sensor pins
const int trigPin = 2;
const int echoPin = 4;

// Define motor control pins
const int leftMotorPin1 = 3;
const int leftMotorPin2 = 5;
const int rightMotorPin1 = 6;
const int rightMotorPin2 = 9;

// Define LED pin
const int ledPin1 = 7;
const int ledPin2 = 8;

// Define PID parameters
double Setpoint = 20; // Setpoint distance (20 cm)
double Input, Output;
double Kp = 1, Ki = 0, Kd = 0; // PID constants
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Define motor speeds
int baseSpeed = 100;
int leftMotorSpeed, rightMotorSpeed;

// Define distance threshold for stopping
const int stopDistance = 10; // 10 cm

// Define LCD address
#define LCD_ADDR 0x27 // Address of I2C LCD

// Initialize LCD
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize motor control pins
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  // Initialize LED pin
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Set initial motor speeds
  analogWrite(leftMotorPin1, 0);
  analogWrite(leftMotorPin2, 0);
  analogWrite(rightMotorPin1, 0);
  analogWrite(rightMotorPin2, 0);

  // Set up PID
  pid.SetMode(AUTOMATIC);

  // Initialize LCD
  lcd.init();                      // Initialize the lcd 
  lcd.backlight();                 // Turn on the backlight
  lcd.clear();                     // Clear the display

  // Print initial message on LCD
  lcd.setCursor(0, 0);
  lcd.print("Distance: ");
}

void loop() {
  // Read distance from ultrasonic sensor
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  // Print distance
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Update distance on LCD
  lcd.setCursor(10, 0);
  lcd.print("   "); // Clear previous distance
  lcd.setCursor(10, 0);
  lcd.print(distance);
  lcd.print(" cm");

  // Check conditions and adjust motor speeds accordingly
  if (distance <= stopDistance) { // Condition 3: Stop if object is too close
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
    digitalWrite(ledPin1, HIGH); // Turn on LED when car stops
    digitalWrite(ledPin2, HIGH); // Turn on LED when car stops
  } else if (distance > Setpoint) { // Condition 2: Maintain normal speed if distance is large
    Input = distance;
    pid.Compute();
    leftMotorSpeed = baseSpeed + Output;
    rightMotorSpeed = baseSpeed - Output;
    digitalWrite(ledPin1, LOW); // Turn off LED if not stopped
    digitalWrite(ledPin2, LOW); // Turn off LED if not stopped
  } else { // Condition 1: Decelerate if object is closer than the setpoint distance
    leftMotorSpeed = baseSpeed / 2;
    rightMotorSpeed = baseSpeed / 2;
    digitalWrite(ledPin1, LOW); // Turn off LED if not stopped
    digitalWrite(ledPin2, LOW); // Turn off LED if not stopped
  }

  // Ensure motor speeds are within bounds
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  // Apply motor speeds
  analogWrite(leftMotorPin1, leftMotorSpeed);
  analogWrite(leftMotorPin2, 0);
  analogWrite(rightMotorPin1, rightMotorSpeed);
  analogWrite(rightMotorPin2, 0);

  // Delay for stability
  delay(100);
}
