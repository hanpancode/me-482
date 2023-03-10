#include <Wire.h>
//#include <VL53L4CX.h>
#include <vl53l4cd_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>


#define DEV_I2C Wire
#define SerialPort Serial

// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;

void setup() {
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Turn off motor A - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void loop() {
  directionControl();
  delay(1000);
  speedControl();
  delay(1000);
}

// This function lets you control spinning direction of motor A
void directionControl() {
  // Set motor A to maximum speed
  // For PWM maximum possible values are 0 to 255
  analogWrite(enA, 255);

  // Turn on motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(2000);
  
  // Now change motor A direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  delay(2000);
  
  // Turn off motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

// This function lets you control speed of motor A
void speedControl() {
  // Turn on motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  // Accelerate from zero to maximum speed
  for (int i = 0; i < 256; i++) {
    analogWrite(enA, i);
    delay(20);
  }
  
  // Decelerate from maximum speed to zero
  for (int i = 255; i >= 0; --i) {
    analogWrite(enA, i);
    delay(20);
  }
  
  // Now turn off motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}
