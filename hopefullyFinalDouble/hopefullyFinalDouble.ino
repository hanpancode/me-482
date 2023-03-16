// operation:
// set home position (run motor for x seconds until they both reach hard stop)
// goes to a set height (run motor for x seconds @ x speed)
// lift bar (initiate ToF)
// ToF > x (bar is moving away, peg goes up)
// ToF < x (bar is moving toward, peg goes down)
// ToF = x for x seconds (stagnant, stop and run motor upwards to assist)
// ToF = y (y << x, failsafe)
// limit switch on peg is actuated (force stop and lock)


#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#define DEV_I2C Wire
#define SerialPort Serial

// Components.
VL53L4CD sensor_left(&DEV_I2C, A1);
VL53L4CD sensor_right(&DEV_I2C, A2);

//distance global variables
int distance_right = 0;
int distance_left = 0;

// Motor Driver
//left motor
const int ENA = 11;
const int IN1 = 12;
const int IN2 = 13;

//right motor
const int ENB = 10;
const int IN3 = 9;
const int IN4 = 8;

// Limit Switch(es)
//left switches
const int TOPLIMA = 7;
const int PEGLIMA = 6;
const int BOTLIMA = 5;
const int BOTSTOPA = 14;
const int TOPSTOPA = 17;

//right switches
const int PEGLIMB = 3;
const int TOPLIMB = 4;
const int BOTLIMB = 2;
const int BOTSTOPB = 18;
const int TOPSTOPB = 19;

// Constant Variables
const int FOLLOW_DIST = 100; // [mm]; distance between the bar and ToF (on the peg)
const float DZ = 25; // [mm]; allowed tolerance movement in +/- dir
const int DANGER_DIST = 2; // [mm]; distance that will require emergency stop
const int MAXSPEED = 50; // [rpm]; PWM value for desired max speed 
const int MOTORDIAM = 10; // [mm]; shaft diameter of the motor
const int LINEARSPEED = MOTORDIAM*PI*(MAXSPEED/60); // [mm/s]; converting MAXSPEED angular vel to linear vel
const int RATE = 30; // [rpm]; acceleration rate for how quickly we want the motor to speed up
const int BRAKERATE = 10; // [rpm]; deceleration rate for slowing motor to stop
const int ESTOPRATE = 50; // [rpm]; deceleration rate for emergency stopping motor
const float MAXHEIGHT = 1.75; // [m]; maximum height for starting position
const float MINHEIGHT = 0.56; // [m]; minimum height for starting position
const unsigned long stagDuration = 10000;
int speed = 0; // [rpm]; current operating speed
uint8_t NewDataReady = 0;
int sync_req = 70; //dist in mm that considers motors out of sync
int speed_target = 80;
int accel_target = 0;
int loop_time = 15/1000;
int break_target = 0;
int decel_target = 0;
// system setup as user initiates exercise
void setup(){
  
  // Initialize serial for output.
  SerialPort.begin(115200);
  SerialPort.println("Starting...");
  // Initialize I2C bus.
  DEV_I2C.begin();
  // Configure VL53L4CD satellite component.
  sensor_left.begin();
  sensor_right.begin();
  // Switch off VL53L4CD satellite component.
  sensor_left.VL53L4CD_Off();
  sensor_right.VL53L4CD_Off();
  //Initialize VL53L4CD satellite component.
  sensor_left.InitSensor();
  sensor_right.InitSensor();
  // Program the highest possible TimingBudget, without enabling the
  // low power mode. This should give the best accuracy
  sensor_left.VL53L4CD_SetRangeTiming(200, 0);
  sensor_right.VL53L4CD_SetRangeTiming(200, 0);
  // Start Measurements
  sensor_left.VL53L4CD_StartRanging();
  sensor_right.VL53L4CD_StartRanging();

  SerialPort.println("Starting reading...");
  //initialize both motors on the driver
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 0);  
  analogWrite(ENB, 0);

  SerialPort.println("Starting motors...");

  /*
  //startup movement to ease motor surging upward with influx of power

  while (digitalRead(PEGLIMA) == LOW || digitalRead(PEGLIMB) == LOW){

    //startup movement to ease motor surging upward with influx of power
    speed = 50;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, abs(speed));
    analogWrite(ENB, abs(speed));
    delay(5);
    speed = 0;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, abs(speed));
    analogWrite(ENB, abs(speed));
    delay(1);

  //motor sync to get pegs in line before squat
  
  }
  */

  while (digitalRead(PEGLIMA) == LOW || digitalRead(PEGLIMB) == LOW){
    delay(10); // Waiting as long as the limit switch is pressed; continue with code when limit switch releases
  }
}
void loop()
{
  NewDataReady = 0;
  VL53L4CD_Result_t results_l;
  uint8_t status_l = 0;
  
  char report_l[64];
  
  do {
    status_l = sensor_left.VL53L4CD_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  if ((!status_l) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    sensor_left.VL53L4CD_ClearInterrupt();

    // Read measured distance. RangeStatus = 0 means valid data
    sensor_left.VL53L4CD_GetResult(&results_l);
    snprintf(report_l, sizeof(report_l), "Status = %3u, Distance Left = %5u mm, Signal = %6u kcps/spad\r\n",
             results_l.range_status,
             results_l.distance_mm,
             results_l.signal_per_spad_kcps);
    SerialPort.println(report_l);
    distance_left = results_l.distance_mm;
  }
  delay(5);

  NewDataReady = 0;
  VL53L4CD_Result_t results_r;
  uint8_t status_r = 0;
  char report_r[64];
do {
    status_r = sensor_right.VL53L4CD_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  if ((!status_r) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    sensor_right.VL53L4CD_ClearInterrupt();

    // Read measured distance. RangeStatus = 0 means valid data
    sensor_right.VL53L4CD_GetResult(&results_r);
    snprintf(report_r, sizeof(report_r), "Status = %3u, Distance Right = %5u mm, Signal = %6u kcps/spad\r\n",
             results_r.range_status,
             results_r.distance_mm,
             results_r.signal_per_spad_kcps);
    SerialPort.println(report_r);
    distance_right = results_r.distance_mm;
  }
  delay(5);
//add code for extra switches here

  int avg_dist = (distance_right + distance_left)/2;
  int diff_dist = distance_right - distance_left;
  int error_left = FOLLOW_DIST - distance_left;
  int error_right = FOLLOW_DIST - distance_right;
  int error;
  //int danger = DANGER_DIST - distance;
  while (digitalRead(BOTLIMA) == HIGH && digitalRead(TOPLIMA) == HIGH && digitalRead(BOTLIMB) == HIGH && digitalRead(TOPLIMB) == HIGH){
    if (error_left < 0 && error_right < 0){
      // case for up ?
      error = min(error_right, error_left);
    }
    else if (error_left > 0 && error_right > 0){
      // case for down ?
      error = max (error_right, error_left);
    }
  //bar is slightly tilted pegs are sent down to avoid interference
    else if (error_left > 0 && error_right < 0 && diff_dist < sync_req){
      error = error_right;
    }
    else if (error_left < 0 && error_right > 0 && diff_dist < sync_req){
      error = error_left;
    }
    while (digitalRead(TOPLIMA) == LOW || digitalRead(TOPLIMB) == LOW){
      speed = -(error/DZ)*speed_target; 
      SerialPort.println("Top Reached");
    }
    while (digitalRead(BOTLIMA) == LOW || digitalRead(BOTLIMB) == LOW){
      speed = (error/DZ)*speed_target;
      SerialPort.println("Bottom Reached");
    }  
    while (digitalRead(PEGLIMA) == LOW || digitalRead(PEGLIMB) == LOW){
      speed = 0;
      SerialPort.println("Implement lift assist");
    } 
  //motor syncing
    if (diff_dist > sync_req){
      if (error_right > error_left){
        //move left motor up
        speed = 50;
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, abs(speed));
        delay(diff_dist/(speed/60000)+5);
        speed = 0;
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, abs(speed));
      }
      else {
        //move right motor up
        speed = 50;
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, abs(speed));
        delay(diff_dist/(speed/60000));
        speed = 0;
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, abs(speed));
      }
    }
  
//speed control for normal case
if (error > DZ){
	if (speed > MAXSPEED){
		speed = MAXSPEED;
	}
	else {
		speed = (error/DZ)*speed_target;
	}
}
else if (error < -DZ){
	if (speed < -MAXSPEED){
		speed = -MAXSPEED;
	}
	else {
		speed = (error/-DZ)*speed_target;
	}
}
else {
	speed = 0;
}
//speed control for dangerously low case
    if (distance_left < DANGER_DIST || distance_right < DANGER_DIST){
      speed = speed_target * (error/DANGER_DIST);
    }
//speed control for deadzone    
    else if (error > -DZ && error < DZ){ 
      // If bar is moving up     
      if (speed > 0){
        // Decelerate up
        //speed = (error/DZ)*speed;
        speed -= BRAKERATE;
      }
      // If bar is moving down
      else if (speed < 0){
        // Decelerate down
        //speed = (error/DZ)*speed;
        speed += BRAKERATE;
      }
      
      // If bar is not moving 
      /*else{
        // Remain still
        speed = 0;
        // Start timer to check for struggle
        unsigned long stagTime = millis();
        // As long as it remains in the deadzone and timer for still bar is less than stagnant duration
        while ((error > -DZ && error < DZ) && stagTime < stagDuration)
        {
          // Keep checking the sensor distance while timer is counting
          error = max (error_left, error_right);
        }
        // If the timer reaches the duration limit
        if (stagTime > stagDuration){
          // Accelerate upwards in order to help user finish rep
          if (speed <= MAXSPEED){
            speed += RATE;
            // Keep going up until it hits the limit switch
            while (digitalRead(TOPLIMA)== LOW || digitalRead(TOPLIMB) == LOW){
            // Once the limit switch is hit, stop the system
            speed=0;
	}
            return;
          }
        // If the timer does not reach duration limit, and it's outside of deadzone
        } 
        else {
          // Continue with set by following and checking for distance value again
        }  
      }*/
    }

/*
if (speed > 0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }else if (speed < 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENA, abs(speed));
  analogWrite(ENB, abs(speed));
  delay(5);
  */
}
}

