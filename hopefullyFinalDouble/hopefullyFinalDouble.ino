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

//#ifndef LED_BUILTIN
//  #define LED_BUILTIN 13
//#endif
//#define LedPin LED_BUILTIN

// Components.
VL53L4CD sensor_left(&DEV_I2C, A1);
VL53L4CD sensor_right(&DEV_I2C, A2);

//distance global variables
int distance_right = 0;
int distance_left = 0;

// Motor Driver A
const int ENA = 11;
const int IN1 = 12;
const int IN2 = 13;

// Motor Driver B
const int ENB = 10;
const int IN3 = 9;
const int IN4 = 8;

// Limit Switch(es)
//const int TOPLIMA = 7;
//const int PEGLIMA = 6;
//const int BOTLIMA = 5;
// const int PEGLIMB = 4;
// const int TOPLIMB = 3;
// const int BOTLIMB = 2;

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
int speed = 0; // [rpm]; current operating speed
uint8_t NewDataReady = 0;

/* Setup ---------------------------------------------------------------------*/




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

  //Led on
  

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

  int avg_dist = (distance_right + distance_left)/2;
  int diff_dist = distance_right - distance_left;
  int error_left = FOLLOW_DIST - distance_left;
  int error_right = FOLLOW_DIST - distance_right;
  int error;
  //int danger = DANGER_DIST - distance;

  if (error_left < 0 && error_right < 0){
    // case for up ?
    error = min(error_right, error_left);
  }
  else if (error_left > 0 && error_right > 0){
    // case for down ?
    error = max (error_right, error_left);
  }
// !! these next 2 cases are when the bar is slightly tilted !!  
// !! i think the best way is to send it down in this case to avoid interference of the peg lmk what u think
  else if (error_left > 0 && error_right < 0){
    error = error_right;
  }
  else if (error_left < 0 && error_right > 0){
    error = error_left;
  }
  //SerialPort.println("average distance = " + avg_dist);
  //int danger = DANGER_DIST - distance;
  
  if (error > DZ){
    if (speed <= MAXSPEED){
      speed += RATE;
      //SerialPort.println("speed increasing upward to = " + speed);
    }
  }else if (error < -DZ){
    if (speed >= -MAXSPEED){
      speed -= RATE;
      //SerialPort.println("speed increasing downward to = " + speed);      
    }
  }else if (error > -DZ && error < DZ){
    if (speed > 0){
      speed -= (BRAKERATE)/2;
      //SerialPort.println("speed decreasing upward to = " + speed);  
    }else if (speed < 0){
      speed += BRAKERATE;
      //SerialPort.println("speed decreasing upward to = " + speed);  
    }else{
      speed = 0;
      //SerialPort.println("speed is zero");  
    }   
  }else if (distance_left < DANGER_DIST || distance_right < DANGER_DIST){
    speed = 0;
    //SerialPort.println("danger indicated, speed is zero");  
  }
  // else if (danger < 0){
  //   if (speed > 0){
  //     speed -= ESTOPRATE;
  //   }else if (speed < 0){
  //     speed += ESTOPRATE;
  //   }else{
  //     speed = 0;
  //     return;
  //   }   
  // }

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
}