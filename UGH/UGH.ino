// operation:
// set home position (run motor for x seconds until they both reach hard stop)
// goes to a set height (run motor for x seconds @ x speed)
// lift bar (initiate ToF)
// ToF > x (bar is moving away, peg goes up)
// ToF < x (bar is moving toward, peg goes down)
// ToF = x for x seconds (stagnant, stop and run motor upwards to assist)
// ToF = y (y << x, failsafe)
// limit switch on peg is actuated (force stop and lock)

#include <Wire.h>
#include <vl53l4cd_class.h>
//#include <VL53L4CX.h>
// VL53L4CX import also works; different firmware, but basically same model, code is universal
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#define DEV_I2C Wire
#define SerialPort Serial

// Different firmware, but basically same model; comment out either, both still named sensor
// VL53L4CX sensor(&DEV_I2C, A1);
VL53L4CD sensor_left(&DEV_I2C, A1);
//!!added another sensor (one left and one right, idk how to correctly choose the pin for it based on the 2 parameters below alex pls help)!!
VL53L4CD sensor_right(&DEV_I2C, A2);

// Motor Driver A
const int ENA = 11;
const int IN1 = 12;
const int IN2 = 13;

// Motor Driver B
const int ENB = 10;
const int IN3 = 9;
const int IN4 = 8;

// Limit Switch(es)
const int TOPLIMA = 7;
const int PEGLIMA = 6;
const int BOTLIMA = 5;
// const int PEGLIMB = 4;
// const int TOPLIMB = 3;
// const int BOTLIMB = 2;


// Constant Variables
const int FOLLOW_DIST = 100; // [mm]; distance between the bar and ToF (on the peg)
const float DZ = 25; // [mm]; allowed tolerance movement in +/- dir
const int DANGER_DIST = 30; // [mm]; distance that will require emergency stop
const int MAXSPEED = 150; // [rpm]; PWM value for desired max speed 
const int MOTORDIAM = 10; // [mm]; shaft diameter of the motor
const int LINEARSPEED = MOTORDIAM*PI*(MAXSPEED/60); // [mm/s]; converting MAXSPEED angular vel to linear vel
const int RATE = 50; // [rpm]; acceleration rate for how quickly we want the motor to speed up
const int BRAKERATE = 25; // [rpm]; deceleration rate for slowing motor to stop
const int ESTOPRATE = 50; // [rpm]; deceleration rate for emergency stopping motor
const float MAXHEIGHT = 1.75; // [m]; maximum height for starting position
const float MINHEIGHT = 0.56; // [m]; minimum height for starting position
int speed = 0; // [rpm]; current operating speed

// ToF initialization/
void setup()
{
  SerialPort.begin(115200);
  SerialPort.println("Starting...");
  Wire.begin();
  sensor_left.begin();
  sensor_right.begin();
  sensor_left.VL53L4CD_Off();
  sensor_right.VL53L4CD_Off();
  sensor_left.InitSensor();
  sensor_right.InitSensor();
  sensor_left.VL53L4CD_SetRangeTiming(200, 0);
  sensor_right.VL53L4CD_SetRangeTiming(200, 0);
  sensor_left.VL53L4CD_StartRanging(); 
  sensor_right.VL53L4CD_StartRanging(); 
}
void motorInit(){
  // Set all the motor control pins to outputs
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
}
// Time of flight sensor output (includes serial monitor to keep spitting out the time of flight results per )
//!!idk what to do about this section yikes!!
int tofOut_left()
{
  uint8_t NewDataReady = 0;
  VL53L4CD_Result_t results;
  uint8_t status;
  char report[64];

  do {
    status = sensor_left.VL53L4CD_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    sensor_left.VL53L4CD_ClearInterrupt();

    // Read measured distance; range status = 0 is valid
    sensor_left.VL53L4CD_GetResult(&results);
    snprintf(report, sizeof(report), "Status = %3u, Distance = %5u mm, Signal = %6u kcps/spad\r\n",
             results.range_status,
             results.distance_mm,
             results.signal_per_spad_kcps);
    SerialPort.print(report);
  }
  sensor_left.VL53L4CD_GetResult(&results);
  int distance_left = results.distance_mm;
  return distance_left;
}
int tofOut_right()
{
  uint8_t NewDataReady = 0;
  VL53L4CD_Result_t results_r;
  uint8_t status_r;
  char report[64];
// !!this is just copied from the last one because i don't think i can just put in like before!!
  do {
    status_r = sensor_right.VL53L4CD_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  if ((!status_r) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    sensor_right.VL53L4CD_ClearInterrupt();

    // Read measured distance; range status = 0 is valid
    sensor_right.VL53L4CD_GetResult(&results_r);
    snprintf(report, sizeof(report), "Status = %3u, Distance = %5u mm, Signal = %6u kcps/spad\r\n",
             results_r.range_status,
             results_r.distance_mm,
             results_r.signal_per_spad_kcps);
    SerialPort.print(report);
  }
  sensor_right.VL53L4CD_GetResult(&results_r);
  int distance_right = results_r.distance_mm;
  return distance_right;
}
void loop()
{
  tofOut_left();
  tofOut_right();
  int distance_left = tofOut_left();
  int distance_right = tofOut_right();

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
  // !! final case is for when bar is tilted to dangerous degree !!
  // !! 100 is random value we will discuss !!
  //else if (error_left < 0 && error_right > 0 && abs(error_left - error_right) > 100){
    // !! danger oh no !!
    // !! i don't think this is actually needed because it is accounted for later !!
  //}



  if (error > DZ){
    if (speed <= MAXSPEED){
      speed += RATE;
    }
  }else if (error < -DZ){
    if (speed >= -MAXSPEED){
      speed -= RATE;
    }
  }else if (error > -DZ && error < DZ){
    if (speed > 0){
      speed -= BRAKERATE;
    }else if (speed < 0){
      speed += BRAKERATE;
    }else{
      speed = 0;
    }   
  }else if (distance_left < DANGER_DIST || distance_right < DANGER_DIST){
    speed = 0;
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
  delay(5);
}

