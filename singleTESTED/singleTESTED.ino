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
VL53L4CD sensor(&DEV_I2C, A1);

// Motor Driver A
const int ENA = 11;
const int IN1 = 12;
const int IN2 = 13;

// // Motor Driver B
// const int ENB = 10;
// const int IN3 = 9;
// const int IN4 = 8;

// Limit Switch(es)
const int PEGLIMA = 7;
const int TOPLIMA = 6;
const int BOTLIMA = 5;
// const int PEGLIMB = 4;
// const int TOPLIMB = 3;
// const int BOTLIMB = 2;

// Constant Variables
const int FOLLOW_DIST = 150; // [mm]; distance between the bar and ToF (on the peg)
const float DZ = 25; // [mm]; allowed tolerance movement in +/- dir
const int DANGER_DIST = 30; // [mm]; distance that will require emergency stop
const int MAXSPEED = 100; // [rpm]; PWM value for desired max speed 
const int MOTORDIAM = 10; // [mm]; shaft diameter of the motor
const int LINEARSPEED = MOTORDIAM*PI*(MAXSPEED/60); // [mm/s]; converting MAXSPEED angular vel to linear vel
const int RATE = 50; // [rpm]; acceleration rate for how quickly we want the motor to speed up
const int BRAKERATE = 25; // [rpm]; deceleration rate for slowing motor to stop
const int ESTOPRATE = 50; // [rpm]; deceleration rate for emergency stopping motor
const float MAXHEIGHT = 1.75; // [m]; maximum height for starting position
const float MINHEIGHT = 0.56; // [m]; minimum height for starting position
int speed = 0; // [rpm]; current operating speed

// ToF initialization/setup
void tofInit()
{
  SerialPort.println("Starting readout...");
  Wire.begin();
  sensor.begin();
  sensor.VL53L4CD_Off();
  sensor.InitSensor();
  sensor.VL53L4CD_SetRangeTiming(200, 0);
  sensor.VL53L4CD_StartRanging(); 
}

// Time of flight sensor output (includes serial monitor to keep spitting out the time of flight results per loop)
int tofOut()
{
  uint8_t NewDataReady = 0;
  VL53L4CD_Result_t results;
  uint8_t status;
  char report[64];

  do {
    status = sensor.VL53L4CD_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    sensor.VL53L4CD_ClearInterrupt();

    // Read measured distance; range status = 0 is valid
    sensor.VL53L4CD_GetResult(&results);
    snprintf(report, sizeof(report), "Status = %3u, Distance = %5u mm, Signal = %6u kcps/spad\r\n",
             results.range_status,
             results.distance_mm,
             results.signal_per_spad_kcps);
    SerialPort.print(report);
  }
  sensor.VL53L4CD_GetResult(&results);
  int distance = results.distance_mm;
  return distance;
}

void motorInit(){
  // Set all the motor control pins to outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  // pinMode(ENB, OUTPUT);
  // pinMode(IN3, OUTPUT);
  // pinMode(IN4, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  // digitalWrite(IN3, LOW);
  // digitalWrite(IN4, LOW);

  analogWrite(ENA, 0);  
  // analogWrite(ENB, 0);
}

void setup()
{
  SerialPort.begin(115200);
  SerialPort.println("Starting program...");
  motorInit();
  SerialPort.println("Starting motors...");

  tofInit();
}
 
void loop()
{
  int distance = tofOut();
  int error = FOLLOW_DIST - distance;
  int danger = DANGER_DIST - distance;
repeat:
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
  }

    if(distance < DANGER_DIST && distance >1)
  {
    // stop motor
    // delay
    // if the lim switch is pressed, move up
    // if the lim switch is not pressed, go to repeat
    speed=0;
    return;
  }

  if (speed > 0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }else if (speed < 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  analogWrite(ENA, abs(speed));
  delay(5);
}
