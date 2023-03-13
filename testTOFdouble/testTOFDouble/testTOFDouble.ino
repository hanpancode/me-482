/* Includes ------------------------------------------------------------------*/
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
VL53L4CD left(&DEV_I2C, A1);
VL53L4CD right(&DEV_I2C, A2);

/* Setup ---------------------------------------------------------------------*/

void setup()
{


  // Initialize serial for output.
  SerialPort.begin(115200);
  SerialPort.println("Starting...");

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L4CD satellite component.
  left.begin();
  right.begin();
  // Switch off VL53L4CD satellite component.
  left.VL53L4CD_Off();
  right.VL53L4CD_Off();
  //Initialize VL53L4CD satellite component.
  left.InitSensor();
  right.InitSensor();
  // Program the highest possible TimingBudget, without enabling the
  // low power mode. This should give the best accuracy
  left.VL53L4CD_SetRangeTiming(200, 0);
  right.VL53L4CD_SetRangeTiming(200, 0);
  // Start Measurements
  left.VL53L4CD_StartRanging();
  right.VL53L4CD_StartRanging();
}

void loop()
{
  uint8_t NewDataReady = 0;
  VL53L4CD_Result_t results_l;
  uint8_t status_l;

  char report_l[64];

  do {
    status_l = left.VL53L4CD_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  //Led on


  if ((!status_l) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    left.VL53L4CD_ClearInterrupt();

    // Read measured distance. RangeStatus = 0 means valid data
    left.VL53L4CD_GetResult(&results_l);
    snprintf(report_l, sizeof(report_l), "Status = %3u, Distance Left = %5u mm, Signal = %6u kcps/spad\r\n",
             results_l.range_status,
             results_l.distance_mm,
             results_l.signal_per_spad_kcps);
    SerialPort.print(report_l);
  }
  delay(5);

  //uint8_t NewDataReady = 0;
  VL53L4CD_Result_t results_r;
  uint8_t status_r;
  char report_r[64];
do {
    status_r = right.VL53L4CD_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  //Led on


  if ((!status_r) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    right.VL53L4CD_ClearInterrupt();

    // Read measured distance. RangeStatus = 0 means valid data
    right.VL53L4CD_GetResult(&results_r);
    snprintf(report_r, sizeof(report_r), "Status = %3u, Distance Right = %5u mm, Signal = %6u kcps/spad\r\n",
             results_r.range_status,
             results_r.distance_mm,
             results_r.signal_per_spad_kcps);
    SerialPort.print(report_r);
  }
  delay(5);
}