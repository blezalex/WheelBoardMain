#pragma once
#include "stm32f10x.h"
#include "arduino.h"

//#define REV5

//***** Apply board orientation transform *** //
// IMU is using right hand coordinate system, it tracks gravity vector and calculates angle between gravity vector and the board frame.
// Normal orientation - X forward, Y right Z down

#define BOARD_ROTATION_MACRO BOARD_ROTATION_UPSIDE_DOWN_X

#define BOARD_ROTATION_UPSIDE_DOWN_X(XYZ) XYZ[1]*=-1; XYZ[2]*=-1;  // rotated 180 deg around X axis

//****** pad sensors ****//
#define MIN_PAD_LEVEL 3700 // pad is considered pressed if ADC reads above this value. ADC reads 0 - 4096.
#define PAD_NO_CONNECT_THR 40 // A crude filter for pad readings. Number of consecutive 'no pressed' reads before pad is marked not pressed.


// ****** start mode settings *********/
#define START_MAX_POWER 300 // [-START_MAX_POWER: +START_MAX_POWER] out of MOTOR_CMD_RANGE
#define START_DURATION 1000 // in ms. Time to bring board from tilted to balanced.

#define START_ANGLE_DRIVE 9
#define START_ANGLE_DRIVE_FULL 6
#define START_D_MAX_MULTIPLIER 2

#define START_ANGLE_STEER 8

#define STOP_ANGLE_DRIVE 14
#define STOP_ANGLE_STEER 40

#define MOTOR_OUT_AVG_RC 0.008
#define POWER_OUT_WARNING_THRESHOLD 0.75 // warn when running at x% or more throttle


#define MAX_CHANGE_SINGLE_LOOP 300

#define MIN_MOTOR_CMD 990
#define MAX_MOTOR_CMD 2010
#define NEUTRAL_MOTOR_CMD 1500

#define MOTOR_CMD_RANGE (MAX_MOTOR_CMD - NEUTRAL_MOTOR_CMD)

#define ANGLE_DRIVE 1
#define ANGLE_STEER 0

#define STATUS_MESSAGE_MIN_GAP 30

//***** batt mon *************//
#define BATT_RC 0.05
#define BATT_VOLT_STATUS_DEVIDER 4.25

#define BAT_CELL_COUNT 10
#define BAT_MIN_CELL_VOLTAGE 3.0
#define BAT_THR_CELL_VOLATGE 3.4

//***** temp ****************//
#define MAX_TEMP 75



#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);   \
  void operator=(const TypeName&)
