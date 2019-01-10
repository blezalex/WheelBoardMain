#pragma once
#include "stm32f10x.h"
#include "arduino.h"

//#define REV5

//***** Apply board orientation transform *** //
// IMU is using right hand coordinate system, it tracks gravity vector and calculates angle between gravity vector and the board frame.
// Normal orientation - X forward, Y right Z down

#define BOARD_ROTATION_MACRO BOARD_ROTATION_UPSIDE_DOWN_X

#define BOARD_ROTATION_UPSIDE_DOWN_X(XYZ) XYZ[1]*=-1; XYZ[2]*=-1;  // rotated 180 deg around X axis


// ****** start mode settings *********/
#define START_MAX_POWER 300 // [-START_MAX_POWER: +START_MAX_POWER] out of MOTOR_CMD_RANGE
#define START_DURATION 1000 // in ms. Time to bring board from tilted to balanced.

#define START_ANGLE_DRIVE 9
#define START_ANGLE_DRIVE_FULL 6
#define START_D_MAX_MULTIPLIER 2

#define START_ANGLE_STEER 15

#define STOP_ANGLE_DRIVE 14
#define STOP_ANGLE_STEER 40

#define MAX_CHANGE_SINGLE_LOOP 300

#define MIN_MOTOR_CMD 990
#define MAX_MOTOR_CMD 2010
#define NEUTRAL_MOTOR_CMD 1500

#define MOTOR_CMD_RANGE (MAX_MOTOR_CMD - NEUTRAL_MOTOR_CMD)

#define ANGLE_DRIVE 1
#define ANGLE_STEER 0


#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);   \
  void operator=(const TypeName&)
