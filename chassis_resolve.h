/**
  ******************************************************************************
  * @file           : chassis_resolve.h
  * @brief          : headers of chassis_resolve.c
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 SCNU PIONEER Team.
  * All rights reserved.</center></h2>
  *
  * This program is free software; you can redistribute it and/or
  * modify it under the terms of the GNU General Public License
  * as published by the Free Software Foundation; either version 2
  * of the License, or (at your option) any later version.
  *
  ******************************************************************************
  */

#include <stdint.h>

#define REDUCTION_RATIO (3591.0f/187.0f)	///< M3508减速比
#define D_WHEEL (0.1524f)					///< 官方麦轮直径

#define K_XY 0.38678f
#define K_CAL 0.9998563f
#define INIT_ANG 4096

typedef struct{
	//输入参数
	uint16_t wheel_ang[4];	///< 底盘四轮角度，8192制
	int16_t wheel_rpm[4];	///< 底盘四轮转速，单位rpm
	uint16_t gimbal_ang;	///< 云台相对角度，8192制
	int16_t gimbal_rpm;		///< 云台转速，单位rpm
	//状态参数
	uint16_t last_wheel_ang[4];
	uint16_t last_gimbal_ang;
	float z_angle;
}moto_ang_bundle_t;

typedef struct{
	int16_t wheel_rpm[4];
	int16_t gimbal_rpm;
}moto_speed_bundle_t;

void reset_ang_data(moto_ang_bundle_t* moto_data);
float get_gimbal_speed(moto_speed_bundle_t* moto_speed_data);
float get_gimbal_angle(moto_ang_bundle_t* moto_data, moto_speed_bundle_t* moto_speed_data);


