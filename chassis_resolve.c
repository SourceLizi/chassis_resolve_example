/**
  ******************************************************************************
  * @file           : chassis_resolve.c
  * @brief          : functions to resolve the chassis motors data
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


#include "chassis_resolve.h"

/**
  * @brief 求两个角度差(8192角度制)
  * @param[in] current 当前角度
  * @param[in] last 上一次的角度
  * @param[in] cur_rpm 当前转速
  * @return 两个角度差值
  */
int16_t get_delta_ang(int16_t current,int16_t last, int16_t cur_rpm){
	int16_t result = current - last;
	//如果直接作差结果方向与转速相反，那么结果应该是多转或者少转了一圈
	if(result > 0 && cur_rpm < 0) result-=8192;
	else if(result < 0 && cur_rpm > 0) result +=8192;
	return result;
}
/**
  * @brief 复位底盘角度
  * @param[in] moto_data 电机角度数据结构体指针
  * @return 无
  */
void reset_ang_data(moto_ang_bundle_t* moto_data){
	//将其置为无效值
	moto_data->last_wheel_ang[0] = 8192;
	moto_data->last_wheel_ang[1] = 8192;
	moto_data->last_wheel_ang[2] = 8192;
	moto_data->last_wheel_ang[3] = 8192;
	moto_data->last_gimbal_ang = 8192;
}

/**
  * @brief 根据电机数据解算云台相对于地面的转速（单位rpm）
  * @param[in] moto_speed_data 电机转速数据结构体指针
  * @return Yaw轴云台的转速
  */
float get_gimbal_speed(moto_speed_bundle_t* moto_speed_data){
	float w_z = (float)(moto_speed_data->wheel_rpm[0] + moto_speed_data->wheel_rpm[1]
			+ moto_speed_data->wheel_rpm[2] + moto_speed_data->wheel_rpm[3])
			*(D_WHEEL/2) / REDUCTION_RATIO / (4 * K_XY) * K_CAL;
	return moto_speed_data->gimbal_rpm - w_z;
}


/**
  * @brief 根据电机数据解算云台相对于地面的绝对角度（8192角度制）
  * @param[in] moto_data 电机角度数据结构体指针
  * @param[in] moto_speed_data 电机转速数据结构体指针
  * @return z_angle Yaw轴云台的绝对角度
  */
float get_gimbal_angle(moto_ang_bundle_t* moto_data, moto_speed_bundle_t* moto_speed_data){
	int16_t wheel_delta_ang[4];
	int16_t gimbal_delta_ang;
	int16_t wheel_delta_ang_sum = 0;
	
	if(moto_data->last_wheel_ang[0] == 8192 || moto_data->last_wheel_ang[1]== 8192
	|| moto_data->last_wheel_ang[2] == 8192 || moto_data->last_wheel_ang[3] == 8192 
	|| moto_data->last_gimbal_ang == 8192){
		for(int i =0;i<4;i++){
			moto_data->last_wheel_ang[i] = moto_data->wheel_ang[i];
		}
		moto_data->z_angle = INIT_ANG;
	}else{
		for(int i=0;i<4;i++){
			wheel_delta_ang[i] = get_delta_ang(moto_data->wheel_ang[i]
							, moto_data->last_wheel_ang[i], moto_speed_data->wheel_rpm[i]);
			wheel_delta_ang_sum += wheel_delta_ang[i];
			moto_data->last_wheel_ang[i] = moto_data->wheel_ang[i];
		}
		gimbal_delta_ang = get_delta_ang(moto_data->gimbal_ang
					, moto_data->last_gimbal_ang, moto_speed_data->gimbal_rpm);
		moto_data->last_gimbal_ang = moto_data->gimbal_ang;
		
		moto_data->z_angle += (((D_WHEEL/2)*(float)wheel_delta_ang_sum)/REDUCTION_RATIO/(4*K_XY)*K_CAL - gimbal_delta_ang);
		if(moto_data->z_angle > 8192) moto_data->z_angle -= 8192;
		else if(moto_data->z_angle < 0) moto_data->z_angle += 8192;
	}
	return moto_data->z_angle;
}

