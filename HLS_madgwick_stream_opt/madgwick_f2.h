/******************************************************************************/
/*                                                                            */
/* madgwick_f.h -- Driver for the Madgwick filter                              */
/*                                                                            */
/******************************************************************************/
/* Author: Diego Galindo                                         */
/* Copyright 2025, Digilent Inc.                                              */
/******************************************************************************/

#ifndef madgwick_f2_H
#define madgwick_f2_H

//----------------------------------------------------------------------------------------------------
// Variable declaration
#define PI 3.141592
#define R2D (180.00f/3.141592f)

extern volatile float beta;	// ganancía del algoritmo

typedef struct {
	float q0, q1, q2, q3;	// quaterniones del algoritmo
	float roll, pitch, yaw;			// angulos x,y,z
}MadgwickFilt;

//---------------------------------------------------------------------------------------------------
// Function declarations
void Madgwick_INIT(MadgwickFilt*f);

void MadgwickAHRSupdate(MadgwickFilt*f,
						float gx, float gy, float gz,
						float ax, float ay, float az,
						float mx, float my, float mz);

void MadgwickAHRSupdateIMU(MadgwickFilt*f,
						   float gx, float gy, float gz,
						   float ax, float ay, float az);

void computeAngles(MadgwickFilt*f);
#endif /* INC_MADGWICK_FILTER_H_ */
