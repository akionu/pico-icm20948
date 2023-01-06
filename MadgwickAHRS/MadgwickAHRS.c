//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	100.0f		// sample frequency in Hz
//#define betaDef		0.5f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

//volatile float beta = betaDef;								// 2 * proportional gain (Kp)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MadgwickAHRSupdate(madgwick_ahrs_t *data, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(data, gx, gy, gz, ax, ay, az);
		return;
	}

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-(data->q[1]) * gx - (data->q[2]) * gy - (data->q[3]) * gz);
	qDot2 = 0.5f * ((data->q[0]) * gx + (data->q[2]) * gz - (data->q[3]) * gy);
	qDot3 = 0.5f * ((data->q[0]) * gy - (data->q[1]) * gz + (data->q[3]) * gx);
	qDot4 = 0.5f * ((data->q[0]) * gz + (data->q[1]) * gy - (data->q[2]) * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * (data->q[0]) * mx;
		_2q0my = 2.0f * (data->q[0]) * my;
		_2q0mz = 2.0f * (data->q[0]) * mz;
		_2q1mx = 2.0f * (data->q[1]) * mx;
		_2q0 = 2.0f * (data->q[0]);
		_2q1 = 2.0f * (data->q[1]);
		_2q2 = 2.0f * (data->q[2]);
		_2q3 = 2.0f * (data->q[3]);
		_2q0q2 = 2.0f * (data->q[0]) * (data->q[2]);
		_2q2q3 = 2.0f * (data->q[2]) * (data->q[3]);
		q0q0 = (data->q[0]) * (data->q[0]);
		q0q1 = (data->q[0]) * (data->q[1]);
		q0q2 = (data->q[0]) * (data->q[2]);
		q0q3 = (data->q[0]) * (data->q[3]);
		q1q1 = (data->q[1]) * (data->q[1]);
		q1q2 = (data->q[1]) * (data->q[2]);
		q1q3 = (data->q[1]) * (data->q[3]);
		q2q2 = (data->q[2]) * (data->q[2]);
		q2q3 = (data->q[2]) * (data->q[3]);
		q3q3 = (data->q[3]) * (data->q[3]);

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * (data->q[3]) + _2q0mz * (data->q[2]) + mx * q1q1 + _2q1 * my * (data->q[2]) + _2q1 * mz * (data->q[3]) - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * (data->q[3]) + my * q0q0 - _2q0mz * (data->q[1]) + _2q1mx * (data->q[2]) - my * q1q1 + my * q2q2 + _2q2 * mz * (data->q[3]) - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * (data->q[2]) + _2q0my * (data->q[1]) + mz * q0q0 + _2q1mx * (data->q[3]) - mz * q1q1 + _2q2 * my * (data->q[3]) - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * (data->q[2]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * (data->q[3]) + _2bz * (data->q[1])) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * (data->q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * (data->q[1]) * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * (data->q[3]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * (data->q[2]) + _2bz * (data->q[0])) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * (data->q[3]) - _4bz * (data->q[1])) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * (data->q[2]) * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * (data->q[2]) - _2bz * (data->q[0])) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * (data->q[1]) + _2bz * (data->q[3])) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * (data->q[0]) - _4bz * (data->q[2])) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * (data->q[3]) + _2bz * (data->q[1])) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * (data->q[0]) + _2bz * (data->q[2])) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * (data->q[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= (data->beta) * s0;
		qDot2 -= (data->beta) * s1;
		qDot3 -= (data->beta) * s2;
		qDot4 -= (data->beta) * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	(data->q[0]) += qDot1 * (1.0f / sampleFreq);
	(data->q[1]) += qDot2 * (1.0f / sampleFreq);
	(data->q[2]) += qDot3 * (1.0f / sampleFreq);
	(data->q[3]) += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt((data->q[0]) * (data->q[0]) + (data->q[1]) * (data->q[1]) + (data->q[2]) * (data->q[2]) + (data->q[3]) * (data->q[3]));
	(data->q[0]) *= recipNorm;
	(data->q[1]) *= recipNorm;
	(data->q[2]) *= recipNorm;
	(data->q[3]) *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(madgwick_ahrs_t *data, float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-(data->q[1]) * gx - (data->q[2]) * gy - (data->q[3]) * gz);
	qDot2 = 0.5f * ((data->q[0]) * gx + (data->q[2]) * gz - (data->q[3]) * gy);
	qDot3 = 0.5f * ((data->q[0]) * gy - (data->q[1]) * gz + (data->q[3]) * gx);
	qDot4 = 0.5f * ((data->q[0]) * gz + (data->q[1]) * gy - (data->q[2]) * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * (data->q[0]);
		_2q1 = 2.0f * (data->q[1]);
		_2q2 = 2.0f * (data->q[2]);
		_2q3 = 2.0f * (data->q[3]);
		_4q0 = 4.0f * (data->q[0]);
		_4q1 = 4.0f * (data->q[1]);
		_4q2 = 4.0f * (data->q[2]);
		_8q1 = 8.0f * (data->q[1]);
		_8q2 = 8.0f * (data->q[2]);
		q0q0 = (data->q[0]) * (data->q[0]);
		q1q1 = (data->q[1]) * (data->q[1]);
		q2q2 = (data->q[2]) * (data->q[2]);
		q3q3 = (data->q[3]) * (data->q[3]);

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * (data->q[1]) - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * (data->q[2]) + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * (data->q[3]) - _2q1 * ax + 4.0f * q2q2 * (data->q[3]) - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= (data->beta) * s0;
		qDot2 -= (data->beta) * s1;
		qDot3 -= (data->beta) * s2;
		qDot4 -= (data->beta) * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	(data->q[0]) += qDot1 * (1.0f / sampleFreq);
	(data->q[1]) += qDot2 * (1.0f / sampleFreq);
	(data->q[2]) += qDot3 * (1.0f / sampleFreq);
	(data->q[3]) += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt((data->q[0]) * (data->q[0]) + (data->q[1]) * (data->q[1]) + (data->q[2]) * (data->q[2]) + (data->q[3]) * (data->q[3]));
	(data->q[0]) *= recipNorm;
	(data->q[1]) *= recipNorm;
	(data->q[2]) *= recipNorm;
	(data->q[3]) *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
