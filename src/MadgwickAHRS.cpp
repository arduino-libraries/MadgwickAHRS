//=============================================================================================
// MadgwickAHRS.c
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=============================================================================================

//-------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"

//============================================================================================
// Functions

//-------------------------------------------------------------------------------------------
Madgwick::Madgwick() {
	beta = betaDef;
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	invSampleFreq = 1.0f / sampleFreqDef;
	anglesComputed = 0;
}

Madgwick::Madgwick(float beta) {
	this->beta = beta;
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	invSampleFreq = 1.0f / sampleFreqDef;
	anglesComputed = 0;
}

Madgwick::Madgwick(float beta, float sampleFrequency) {
	this->beta = beta;
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	invSampleFreq = 1.0f / sampleFrequency;
	anglesComputed = 0;
}

/*Set filter parameter (beta)*/
void Madgwick::setBeta(float beta) {
	this->beta = beta;
}

/*Set IMU's update frequency (sampleFrequency)*/
void Madgwick::setFrequency(float sampleFrequency) { 
	invSampleFreq = 1.0f / sampleFrequency; 
}

/*Getter function to obtain quaternion of body orientation and store it in quat*/
void Madgwick::getQuaternion(float quat[4]) {
	quat[0] = q0;
	quat[1] = q1;
	quat[2] = q2;
	quat[3] = q3;
}

/*AHRS algorithm update with constant update frequency, which fuses gyroscope, accelerometer and magnetometer
* Template parameters:
* type = 0: NWU
* type = 1: NED
* type = 2: ENU
* angle = D : Degree per second (Gyroscope's reading unit)
* angle = R : Radian per second (Gyroscope's reading unit)
*/
template void Madgwick::update<0,'D'>(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
template void Madgwick::update<1,'D'>(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
template void Madgwick::update<2,'D'>(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
template void Madgwick::update<0,'R'>(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
template void Madgwick::update<1,'R'>(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
template void Madgwick::update<2,'R'>(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

/*AHRS algorithm update with varying update frequency, which fuses gyroscope, accelerometer and magnetometer
* Template parameters:
* type = 0: NWU
* type = 1: NED
* type = 2: ENU
* angle = D : Degree per second (Gyroscope's reading unit)
* angle = R : Radian per second (Gyroscope's reading unit)
* 
* Inputs: timeDiff = elapsed time between present and previous IMU's reading
*/
template void Madgwick::update<0,'D'>(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float timeDiff);
template void Madgwick::update<1,'D'>(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float timeDiff);
template void Madgwick::update<2,'D'>(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float timeDiff);
template void Madgwick::update<0,'R'>(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float timeDiff);
template void Madgwick::update<1,'R'>(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float timeDiff);
template void Madgwick::update<2,'R'>(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float timeDiff);

template<int type, char angle>
void Madgwick::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	
	float qDot1, qDot2, qDot3, qDot4;
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		updateIMU<type, angle>(gx, gy, gz, ax, ay, az);
		return;
	}

	updateCore<type, angle>(gx, gy, gz, ax, ay, az, mx, my, mz, qDot1, qDot2, qDot3, qDot4);

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * invSampleFreq;
	q1 += qDot2 * invSampleFreq;
	q2 += qDot3 * invSampleFreq;
	q3 += qDot4 * invSampleFreq;

	// Normalise quaternion
	float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

template<int type, char angle>
void Madgwick::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float timeDiff) {

	float qDot1, qDot2, qDot3, qDot4;
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		updateIMU<type, angle>(gx, gy, gz, ax, ay, az, timeDiff);
		return;
	}

	updateCore<type, angle>(gx, gy, gz, ax, ay, az, mx, my, mz, qDot1, qDot2, qDot3, qDot4);

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * timeDiff;
	q1 += qDot2 * timeDiff;
	q2 += qDot3 * timeDiff;
	q3 += qDot4 * timeDiff;

	// Normalise quaternion
	float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

/*Part of the Madgwick::update() function to avoid code duplication*/
template<int type, char angle>
inline void Madgwick::updateCore(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float& qDot1, float& qDot2, float& qDot3, float& qDot4) {
	float recipNorm;
	float s0, s1, s2, s3;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

#if __cplusplus < 201703L
	if (angle == 'D')
#else
	if constexpr (angle == 'D' && angle != 'R')
#endif
	{
		// Convert gyroscope degrees/sec to radians/sec
		gx = deg2rad(gx);
		gy = deg2rad(gy);
		gz = deg2rad(gz);
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

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
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

#if __cplusplus < 201703L
		if (type == 0) 
#else
		if constexpr (type == 0) 
#endif
		{
			// Gradient decent algorithm corrective step (NWU Frame)
			s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		}
#if __cplusplus < 201703L
		else if (type == 1)
#else
		else if constexpr (type == 1)
#endif 
		{
			// Gradient decent algorithm corrective step (NED Frame)
			s0 = _2q2 * (- 2.0f * q1q3 + _2q0q2 - ax) - _2q1 * (- 2.0f * q0q1 - _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s1 = -_2q3 * (- 2.0f * q1q3 + _2q0q2 - ax) - _2q0 * (- 2.0f * q0q1 - _2q2q3 - ay) + 4.0f * q1 * (-1 + 2.0f * q1q1 + 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s2 = _2q0 * (- 2.0f * q1q3 + _2q0q2 - ax) - _2q3 * (- 2.0f * q0q1 - _2q2q3 - ay) + 4.0f * q2 * (-1 + 2.0f * q1q1 + 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s3 = -_2q1 * (- 2.0f * q1q3 + _2q0q2 - ax) - _2q2 * (- 2.0f * q0q1 - _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		}
#if __cplusplus < 201703L
		else
#else
		else if constexpr (type == 2) 
#endif 
		{
			// Gradient decent algorithm corrective step (ENU Frame)
			s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) + (_2bx * q3 - _2bz * q2) * (_2bx * (q1q2 + q0q3) + _2bz * (q1q3 - q0q2) - mx) + _2bz * q1 * (_2bx * (0.5f - q1q1 - q3q3) + _2bz * (q0q1 + q2q3) - my) - _2bx * q1 * (_2bx * (q2q3 - q0q1) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (_2bx * q2 + _2bz * q3) * (_2bx * (q1q2 + q0q3) + _2bz * (q1q3 - q0q2) - mx) + (-_4bx * q1 + _2bz * q0) * (_2bx * (0.5f - q1q1 - q3q3) + _2bz * (q0q1 + q2q3) - my) + (-_2bx * q0 - _4bz * q1) * (_2bx * (q2q3 - q0q1) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (_2bx * q1 - _2bz * q0) * (_2bx * (q1q2 + q0q3) + _2bz * (q1q3 - q0q2) - mx) + _2bz * q3 * (_2bx * (0.5f - q1q1 - q3q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q2) * (_2bx * (q2q3 - q0q1) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (_2bx * q0 + _2bz * q1) * (_2bx * (q1q2 + q0q3) + _2bz * (q1q3 - q0q2) - mx) + (-_4bx * q3 + _2bz * q2) * (_2bx * (0.5f - q1q1 - q3q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q2q3 - q0q1) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		}
		
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}
}

//-------------------------------------------------------------------------------------------
/*IMU algorithm update with constant update frequency, which fuses gyroscope and accelerometer
* Template parameters:
* type = 0: NWU
* type = 1: NED
* type = 2: ENU
* angle = D : Degree per second (Gyroscope's reading unit)
* angle = R : Radian per second (Gyroscope's reading unit)
*/
template void Madgwick::updateIMU<0,'D'>(float gx, float gy, float gz, float ax, float ay, float az);
template void Madgwick::updateIMU<1,'D'>(float gx, float gy, float gz, float ax, float ay, float az);
template void Madgwick::updateIMU<2,'D'>(float gx, float gy, float gz, float ax, float ay, float az);
template void Madgwick::updateIMU<0,'R'>(float gx, float gy, float gz, float ax, float ay, float az);
template void Madgwick::updateIMU<1,'R'>(float gx, float gy, float gz, float ax, float ay, float az);
template void Madgwick::updateIMU<2,'R'>(float gx, float gy, float gz, float ax, float ay, float az);

/*IMU algorithm update with varying update frequency, which fuses gyroscope and accelerometer
* Template parameters:
* type = 0: NWU
* type = 1: NED
* type = 2: ENU
* angle = D : Degree per second (Gyroscope's reading unit)
* angle = R : Radian per second (Gyroscope's reading unit)
* 
* Inputs: timeDiff = elapsed time between present and previous IMU's reading
*/
template void Madgwick::updateIMU<0,'D'>(float gx, float gy, float gz, float ax, float ay, float az, float timeDiff);
template void Madgwick::updateIMU<1,'D'>(float gx, float gy, float gz, float ax, float ay, float az, float timeDiff);
template void Madgwick::updateIMU<2,'D'>(float gx, float gy, float gz, float ax, float ay, float az, float timeDiff);
template void Madgwick::updateIMU<0,'R'>(float gx, float gy, float gz, float ax, float ay, float az, float timeDiff);
template void Madgwick::updateIMU<1,'R'>(float gx, float gy, float gz, float ax, float ay, float az, float timeDiff);
template void Madgwick::updateIMU<2,'R'>(float gx, float gy, float gz, float ax, float ay, float az, float timeDiff);

template<int type, char angle>
void Madgwick::updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	
	float qDot1, qDot2, qDot3, qDot4;

	updateIMUCore<type, angle>(gx, gy, gz, ax, ay, az, qDot1, qDot2, qDot3, qDot4);

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * invSampleFreq;
	q1 += qDot2 * invSampleFreq;
	q2 += qDot3 * invSampleFreq;
	q3 += qDot4 * invSampleFreq;

	// Normalise quaternion
	float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

template<int type, char angle>
void Madgwick::updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float timeDiff) {
	
	float qDot1, qDot2, qDot3, qDot4;

	updateIMUCore<type, angle>(gx, gy, gz, ax, ay, az, qDot1, qDot2, qDot3, qDot4);

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * timeDiff;
	q1 += qDot2 * timeDiff;
	q2 += qDot3 * timeDiff;
	q3 += qDot4 * timeDiff;

	// Normalise quaternion
	float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

/*Part of the Madgwick::updateIMU() function to avoid code duplication*/
template <int type, char angle>
inline void Madgwick::updateIMUCore(float gx, float gy, float gz, float ax, float ay, float az, float& qDot1, float& qDot2, float& qDot3, float& qDot4) {
	float recipNorm;
	float s0, s1, s2, s3;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

#if __cplusplus < 201703L
	if (angle == 'D')
#else
	if constexpr (angle == 'D' && angle != 'R')
#endif
	{
		// Convert gyroscope degrees/sec to radians/sec
		gx = deg2rad(gx);
		gy = deg2rad(gy);
		gz = deg2rad(gz);
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

#if __cplusplus < 201703L
		if (type == 0 || type == 2) 
#else
		if constexpr (type == 0 || type == 2) 
#endif
		{
			// Gradient decent algorithm corrective step (NWU / ENU)
			s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
			s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
			s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
			s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		}
#if __cplusplus < 201703L
		else 
#else	
		else if constexpr (type == 1) 
#endif	
		{
			// Gradient decent algorithm corrective step (NED)
			s0 = _4q0 * q2q2 - _2q2 * ax + _4q0 * q1q1 + _2q1 * ay;
			s1 = _4q1 * q3q3 + _2q3 * ax + 4.0f * q0q0 * q1 + _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 - _4q1 * az;
			s2 = 4.0f * q0q0 * q2 - _2q0 * ax + _4q2 * q3q3 + _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 - _4q2 * az;
			s3 = 4.0f * q1q1 * q3 + _2q1 * ax + 4.0f * q2q2 * q3 + _2q2 * ay;
		}
		
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}
}

//-------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float Madgwick::invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//-------------------------------------------------------------------------------------------

/*Compute Tait-Bryan angles in ZYX convention*/
void Madgwick::computeAngles() {
	roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
	anglesComputed = 1;
}

/*Get roll angle in degree (Tait-Bryan in ZYX convention)*/
float Madgwick::getRollDegree() {
	if (!anglesComputed) computeAngles();
	return rad2deg(roll);
}

/*Get pitch angle in degree (Tait-Bryan in ZYX convention)*/
float Madgwick::getPitchDegree() {
	if (!anglesComputed) computeAngles();
	return rad2deg(pitch);
}

/*Get yaw angle in degree (Tait-Bryan in ZYX convention)*/
float Madgwick::getYawDegree() {
	if (!anglesComputed) computeAngles();
	return rad2deg(yaw);
}

/*Get roll angle in radians (Tait-Bryan in ZYX convention)*/
float Madgwick::getRollRadians() {
	if (!anglesComputed) computeAngles();
	return roll;
}

/*Get pitch angle in radians (Tait-Bryan in ZYX convention)*/
float Madgwick::getPitchRadians() {
	if (!anglesComputed) computeAngles();
	return pitch;
}

/*Get yaw angle in radians (Tait-Bryan in ZYX convention)*/
float Madgwick::getYawRadians() {
	if (!anglesComputed) computeAngles();
	return yaw;
}

/*Convert degree to radian*/
inline float Madgwick::deg2rad(float value) {
    return value * PI / 180.0f;
}

/*Convert radian to degree*/
inline float Madgwick::rad2deg(float value) {
    return value * 180.0f / PI;
}
