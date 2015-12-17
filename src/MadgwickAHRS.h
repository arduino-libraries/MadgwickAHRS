//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>
#define sampleFreq	512.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

//----------------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick{
private:
    float invSqrt(float x);
    volatile float beta = betaDef;				// algorithm gain
    volatile float q0 = 1.0f;
    volatile float q1 = 0.0f;
    volatile float q2 = 0.0f;
    volatile float q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations
public:
    Madgwick(void){};
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    float getPitch(){return atan2(2 * q2 * q3 - 2 * q0 * q1, 2 * q0 * q0 + 2 * q3 * q3 - 1);};
    float getRoll(){return -1 * asin(2 * q1 * q3 + 2 * q0 * q2);};
    float getYaw(){return atan2(2 * q1 * q2 - 2 * q0 * q3, 2 * q0 * q0 + 2 * q1 * q1 - 1);};
};
#endif

//=====================================================================================================
// End of file
//=====================================================================================================
