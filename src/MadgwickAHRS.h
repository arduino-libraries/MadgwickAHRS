//=============================================================================================
// MadgwickAHRS.h
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
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick{
private:
    static float invSqrt(float x);
    float beta;				// algorithm gain
    float q0;
    float q1;
    float q2;
    float q3;	// quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    unsigned long lastUpdateMillis;
    float roll;
    float pitch;
    float yaw;
    char anglesComputed;
    void innerUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float ellapsedTime);
    void innerUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float ellapsedTime);
    void computeAngles();
    float computeEllapsedTime(unsigned long currentMillis);

//-------------------------------------------------------------------------------------------
// Function declarations
public:
    Madgwick(void);
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, unsigned long currentMillis);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, unsigned long currentMillis);
    //float getPitch(){return atan2f(2.0f * q2 * q3 - 2.0f * q0 * q1, 2.0f * q0 * q0 + 2.0f * q3 * q3 - 1.0f);};
    //float getRoll(){return -1.0f * asinf(2.0f * q1 * q3 + 2.0f * q0 * q2);};
    //float getYaw(){return atan2f(2.0f * q1 * q2 - 2.0f * q0 * q3, 2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f);};
    float getRoll() {
        if (!anglesComputed) computeAngles();
        return roll * 57.29578f;
    }
    float getPitch() {
        if (!anglesComputed) computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw() {
        if (!anglesComputed) computeAngles();
        return yaw * 57.29578f + 180.0f;
    }
    void getRPY(float * _roll, float * _pitch, float * _yaw) {
	if (!anglesComputed) computeAngles();
	* _roll = roll * 57.29578f;
	* _pitch = pitch * 57.29578f;
	* _yaw = yaw * 57.29578f + 180.0f;
    }
    float getRollRadians() {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    float getPitchRadians() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    float getYawRadians() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }
    void getRPYRadians(float * _roll, float * _pitch, float * _yaw) {
	if (!anglesComputed) computeAngles();
	* _roll = roll;
	* _pitch = pitch;
	* _yaw = yaw;
    }
    void getQuaternion(float * _q0, float * _q1, float * _q2, float * _q3) {
	* _q0 = q0;
	* _q1 = q1;
	* _q2 = q2;
	* _q3 = q3;
    }
};
#endif

