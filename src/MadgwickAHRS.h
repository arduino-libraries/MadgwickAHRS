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

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>

//-------------------------------------------------------------------------------------------
// Definitions

#define sampleFreqDef   512.0f          // sample frequency in Hz
#define betaDef         0.1f            // 2 * proportional gain

class Madgwick{
public:
    /*Initialize filter parameter (beta) = 0.1 and IMU's update frequency (sampleFrequency) = 512Hz*/
    Madgwick(); 
    /*Initialize filter parameter (beta)*/
    Madgwick(float beta);
    /*Initialize filter parameter (beta) and IMU's update frequency (sampleFrequency)*/
    Madgwick(float beta, float sampleFrequency);

    /*Set filter parameter (beta)*/
    void setBeta(float beta);
    /*Set IMU's update frequency (sampleFrequency)*/
    void setFrequency(float sampleFrequency);
    /*Getter function to obtain quaternion of body orientation and store it in quat*/
    void getQuaternion(float quat[4]);

    /*AHRS algorithm update with constant update frequency, which fuses gyroscope, accelerometer and magnetometer
    * Template parameters:
    * type = 0: NWU
    * type = 1: NED
    * type = 2: ENU
    * angle = D : Degree per second (Gyroscope's reading unit)
    * angle = R : Radian per second (Gyroscope's reading unit)
    */
    template<int type, char angle>
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
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
    template<int type, char angle>
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float timeDiff);
    /*IMU algorithm update with constant update frequency, which fuses gyroscope and accelerometer
    * Template parameters:
    * type = 0: NWU
    * type = 1: NED
    * type = 2: ENU
    * angle = D : Degree per second (Gyroscope's reading unit)
    * angle = R : Radian per second (Gyroscope's reading unit)
    */
    template<int type, char angle>
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
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
    template<int type, char angle>
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float timeDiff);
    
    /*Get roll angle in degree (Tait-Bryan in ZYX convention)*/
    float getRollDegree();
    /*Get pitch angle in degree (Tait-Bryan in ZYX convention)*/
    float getPitchDegree();
    /*Get yaw angle in degree (Tait-Bryan in ZYX convention)*/
    float getYawDegree();
    /*Get roll angle in radians (Tait-Bryan in ZYX convention)*/
    float getRollRadians();
    /*Get pitch angle in radians (Tait-Bryan in ZYX convention)*/
    float getPitchRadians();
    /*Get yaw angle in radians (Tait-Bryan in ZYX convention)*/
    float getYawRadians();

private:
    float q0, q1, q2, q3;   // quaternion of sensor frame relative to auxiliary frame (q0 q1 q2 q3)    
    float beta;		        // algorithm gain or filter parameter
    float invSampleFreq;    // reciprocal of IMU's update frequency
    float roll;             // roll angle (rad)
    float pitch;            // pitch angle (rad)
    float yaw;              // yaw angle (rad)
    char anglesComputed;    // variable to store if angles has been computed; 1 = computed, 0 = not computed

private:
    /*Part of the update() function to avoid code duplication*/
    template <int type, char angle>
    inline void updateCore(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float& qDot1, float& qDot2, float& qDot3, float& qDot4);
    /*Part of the updateIMU() function to avoid code duplication*/
    template <int type, char angle>
    inline void updateIMUCore(float gx, float gy, float gz, float ax, float ay, float az, float& qDot1, float& qDot2, float& qDot3, float& qDot4);
    /*Fast inverse square-root*/
    static float invSqrt(float x);
    /*Compute Tait-Bryan angles in ZYX convention*/
    void computeAngles();
    /*Convert degree to radian*/
    inline static float deg2rad(float value);
    /*Convert radian to degree*/
    inline static float rad2deg(float value);
};

#endif

