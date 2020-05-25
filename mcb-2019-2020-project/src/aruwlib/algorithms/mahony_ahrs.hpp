/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef MAHONY_AHRS_H_
#define MAHONY_AHRS_H_

class MahonyAhrs
{
 public:
    MahonyAhrs() : q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
        integralFBx(0.0f), integralFBy(0.0f), integralFBz(0.0f),
        gx(0.0f), gy(0.0f), gz(0.0f), ax(0.0f), ay(0.0f), az(0.0f),
        mx(0.0f), my(0.0f), mz(0.0f)
    {}

    typedef struct
    {
        float ax;
        float ay;
        float az;

        float wx;
        float wy;
        float wz;

        float mx;
        float my;
        float mz;
    } ahrs_sensor;

    typedef struct
    {
        float roll;
        float pitch;
        float yaw;
    } attitude;

    void mahony_ahrs_update(ahrs_sensor *sensor, attitude *atti);
    void mahony_ahrs_updateIMU(ahrs_sensor *sensor, attitude *atti);
    void resetQuaternion(void);

 private:
    #define sampleFreq 500.0f         ///< sample frequency in Hz.
    #define twoKpDef (2.0f * 0.5f)    ///< 2 * proportional gain.
    #define twoKiDef (2.0f * 0.005f)  ///< 2 * integral gain.

    //---------------------------------------------------------------------
    // Variable definitions
    static constexpr float twoKp = twoKpDef;  ///< 2 * proportional gain (Kp).
    static constexpr float twoKi = twoKiDef;  ///< 2 * integral gain (Ki).

    ///< Quaternion of sensor frame relative to auxiliary frame.
    float q0, q1, q2, q3;

    ///< Integral error terms scaled by Ki.
    float integralFBx, integralFBy, integralFBz;

    float gx, gy, gz, ax, ay, az, mx, my, mz;
};  // class MahonyAhrs

#endif  // MAHONY_AHRS_H_
