/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * Copyright (c) 2019 Sanger_X
 */

#include "chassis_subsystem.hpp"

#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/communication/remote.hpp>

using namespace aruwlib;
using namespace aruwlib::algorithms;

namespace aruwsrc
{
namespace chassis
{
void ChassisSubsystem::initialize()
{
    leftBackMotor.initialize();
    leftFrontMotor.initialize();
    rightBackMotor.initialize();
    rightFrontMotor.initialize();
}

void ChassisSubsystem::setDesiredOutput(float x, float y, float r)
{
    mecanumDriveCalculate(x, y, r, MAX_WHEEL_SPEED_SINGLE_MOTOR);
}

void ChassisSubsystem::refresh()
{
    updateMotorRpmPid(&leftFrontVelocityPid, &leftFrontMotor, leftFrontRpm);
    updateMotorRpmPid(&leftBackVelocityPid, &leftBackMotor, leftBackRpm);
    updateMotorRpmPid(&rightFrontVelocityPid, &rightFrontMotor, rightFrontRpm);
    updateMotorRpmPid(&rightBackVelocityPid, &rightBackMotor, rightBackRpm);
}

void ChassisSubsystem::mecanumDriveCalculate(float x, float y, float r, float maxWheelSpeed)
{
    // this is the distance between the center of the chassis to the wheel
    float chassisRotationRatio = sqrtf(
        powf(WIDTH_BETWEEN_WHEELS_X / 2.0f, 2.0f) + powf(WIDTH_BETWEEN_WHEELS_Y / 2.0f, 2.0f));

    // to take into account the location of the turret so we rotate around the turret rather
    // than the center of the chassis, we calculate the offset and than multiply however
    // much we want to rotate by
    float leftFrontRotationRatio =
        degreesToRadians(chassisRotationRatio - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightFroneRotationRatio =
        degreesToRadians(chassisRotationRatio - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);
    float leftBackRotationRatio =
        degreesToRadians(chassisRotationRatio + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightBackRotationRatio =
        degreesToRadians(chassisRotationRatio + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);

    float chassisRotateTranslated = radiansToDegrees(r) / chassisRotationRatio;
    leftFrontRpm = y + x + chassisRotateTranslated * leftFrontRotationRatio;
    rightFrontRpm = y - x + chassisRotateTranslated * rightFroneRotationRatio;
    leftBackRpm = -y + x + chassisRotateTranslated * leftBackRotationRatio;
    rightBackRpm = -y - x + chassisRotateTranslated * rightBackRotationRatio;

    leftFrontRpm =
        aruwlib::algorithms::limitVal<float>(leftFrontRpm, -maxWheelSpeed, maxWheelSpeed);
    rightFrontRpm =
        aruwlib::algorithms::limitVal<float>(rightFrontRpm, -maxWheelSpeed, maxWheelSpeed);
    leftBackRpm = aruwlib::algorithms::limitVal<float>(leftBackRpm, -maxWheelSpeed, maxWheelSpeed);
    rightBackRpm =
        aruwlib::algorithms::limitVal<float>(rightBackRpm, -maxWheelSpeed, maxWheelSpeed);

    chassisDesiredR = r;
}

void ChassisSubsystem::updateMotorRpmPid(
    modm::Pid<float>* pid,
    aruwlib::motor::DjiMotor* const motor,
    float desiredRpm)
{
    pid->update(desiredRpm - motor->getShaftRPM());
    motor->setDesiredOutput(pid->getValue());
}

float ChassisSubsystem::chassisSpeedRotationPID(float currentAngleError, float kp)
{
    float currentFilteredAngleErrorPrevious = chassisRotationErrorKalman.getLastFiltered();
    float currentFilteredAngleError = chassisRotationErrorKalman.filterData(currentAngleError);

    // P
    float currRotationPidP = currentAngleError * kp;
    currRotationPidP = aruwlib::algorithms::limitVal<float>(
        currRotationPidP,
        -CHASSIS_REVOLVE_PID_MAX_P,
        CHASSIS_REVOLVE_PID_MAX_P);

    // D
    float currentRotationPidD = 0.0f;
    if (abs(currentFilteredAngleError) > MIN_ERROR_ROTATION_D)
    {
        float currentErrorRotation = currentFilteredAngleError - currentFilteredAngleErrorPrevious;

        currentRotationPidD = -(currentErrorRotation)*CHASSIS_REVOLVE_PID_KD;

        currentRotationPidD = aruwlib::algorithms::limitVal<float>(
            currentRotationPidD,
            -CHASSIS_REVOLVE_PID_MAX_D,
            CHASSIS_REVOLVE_PID_MAX_D);
    }

    float wheelRotationSpeed = aruwlib::algorithms::limitVal<float>(
        currRotationPidP + currentRotationPidD,
        -MAX_WHEEL_SPEED_SINGLE_MOTOR,
        MAX_WHEEL_SPEED_SINGLE_MOTOR);

    return wheelRotationSpeed;
}

float ChassisSubsystem::calculateRotationTranslationalGain(float chassisRotationDesiredWheelspeed)
{
    // what we will multiply x and y speed by to take into account rotation
    float rTranslationalGain = 1.0f;

    // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing
    // power consumption when the wheel rotation speed for chassis rotationis greater than the
    // MIN_ROTATION_THRESHOLD
    if (fabsf(chassisRotationDesiredWheelspeed) > MIN_ROTATION_THRESHOLD)
    {
        // power(max revolve speed - specified revolve speed, 2)
        // / power(max revolve speed, 2)
        rTranslationalGain = powf(
            ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR + MIN_ROTATION_THRESHOLD -
                fabsf(chassisRotationDesiredWheelspeed) /
                    ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR,
            2.0f);
        rTranslationalGain = aruwlib::algorithms::limitVal<float>(rTranslationalGain, 0.0f, 1.0f);
    }
    return rTranslationalGain;
}

float ChassisSubsystem::getChassisDesiredRotation() const { return chassisDesiredR; }
}  // namespace chassis

}  // namespace aruwsrc
