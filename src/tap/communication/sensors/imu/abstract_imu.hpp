/*
 * Copyright (c) 2020-2025 Advanced Robotics ...
 */

 #ifndef TAPROOT_ABSTRACT_IMU_HPP_
 #define TAPROOT_ABSTRACT_IMU_HPP_
 
 #include "tap/algorithms/MahonyAHRS.h"
 #include "tap/algorithms/transforms/orientation.hpp"
 #include "tap/algorithms/transforms/transform.hpp"
 #include "tap/algorithms/transforms/vector.hpp"
 #include "tap/architecture/periodic_timer.hpp"
 #include "tap/communication/sensors/imu/imu_interface.hpp"
 
 namespace tap::communication::sensors::imu
 {
 using tap::algorithms::transforms::Orientation;
 using tap::algorithms::transforms::Transform;
 
 
 constexpr float GRAVITY_MPS2 = 9.81f;
 
 class AbstractIMU : public ImuInterface
 {
 public:
     AbstractIMU(const Transform& mountingTransform = Transform::identity())
         : mountingTransform(mountingTransform)    
         {
     }
 
     virtual void setMountingTransform(const Transform& transform);
 
     virtual ~AbstractIMU() = default;
 
     virtual void initialize(float sampleFrequency, float mahonyKp, float mahonyKi);
 
     virtual void requestCalibration();
     virtual void periodicIMUUpdate();
 
     virtual ImuState getImuState() const { return imuState; }
 
     inline float getAx() const override { return mountingTransform.apply(imuData.accG).x(); }
     inline float getAy() const override { return mountingTransform.apply(imuData.accG).y(); }
     inline float getAz() const override { return mountingTransform.apply(imuData.accG).z(); }
     inline float getAzMinusG() const { return mountingTransform.apply(imuData.accG).z() - GRAVITY_MPS2; }
                 // mountingTransform.apply(tap::algorithms::transforms::Vector(0,0,GRAVITY_MPS2)).z(); }
 
     inline float getGx() const override { return imuData.gyroDegPerSec.x(); }
     inline float getGy() const override { return imuData.gyroDegPerSec.y(); }
     inline float getGz() const override { return imuData.gyroDegPerSec.z(); }
 
     inline float getTemp() const override { return imuData.temperature; }
 
     virtual float getYaw() const override { return mahonyAlgorithm.getYaw(); }
     virtual float getPitch() const override { return mahonyAlgorithm.getPitch(); }
     virtual float getRoll() const override { return mahonyAlgorithm.getRoll(); }
 
     virtual Orientation getImuOrientation() const
     {
         return Orientation(getRoll(), getPitch(), getYaw());
     }
 
     inline float getIMUFrameAx() const { return imuData.accG.x(); }
     inline float getIMUFrameAy() const { return imuData.accG.y(); }
     inline float getIMUFrameAz() const { return imuData.accG.z(); }
 
     struct ImuData
     {
         tap::algorithms::transforms::Vector accRaw = {0, 0, 0};
         tap::algorithms::transforms::Vector gyroRaw = {0, 0, 0};
         tap::algorithms::transforms::Vector accOffsetRaw = {0, 0, 0};
         tap::algorithms::transforms::Vector gyroOffsetRaw = {0, 0, 0};
         tap::algorithms::transforms::Vector accG = {0, 0, 0};
         tap::algorithms::transforms::Vector gyroDegPerSec = {0, 0, 0};
         float temperature = 0;
     };
 
     void setCalibrationSamples(int sampleCount) { offsetSampleCount = sampleCount; }
 
 protected:
     void resetOffsets();
     void computeOffsets();
     void setAccelOffset(float x, float y, float z);
     void setGyroOffset(float x, float y, float z);
 
     virtual inline float getAccelerationSensitivity() = 0;
 
     Transform mountingTransform;
     Mahony mahonyAlgorithm;
     ImuState imuState = ImuState::IMU_NOT_CONNECTED;
     int calibrationSample = 0;
     int offsetSampleCount = 1000;
     ImuData imuData;
     tap::arch::PeriodicMicroTimer readTimeout;
     uint32_t prevIMUDataReceivedTime = 0;
 };
 
 }  // namespace tap::communication::sensors::imu
 
 #endif  // TAPROOT_ABSTRACT_IMU_HPP_
 