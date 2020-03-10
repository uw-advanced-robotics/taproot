/**
  * the kalman filter can be used by first initializing the kalman
  * filter using KalmanCreate, than call filterData. The kalaman
  * filter will run and the optimal result will be returned
  * 
  * Example source:
  * 
  * float sensorData;
  * float filtered;
  * ExtendedKalman kalman(1.0f, 0.0f);
  * while(1)
  * {
  *     filtered = kalman.filterData(sensorData);
  * }
  */

#ifndef __KALMAN_HPP__
#define __KALMAN_HPP__

namespace aruwlib
{

namespace algorithms
{

class ExtendedKalman
{
 public:
    /**
     * @brief initializes a kalman filter with the given covariances
     * @param p the kalaman struct
     * @param tQ the system noise covariance 
     * @param tR the measurement noise covariance
     * @attention R is fixed. Larger Q means more trust in the
     *            data we are measuring.
     *            conversely, a smaller Q means more trust in
     *            the model's prediction (rather than the measured)
     *            value.
     */
    ExtendedKalman(float tQ, float tR);

    /**
     * @brief runs the kalman filter, returning the current prediction
     * @param p the kalman filter that should be ran
     * @param dat the value to be filtered
     * @retval the current prediction of what the data should be
     * 
     * @attention description of data
     * x(k | k) is the current prediction (filtered output)
     * (and than k - 1 would be the previous output)
     * Corresponding formula:
     * x(k | k-1) = A * X(k-1 | k-1) + B * U(k) + W(K)
     * p(k | k-1) = A*p(k-1 | k-1) * A' + Q
     * kg(k) = p(k | k-1) * H' / (H * p(k | k-1) * H' + R)
     * x(k | k) = X(k | k - 1) + kg(k) * (Z(k) - H * X(k | k-1))
     * p(k | k) = (I - kg(k) * H) * P(k | k-1)
     */
    float filterData(float dat);

    float getLastFiltered() const;

    void reset();

 private:
    float xLast;  // last optimal prediction
    float xMid;   // forcast optimal prediction
    float xNow;   // current optimal prediction
    float pMid;   // predicted covariance
    float pNow;   // current covariance
    float pLast;  // previous covariance
    float kg;  // kalman gain
    float A;   // system parameters
    float B;
    float Q;
    float R;
    float H;
};

}  // namespace algorithms

}  // namespace aruwlib

#endif
