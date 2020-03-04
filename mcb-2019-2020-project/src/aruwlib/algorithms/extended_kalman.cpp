#include "extended_kalman.hpp"

namespace aruwlib
{

namespace algorithms
{

ExtendedKalman::ExtendedKalman(float tQ, float tR) :
    xLast(0.0f),
    xMid(0.0f),
    xNow(0.0f),
    pMid(0.0f),
    pNow(0.0f),
    pLast(0.0f),
    kg(0.0f),
    A(1.0f),
    B(0.0f),
    Q(tQ),
    R(tR),
    H(1.0f)
{}

float ExtendedKalman::filterData(float dat)
{
    xMid = A * xLast;
    pMid = A * pLast + Q;
    kg = pMid / (pMid + R);
    xNow = xMid + kg * (dat-xMid);
    pNow = (1 - kg) * pMid;
    pLast = pNow;
    xLast = xNow;
    return xNow;
}

float ExtendedKalman::getLastFiltered() const
{
    return xLast;
}

void ExtendedKalman::reset()
{
    xNow = 0.0f;
    xMid = 0.0f;
    xLast = 0.0f;
    pNow = 0.0f;
    pMid = 0.0f;
    pLast = 0.0f;
}

}  // namespace algorithms

}  // namespace aruwlib
