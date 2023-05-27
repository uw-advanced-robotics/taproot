#ifndef TAPROOT_CROSS_PRODUCT_HPP_
#define TAPROOT_CROSS_PRODUCT_HPP_
#include "cmsis_mat.hpp"
namespace tap::algorithms
// TODO: move to math_user_utils
{
inline CMSISMat<3, 1> cross(const CMSISMat<3, 1>& a, const CMSISMat<3, 1>& b)
{
    return CMSISMat<3, 1>({
        a.data[1] * b.data[2] - a.data[2] * b.data[1],
        a.data[2] * b.data[0] - a.data[0] * b.data[2],
        a.data[0] * b.data[1] - a.data[1] * b.data[0]});
}
}
#endif  // TAPROOT_CROSS_PRODUCT_HPP_