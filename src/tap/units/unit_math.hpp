#ifndef TAPROOT_UNIT_MATH_HPP_
#define TAPROOT_UNIT_MATH_HPP_

#include <cmath>

#include "quantity.hpp"
#include "units.hpp"

namespace tap::units::math
{
/**
 * @brief Takes the absolute value of a quantity
 *
 * @param lhs the quantity
 * @return constexpr Q the absolute value
 */
template <isQuantity Q>
constexpr Q abs(const Q& lhs)
{
    return Q(std::abs(lhs.internal()));
}

/**
 * @brief Takes the maximum (closest to +infinity) of two isomorphic quantities
 *
 * @param lhs the first operand
 * @param rhs the second operand
 * @return constexpr Q
 */
template <isQuantity Q, isQuantity R>
constexpr Q max(const Q& lhs, const R& rhs) requires Isomorphic<Q, R>
{
    return (lhs > rhs ? lhs : rhs);
}

/**
 * @brief Takes the minimum (closest to -infinity) of two isomorphic quantites
 *
 * @param lhs the first operand
 * @param rhs the second operand
 * @return constexpr Q
 */
template <isQuantity Q, isQuantity R>
constexpr Q min(const Q& lhs, const R& rhs) requires Isomorphic<Q, R>
{
    return (lhs < rhs ? lhs : rhs);
}

/**
 * @brief Takes the R'th power of a quantity
 *
 * @tparam R the power to raise the quantity to
 * @param lhs
 * @return constexpr S the result of the operation
 */
template <int R, isQuantity Q, isQuantity S = Exponentiated<Q, ratio<R>>>
constexpr S pow(const Q& lhs)
{
    return S(std::pow(lhs.internal(), R));
}

/**
 * @brief Takes the square of a quantity
 *
 * @param lhs the quantity
 * @return constexpr S the square of the quantity
 */
template <isQuantity Q, isQuantity S = Exponentiated<Q, ratio<2>>>
constexpr S square(const Q& lhs)
{
    return pow<2>(lhs);
}

/**
 * @brief Takes the cube of a quantity
 *
 * @param lhs the quantity
 * @return constexpr S the cube of the quantity
 */
template <isQuantity Q, isQuantity S = Exponentiated<Q, ratio<3>>>
constexpr S cube(const Q& lhs)
{
    return pow<3>(lhs);
}

/**
 * @brief Takes the R'th root of a quantity

 * @tparam R the order of the root
 * @param lhs the quantity
 * @return constexpr S the R root of the quantity
 */
template <int R, isQuantity Q, isQuantity S = Rooted<Q, ratio<R>>>
constexpr S root(const Q& lhs)
{
    return S(std::pow(lhs.internal(), 1.0 / R));
}

/**
 * @brief Takes the square root of a quantity
 *
 * @param lhs the quantity
 * @return constexpr S the square root of the quantity
 */
template <isQuantity Q, isQuantity S = Rooted<Q, ratio<2>>>
constexpr S sqrt(const Q& lhs)
{
    return root<2>(lhs);
}

/**
 * @brief Takes the cube root of a quantity
 *
 * @param lhs the quantity
 * @return constexpr S the cube root of the quantity
 */
template <isQuantity Q, isQuantity S = Rooted<Q, ratio<3>>>
constexpr S cbrt(const Q& lhs)
{
    return root<3>(lhs);
}

/**
 * @brief Calculates the hypotenuse of a right triangle with two sides of isomorphic quantities
 *
 * @param lhs x side
 * @param rhs y side
 * @return constexpr Q the hypotenuse
 */
template <isQuantity Q, isQuantity R>
constexpr Q hypot(const Q& lhs, const R& rhs) requires Isomorphic<Q, R>
{
    return Q(std::hypot(lhs.internal(), rhs.internal()));
}

/**
 * @brief Returns the remainder of a division of two isomorphic quantities
 *
 * @param lhs the dividend
 * @param rhs the divisor
 * @return constexpr Q the remainder
 */
template <isQuantity Q, isQuantity R>
constexpr Q mod(const Q& lhs, const R& rhs) requires Isomorphic<Q, R>
{
    return Q(std::fmod(lhs.internal(), rhs.internal()));
}

/**
 * @brief Returns the absolute value of x with the sign of y
 *
 * @param lhs the quantity to take the absolute value of (x)
 * @param rhs the quantity to take the sign of (y)
 * @return constexpr the first quantity with the sign of the second
 */
template <isQuantity Q1, isQuantity Q2>
constexpr Q1 copysign(const Q1& lhs, const Q2& rhs)
{
    return Q1(std::copysign(lhs.internal(), rhs.internal()));
}

/**
 * @brief Returns the sign of a quantity
 *
 * @param lhs the quantity
 * @return constexpr int the sign of the quantity
 */
template <isQuantity Q>
constexpr int sgn(const Q& lhs)
{
    return lhs.internal() < 0 ? -1 : 1;
}

/**
 * @brief Returns true if the quantity is negative
 *
 * @param lhs the quantity
 * @return true if the quantity is negative, false otherwise
 */
template <isQuantity Q>
constexpr bool signbit(const Q& lhs)
{
    return std::signbit(lhs.internal());
}

/**
 * @brief Clamps a quantity between two other isomporphic quantities
 *
 * @param lhs the quantity to clamp
 * @param lo the lower bound
 * @param hi the upper bound
 * @return constexpr Q the clamped quantity
 */
template <isQuantity Q, isQuantity R, isQuantity S>
constexpr Q clamp(const Q& lhs, const R& lo, const S& hi) requires Isomorphic<Q, R, S>
{
    return Q(std::clamp(lhs.internal(), lo.internal(), hi.internal()));
}

/**
 * @brief Rounds a quantity up (towards +infinity) to the nearest multiple of another isomorphic
 * quantity
 * @param lhs the quantity to round
 * @param rhs the multiple to round to
 */
template <isQuantity Q, isQuantity R>
constexpr Q ceil(const Q& lhs, const R& rhs) requires Isomorphic<Q, R>
{
    return Q(std::ceil(lhs.internal() / rhs.internal()) * rhs.internal());
}

/**
 * @brief Rounds a quantity down (towards -infinity) to the nearest multiple of another isomorphic
 * quantity
 * @param lhs the quantity to round
 * @param rhs the multiple to round to
 */
template <isQuantity Q, isQuantity R>
constexpr Q floor(const Q& lhs, const R& rhs) requires Isomorphic<Q, R>
{
    return Q(std::floor(lhs.internal() / rhs.internal()) * rhs.internal());
}

/**
 * @brief Rounds a quantity (towards zero) to the nearest multiple of another isomorphic quantity
 * @param lhs the quantity to round
 * @param rhs the multiple to round to
 */
template <isQuantity Q, isQuantity R>
constexpr Q trunc(const Q& lhs, const R& rhs) requires Isomorphic<Q, R>
{
    return Q(std::trunc(lhs.internal() / rhs.internal()) * rhs.internal());
}

/**
 * @brief Rounds a quantity to the nearest multiple of another isomorphic quantity
 * @param lhs the quantity to round
 * @param rhs the multiple to round to
 */
template <isQuantity Q, isQuantity R>
constexpr Q round(const Q& lhs, const R& rhs) requires Isomorphic<Q, R>
{
    return Q(std::round(lhs.internal() / rhs.internal()) * rhs.internal());
}

/**
 * @brief Calculates the trigonometric sine of an angle
 * @param rhs the angle
 * @return constexpr Number the sine of the angle
 */
template <int F = 0>
constexpr Number<F> sin(const Angle<F>& rhs)
{
    return Number(std::sin(rhs.internal()));
}

/**
 * @brief Calculates the trigonometric cosine of an angle
 * @param rhs the angle
 * @return constexpr Number the cosine of the angle
 */
template <int F = 0>
constexpr Number<F> cos(const Angle<F>& rhs)
{
    return Number(std::cos(rhs.internal()));
}

/**
 * @brief Calculates the trigonometric tangent of an angle
 * @param rhs the angle
 * @return constexpr Number the tangent of the angle
 */
template <int F = 0>
constexpr Number<F> tan(const Angle<F>& rhs)
{
    return Number(std::tan(rhs.internal()));
}

/**
 * @brief Calculates the trigonometric arcsin of a ratio
 * @param rhs the ratio
 * @return the angle
 */
template <isQuantity Q, int F = 0>
constexpr Angle<F> asin(const Q& rhs)
{
    return Angle<F>(std::asin(rhs.internal()));
}

/**
 * @brief Calculates the trigonometric arccosine of a ratio
 * @param rhs the ratio
 * @return the angle
 */
template <isQuantity Q, int F = 0>
constexpr Angle<F> acos(const Q& rhs)
{
    return Angle<F>(std::acos(rhs.internal()));
}

/**
 * @brief Calculates the trigonometric arctangent of a angle
 * @param rhs the ratio
 * @return the angle
 */
template <isQuantity Q, int F = 0>
constexpr Angle<F> atan(const Q& rhs)
{
    return Angle<F>(std::atan(rhs.internal()));
}

/**
 * @brief Calculates the trigonometric arctangent of a ratio
 * @param lhs the y coordinate
 * @param rhs the x coordinate
 * @return the angle
 */
template <isQuantity Q, int F = 0>
constexpr Angle<F> atan2(const Q& lhs, const Q& rhs)
{
    return Angle<F>(std::atan2(lhs.internal(), rhs.internal()));
}
}  // namespace tap::units::math
#endif