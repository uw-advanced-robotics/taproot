#ifndef TAPROOT_UNITS_HPP_
#define TAPROOT_UNITS_HPP_
#include "unit_macros.hpp"

namespace tap::units
{
// Number
NEW_UNIT(Number, number, n, 0, 0, 0, 0, 0, 0)
NEW_UNIT_LITERAL(Number, percent, pct, 0.01f);

// Time, Frequency
NEW_UNIT(Time, second, s, 1, 0, 0, 0, 0, 0)
UNIT_METRIC_PREFIXES_SMALL(Time, second, s)
NEW_UNIT_LITERAL(Time, minute, min, second<F> * 60.0f)
NEW_UNIT_LITERAL(Time, hour, min, minute<F> * 60.0f)
NEW_UNIT_LITERAL(Time, day, min, hour<F> * 24.0f)

NEW_UNIT(Frequency, hertz, Hz, -1, 0, 0, 0, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Frequency, hertz, Hz)

// Length, Area, Volume
NEW_UNIT(Length, meter, m, 0, 1, 0, 0, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Length, meter, m)
NEW_UNIT_LITERAL(Length, inch, in, centimeter<F> * 2.54f)
NEW_UNIT_LITERAL(Length, foot, ft, inch<F> * 12.0f)

NEW_UNIT(Area, square_meter, m2, 0, 2, 0, 0, 0, 0)
NEW_UNIT_LITERAL(Area, square_inch, in2, inch<F>* inch<F>)
NEW_UNIT_LITERAL(Area, square_foot, ft2, foot<F>* foot<F>)

NEW_UNIT(Volume, cubic_meter, m3, 0, 3, 0, 0, 0, 0)
NEW_UNIT_LITERAL(Volume, cubic_inch, in3, inch<F>* inch<F>* inch<F>)
NEW_UNIT_LITERAL(Volume, cubic_foot, ft3, foot<F>* foot<F>* foot<F>)
NEW_UNIT_LITERAL(Volume, cubic_yard, yd3, yard<F>* yard<F>* yard<F>)
NEW_UNIT_LITERAL(Volume, liter, L, cubic_meter<F> * 0.001f)

// Mass, Inertia
NEW_UNIT(Mass, kilogram, kg, 0, 0, 1, 0, 0, 0)
NEW_UNIT_LITERAL(Mass, gram, g, kilogram<F> * 10E-3f)
UNIT_METRIC_PREFIXES_SMALL(Mass, gram, g)
NEW_UNIT_LITERAL(Mass, pound, lb, gram<F> * 453.59237f)

NEW_UNIT(Inertia, kilogram_meter_squared, kgm2, 0, 2, 1, 0, 0, 0)

// Current, Charge, Voltage
NEW_UNIT(Current, amp, A, 0, 0, 0, 1, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Current, amp, A)

NEW_UNIT(Charge, coulomb, C, 1, 0, 0, 1, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Charge, coulomb, C)

NEW_UNIT(Voltage, volt, V, 2, 1, 1, -1, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Voltage, volt, V)

// Angle
NEW_UNIT(Angle, radian, rad, 0, 0, 0, 0, 0, 1)
NEW_UNIT_LITERAL(Angle, degree, deg, radian<F> * 180.0f / M_PI_F)
NEW_UNIT_LITERAL(Angle, rotation, rot, radian<F>* M_2PI_F)

// Linear Velocity, Acceleration, Jerk
NEW_UNIT(LinearVelocity, meter_per_second, mps, -1, 1, 0, 0, 0, 0)
NEW_UNIT_LITERAL(LinearVelocity, foot_per_second, fps, foot<F> / second<F>)
NEW_UNIT_LITERAL(LinearVelocity, mile_per_hour, mph, mile<F> / hour<F>)
NEW_UNIT_LITERAL(LinearVelocity, kilometer_per_hour, kmph, kilometer<F> / hour<F>)
NEW_UNIT_LITERAL(LinearVelocity, knot, knot, nautical_mile<F> / hour<F>)

NEW_UNIT(LinearAcceleration, meter_per_second_squared, mps2, -2, 1, 0, 0, 0, 0)
NEW_UNIT_LITERAL(
    LinearAcceleration,
    foot_per_second_squared,
    fps2,
    foot<F> / (second<F> * second<F>))

NEW_UNIT(LinearJerk, meter_per_second_cubed, mps3, -3, 1, 0, 0, 0, 0)
NEW_UNIT_LITERAL(
    LinearJerk,
    foot_per_second_cubed,
    fps3,
    foot<F> / (second<F> * second<F> * second<F>))

// Angular Velocity, Acceleration, Jerk
NEW_UNIT(AngularVelocity, radian_per_second, rps, -1, 0, 0, 0, 0, 1)
NEW_UNIT_LITERAL(AngularVelocity, rpm, rpm, rotation<F> / minute<F>)

NEW_UNIT(AngularAcceleration, radian_per_second_squared, rps2, -2, 0, 0, 0, 0, 1)
NEW_UNIT_LITERAL(AngularAcceleration, rpm2, rpm2, rotation<F> / (minute<F> * minute<F>))

NEW_UNIT(AngularJerk, radian_per_second_cubed, rps3, -3, 0, 0, 0, 0, 1)
NEW_UNIT_LITERAL(AngularJerk, rpm3, rpm3, rotation<F> / (minute<F> * minute<F> * minute<F>))

// Force, Pressure, Momentum, Impulse, Energy, Power
NEW_UNIT(Force, newton, N, -2, 1, 1, 0, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Force, newton, N)

NEW_UNIT(Pressure, pascal, Pa, -1, -1, 1, 0, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Pressure, pascal, Pa)

NEW_UNIT(Momentum, newton_second, Ns, -1, 1, 1, 0, 0, 0)
template <int F = 0>
using Impulse = Momentum<F>;

NEW_UNIT(Energy, joule, J, -2, 2, 1, 0, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Energy, joule, J)

NEW_UNIT(Power, watt, W, -2, 2, 1, 0, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Power, watt, W)

// Radius, Curvature
NEW_UNIT(Radius, meter_per_radian, mprad, 0, 1, 0, 0, 0, -1)

NEW_UNIT(Curvature, radian_per_meter, radpm, 0, -1, 0, 0, 0, 1)

}  // namespace tap::units
#endif  // TAPROOT_UNITS_HPP_