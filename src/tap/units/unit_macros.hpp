/*
 * Copyright (c) 2024-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_UNIT_MACROS_HPP_
#define TAPROOT_UNIT_MACROS_HPP_
#include "quantity.hpp"

#define NEW_UNIT_LITERAL(_name, _qname, _qsuffix, _value)                 \
    namespace constants                                                   \
    {                                                                     \
    template <int F = 0>                                                  \
    constexpr _name<F> _qname = _value;                                   \
    }                                                                     \
    namespace conversions                                                 \
    {                                                                     \
    template <int F = 0>                                                  \
    constexpr inline _name<F> from_##_qname(float value)                  \
    {                                                                     \
        return constants::_qname<F> * value;                              \
    }                                                                     \
    template <int F = 0>                                                  \
    constexpr inline float to_##_qname(_name<F> quantity)                 \
    {                                                                     \
        return quantity.internal() / constants::_qname<F>.internal();     \
    }                                                                     \
    }                                                                     \
    namespace literals                                                    \
    {                                                                     \
    constexpr _name<> operator""_##_qsuffix(long double value)            \
    {                                                                     \
        return constants::_qname<> * static_cast<float>(value);          \
    }                                                                     \
    constexpr _name<> operator""_##_qsuffix(unsigned long long int value) \
    {                                                                     \
        return constants::_qname<> * static_cast<float>(value);          \
    }                                                                     \
    }

#define UNIT_METRIC_PREFIXES_LARGE(_name, _qname, _qsuffix)            \
    NEW_UNIT_LITERAL(_name, kilo##_qname, k##_qsuffix, _qname<F> * 10E3f) \
    NEW_UNIT_LITERAL(_name, mega##_qname, M##_qsuffix, _qname<F> * 10E6f) \
    NEW_UNIT_LITERAL(_name, giga##_qname, G##_qsuffix, _qname<F> * 10E9f) \
    NEW_UNIT_LITERAL(_name, terra##_qname, T##_qsuffix, _qname<F> * 10E12f)

#define UNIT_METRIC_PREFIXES_SMALL(_name, _qname, _qsuffix)              \
    NEW_UNIT_LITERAL(_name, centi##_qname, c##_qsuffix, _qname<F> * 10E-2f) \
    NEW_UNIT_LITERAL(_name, milli##_qname, m##_qsuffix, _qname<F> * 10E-3f) \
    NEW_UNIT_LITERAL(_name, micro##_qname, u##_qsuffix, _qname<F> * 10E-6f) \
    NEW_UNIT_LITERAL(_name, nano##_qname, n##_qsuffix, _qname<F> * 10E-9f)

#define UNIT_METRIC_PREFIXES_ALL(_name, _qname, _qsuffix) \
    UNIT_METRIC_PREFIXES_LARGE(_name, _qname, _qsuffix)   \
    UNIT_METRIC_PREFIXES_SMALL(_name, _qname, _qsuffix)

#define NEW_UNIT(_name, _qname, _qsuffix, _time, _length, _mass, _current, _temperature, _angle) \
    template <int Frame = 0>                                                                     \
    class _name : public Quantity<                                                               \
                      ratio<_time>,                                                              \
                      ratio<_length>,                                                            \
                      ratio<_mass>,                                                              \
                      ratio<_current>,                                                           \
                      ratio<_temperature>,                                                       \
                      ratio<_angle>,                                                             \
                      Frame>                                                                     \
    {                                                                                            \
    public:                                                                                      \
        explicit constexpr _name(float value)                                                    \
            : Quantity<                                                                          \
                  ratio<_time>,                                                                  \
                  ratio<_length>,                                                                \
                  ratio<_mass>,                                                                  \
                  ratio<_current>,                                                               \
                  ratio<_temperature>,                                                           \
                  ratio<_angle>,                                                                 \
                  Frame>(value)                                                                  \
        {                                                                                        \
        }                                                                                        \
        constexpr _name(Quantity<                                                                \
                        ratio<_time>,                                                            \
                        ratio<_length>,                                                          \
                        ratio<_mass>,                                                            \
                        ratio<_current>,                                                         \
                        ratio<_temperature>,                                                     \
                        ratio<_angle>,                                                           \
                        Frame> value)                                                            \
            : Quantity<                                                                          \
                  ratio<_time>,                                                                  \
                  ratio<_length>,                                                                \
                  ratio<_mass>,                                                                  \
                  ratio<_current>,                                                               \
                  ratio<_temperature>,                                                           \
                  ratio<_angle>,                                                                 \
                  Frame>(value)                                                                  \
        {                                                                                        \
        }                                                                                        \
    };                                                                                           \
    template <int F>                                                                             \
    struct lookupName<Quantity<                                                                  \
        ratio<_time>,                                                                            \
        ratio<_length>,                                                                          \
        ratio<_mass>,                                                                            \
        ratio<_current>,                                                                         \
        ratio<_temperature>,                                                                     \
        ratio<_angle>,                                                                           \
        F>>                                                                                      \
    {                                                                                            \
        using Named = _name<F>;                                                                  \
    };                                                                                           \
    NEW_UNIT_LITERAL(_name, _qname, _qsuffix, _name<F>(1.0f))
#endif