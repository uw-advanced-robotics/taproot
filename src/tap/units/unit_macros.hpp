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