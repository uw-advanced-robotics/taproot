#ifndef ANALOG_REMOTE_MAPPING_HPP_
#define ANALOG_REMOTE_MAPPING_HPP_


namespace tap::control
{

class AnalogRemoteMapping
{
public:
    virtual float const getMax() = 0;
    virtual float const getMin() = 0;
    virtual float const getValue() = 0;
};

}

#endif  // ANALOG_REMOTE_MAPPING