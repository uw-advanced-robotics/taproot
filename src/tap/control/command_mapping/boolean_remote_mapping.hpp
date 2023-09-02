#ifndef BOOLEAN_REMOTE_MAPPING_HPP_
#define BOOLEAN_REMOTE_MAPPING_HPP_

namespace tap::control::command_mapping
{

class BooleanRemoteMapping
{
public:
    virtual bool isActive() = 0;
};

}

#endif