#include "tap/control/command_mapping/boolean_input.hpp"
#include "tap/control/command_mapping/analog_input.hpp"
#include "tap/communication/serial/remote.hpp"


namespace tap::control::command_mapping
{

class CommandInput
{
public:
    CommandInput(
        tap::communication::serial::Remote& remote,
        BooleanInput& trigger,
        AnalogInput& inputValue,
        float defaultValue = 0)
        : remote(remote),
          trigger(trigger),
          inputValue(inputValue),
          defaultValue(defaultValue)
    {

    }

    float getValue() const
    {
        if (this->trigger.triggered(this->remote))
        {
            return this->inputValue.getValue(this->remote);
        }
        return this->defaultValue;
    }
private:
    tap::communication::serial::Remote& remote;
    BooleanInput& trigger;
    AnalogInput& inputValue;
    float defaultValue;
};
}