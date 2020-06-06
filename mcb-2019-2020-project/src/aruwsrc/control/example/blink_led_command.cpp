#include <aruwlib/Drivers.hpp>
#include <aruwlib/control/subsystem.hpp>
#include "blink_led_command.hpp"

using aruwlib::Drivers;

namespace aruwsrc
{

namespace control
{
    BlinkLEDCommand::BlinkLEDCommand(aruwsrc::control::ExampleSubsystem* subsystem)
    {
        this->addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(subsystem));
    }

    void BlinkLEDCommand::initialize() {
        completedTimer.restart(3000);
        startCounter++;
    }

    void BlinkLEDCommand::execute()
    {
        refershCounter++;
        Drivers::leds.set(aruwlib::gpio::Leds::A, true);
    }

    // NOLINTNEXTLINE (see https://github.com/cpplint/cpplint/issues/131)
    void BlinkLEDCommand::end(bool)
    {
        endCounter++;
        Drivers::leds.set(aruwlib::gpio::Leds::A, false);
    }

    bool BlinkLEDCommand::isFinished() const
    {
        return completedTimer.isExpired();
    }
}  // namespace control

}  // namespace aruwsrc
