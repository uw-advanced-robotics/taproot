#include <aruwlib/communication/gpio/leds.hpp>
#include "blink_led_command.hpp"

namespace aruwsrc
{

namespace control
{
    BlinkLEDCommand::BlinkLEDCommand(aruwsrc::control::ExampleSubsystem* subsystem)
    {
        this->addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
    }

    void BlinkLEDCommand::initialize() {
        completedTimer.restart(3000);
        startCounter++;
    }

    void BlinkLEDCommand::execute()
    {
        refershCounter++;
        aruwlib::gpio::Leds::set(aruwlib::gpio::Leds::A, true);
    }

    // NOLINTNEXTLINE (see https://github.com/cpplint/cpplint/issues/131)
    void BlinkLEDCommand::end(bool)
    {
        endCounter++;
        aruwlib::gpio::Leds::set(aruwlib::gpio::Leds::A, false);
    }

    bool BlinkLEDCommand::isFinished() const
    {
        return completedTimer.isExpired();
    }
}  // namespace control

}  // namespace aruwsrc
