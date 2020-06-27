#include "blink_led_command.hpp"

#include <aruwlib/Drivers.hpp>
#include <aruwlib/control/subsystem.hpp>

using aruwlib::Drivers;

namespace aruwsrc
{
namespace control
{
BlinkLEDCommand::BlinkLEDCommand(aruwsrc::control::ExampleSubsystem* subsystem)
{
    this->addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(subsystem));
}

void BlinkLEDCommand::initialize()
{
    completedTimer.restart(3000);
    startCounter++;
}

void BlinkLEDCommand::execute()
{
    refershCounter++;
    Drivers::leds.set(aruwlib::gpio::Leds::A, true);
}

bool BlinkLEDCommand::isFinished() const { return completedTimer.isExpired(); }
}  // namespace control

}  // namespace aruwsrc
