#include <rm-dev-board-a/board.hpp>
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
        Board::LedA::set();
    }

    // NOLINTNEXTLINE (see https://github.com/cpplint/cpplint/issues/131)
    void BlinkLEDCommand::end(bool)
    {
        endCounter++;
        Board::LedA::reset();
    }

    bool BlinkLEDCommand::isFinished() const
    {
        return completedTimer.isExpired();
    }
}  // namespace control

}  // namespace aruwsrc
