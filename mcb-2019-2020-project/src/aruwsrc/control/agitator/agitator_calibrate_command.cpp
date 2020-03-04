#include "agitator_calibrate_command.hpp"
#include "src/aruwlib/control/subsystem.hpp"

namespace aruwsrc
{

namespace agitator
{
    AgitatorCalibrateCommand::AgitatorCalibrateCommand(AgitatorSubsystem* agitator) :
    agitator(agitator)
    {
        this->addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(agitator));
    }

    void AgitatorCalibrateCommand::initialize()
    {
        agitator->agitatorCalibrateHere();
    }

    void AgitatorCalibrateCommand::execute()
    {
        agitator->agitatorCalibrateHere();
    }

    // NOLINTNEXTLINE
    void AgitatorCalibrateCommand::end(bool)
    {}

    bool AgitatorCalibrateCommand::isFinished() const
    {
        return agitator->isAgitatorCalibrated();
    }
}  // namespace agitator

}  // namespace aruwsrc
