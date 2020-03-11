#include "sentinel_drive_manual_command.hpp"

#include <stdlib.h>

#include "src/aruwlib/control/control_operator_interface.hpp"
#include "sentinel_auto_drive_command.hpp"
#include "sentinel_drive_subsystem.hpp"
#include "src/aruwlib/communication/remote.hpp"

namespace aruwsrc
{

namespace control
{
    SentinelDriveManualCommand::SentinelDriveManualCommand(SentinelDriveSubsystem* subsystem)
        : Command(), subsystemSentinelDrive(subsystem)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
    }

    void SentinelDriveManualCommand::initialize()
    {}

    void SentinelDriveManualCommand::execute()
    {
        subsystemSentinelDrive->setDesiredRpm(ControlOperatorInterface::getSentinelSpeedInput());
    }

    // NOLINTNEXTLINE
    void SentinelDriveManualCommand::end(bool)
    {
        subsystemSentinelDrive->setDesiredRpm(0);
    }

    bool SentinelDriveManualCommand::isFinished() const
    {
        return false;
    }

    void SentinelDriveManualCommand::interrupted()
    {}
}  // namespace control

}  // namespace aruwsrc
