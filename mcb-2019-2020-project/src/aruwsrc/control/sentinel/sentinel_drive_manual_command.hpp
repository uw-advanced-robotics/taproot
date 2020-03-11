#ifndef __SENTINEL_DRIVE_MANUAL_COMMAND_HPP__
#define __SENTINEL_DRIVE_MANUAL_COMMAND_HPP__


#include "src/aruwlib/control/command.hpp"
#include "sentinel_drive_subsystem.hpp"
#include "modm/processing/timer.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class SentinelDriveSubsystem;

class SentinelDriveManualCommand : public Command
{
 public:
    explicit SentinelDriveManualCommand(SentinelDriveSubsystem* subsystem);

    void initialize();

    void execute();

    void end(bool);

    bool isFinished() const;

    void interrupted();

 private:
    SentinelDriveSubsystem* subsystemSentinelDrive;
};

}  // namespace control

}  // namespace aruwsrc

#endif
