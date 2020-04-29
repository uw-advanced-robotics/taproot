#ifndef __COMMAND_SENTINEL_DRIVE_RANDOM_HPP__
#define __COMMAND_SENTINEL_DRIVE_RANDOM_HPP__

#include <aruwlib/control/command.hpp>
#include <aruwlib/architecture/timeout.hpp>
#include "sentinel_drive_subsystem.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class SentinelDriveSubsystem;

class SentinelAutoDriveCommand : public Command
{
 public:
    explicit SentinelAutoDriveCommand(SentinelDriveSubsystem* subsystem);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

 private:
    static const int16_t MIN_RPM = 5000;
    static const int16_t MAX_RPM = 7000;
    static const int16_t CHANGE_TIME_INTERVAL = 750;
    static constexpr float RAIL_BUFFER = 0.1f * SentinelDriveSubsystem::RAIL_LENGTH;

    float currentRPM = 0;
    bool chosenNewRPM = false;

    SentinelDriveSubsystem* subsystemSentinelDrive;
    aruwlib::arch::MilliTimeout changeVelocityTimer;
};

}  // namespace control

}  // namespace aruwsrc

#endif
