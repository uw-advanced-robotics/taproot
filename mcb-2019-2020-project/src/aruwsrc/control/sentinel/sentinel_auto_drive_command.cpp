#ifndef ENV_SIMULATOR
#include <modm/platform/random/random_number_generator.hpp>
#endif

#include "sentinel_auto_drive_command.hpp"
#include "sentinel_drive_subsystem.hpp"

#ifndef ENV_SIMULATOR
using modm::platform::RandomNumberGenerator;
#endif
using aruwlib::control::Subsystem;

namespace aruwsrc
{
namespace control
{
SentinelAutoDriveCommand::SentinelAutoDriveCommand(SentinelDriveSubsystem* subsystem)
    : subsystemSentinelDrive(subsystem),
      changeVelocityTimer(CHANGE_TIME_INTERVAL)
{
    addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
#ifndef ENV_SIMULATOR
    RandomNumberGenerator::enable();
#endif
}

void SentinelAutoDriveCommand::initialize() { chosenNewRPM = false; }

void SentinelAutoDriveCommand::execute()
{
    if (this->changeVelocityTimer.isExpired() || !chosenNewRPM)
    {
#ifdef ENV_SIMULATOR
        chosenNewRPM = true;
#else
        chosenNewRPM = RandomNumberGenerator::isReady();
#endif
        if (chosenNewRPM)
        {
            this->changeVelocityTimer.restart(CHANGE_TIME_INTERVAL);
#ifdef ENV_SIMULATOR
            currentRPM = MIN_RPM + (MAX_RPM - MIN_RPM) / 2;
#else
            uint32_t randVal = RandomNumberGenerator::getValue();
            currentRPM = randVal % (MAX_RPM - MIN_RPM + 1) + MIN_RPM;
            if (randVal % 2 == 0)
            {
                currentRPM *= -1.0f;
            }
#endif
        }
    }

    // reverse direction if close to the end of the rail
    float curPos = subsystemSentinelDrive->absolutePosition();
    if ((currentRPM < 0 && curPos < RAIL_BUFFER) ||
        (currentRPM > 0 && curPos > SentinelDriveSubsystem::RAIL_LENGTH - RAIL_BUFFER))
    {
        currentRPM = -currentRPM;
    }

    subsystemSentinelDrive->setDesiredRpm(currentRPM);
}

void SentinelAutoDriveCommand::end(bool) { subsystemSentinelDrive->setDesiredRpm(0); }

bool SentinelAutoDriveCommand::isFinished() const { return false; }
}  // namespace control

}  // namespace aruwsrc
