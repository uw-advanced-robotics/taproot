#include <modm/platform/random/random_number_generator.hpp>

#include "sentinel_auto_drive_command.hpp"
#include "sentinel_drive_subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    SentinelAutoDriveCommand::SentinelAutoDriveCommand(SentinelDriveSubsystem* subsystem)
        : subsystemSentinelDrive(subsystem), changeVelocityTimer(CHANGE_TIME_INTERVAL)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
        RandomNumberGenerator::enable();
    }

    void SentinelAutoDriveCommand::initialize()
    {
        chosenNewRPM = false;
    }

    void SentinelAutoDriveCommand::execute()
    {
        if (this->changeVelocityTimer.isExpired() || !chosenNewRPM) {
            chosenNewRPM = RandomNumberGenerator::isReady();
            if (chosenNewRPM) {
                this->changeVelocityTimer.restart(CHANGE_TIME_INTERVAL);
                uint32_t randVal = RandomNumberGenerator::getValue();
                currentRPM = randVal % (MAX_RPM - MIN_RPM + 1) + MIN_RPM;
                if (randVal % 2 == 0) {
                    currentRPM *= -1.0f;
                }
            }
        }

        // reverse direction if close to the end of the rail
        float curPos = subsystemSentinelDrive->absolutePosition();
        if ((currentRPM < 0 && curPos < RAIL_BUFFER) ||
            (currentRPM > 0 && curPos > SentinelDriveSubsystem::RAIL_LENGTH - RAIL_BUFFER)) {
            currentRPM = -currentRPM;
        }

        subsystemSentinelDrive->setDesiredRpm(currentRPM);
    }

    // NOLINTNEXTLINE
    void SentinelAutoDriveCommand::end(bool)
    {
        subsystemSentinelDrive->setDesiredRpm(0);
    }

    bool SentinelAutoDriveCommand::isFinished() const
    {
        return false;
    }
}  // namespace control

}  // namespace aruwsrc
