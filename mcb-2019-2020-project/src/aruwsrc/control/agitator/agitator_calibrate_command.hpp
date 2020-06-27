#ifndef __AGITATOR_CALIBRATE_COMMAND_HPP__
#define __AGITATOR_CALIBRATE_COMMAND_HPP__

#include <aruwlib/control/command.hpp>

#include "agitator_subsystem.hpp"

namespace aruwsrc
{
namespace agitator
{
/**
 * Default command that can be used to calibrate the agitator (i.e. spam calls
 * agitatorCalibrateHere). By default, the agitator will keep calling agitatorCalibrateHere
 * until the agitator is connected, however this command is for the following:
 *  - a placeholder command initially
 *  - allows you to recalibrate an agitator that has already been calibrated if necessary
 */
class AgitatorCalibrateCommand : public aruwlib::control::Command
{
public:
    explicit AgitatorCalibrateCommand(AgitatorSubsystem* agitator);

    const char* getName() const override { return "agitator calibrate command"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

private:
    AgitatorSubsystem* agitator;
};

}  // namespace agitator

}  // namespace aruwsrc

#endif
