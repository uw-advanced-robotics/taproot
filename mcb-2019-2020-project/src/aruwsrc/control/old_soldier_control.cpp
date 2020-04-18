#include "robot_type.hpp"

#if defined(TARGET_OLD_SOLDIER)

namespace aruwsrc
{

namespace control
{

/* define subsystems --------------------------------------------------------*/

/* define commands ----------------------------------------------------------*/

/* register subsystems here -------------------------------------------------*/
void registerOldSoldierSubsystems()
{}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultOldSoldierCommands()
{}

/* add any starting commands to the scheduler here --------------------------*/
void startOldSoldierCommands()
{}

/* register io mappings here ------------------------------------------------*/
void registerOldSoldierIoMappings()
{}

void initSubsystemCommands()
{
    registerOldSoldierSubsystems();
    setDefaultOldSoldierCommands();
    startOldSoldierCommands();
    registerOldSoldierIoMappings();
}

}  // namespace control

}  // namespace aruwsrc

#endif
