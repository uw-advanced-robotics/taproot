#include "robot_type.hpp"

#if defined(TARGET_DRONE)

namespace aruwsrc
{

namespace control
{

/* define subsystems --------------------------------------------------------*/

/* define commands ----------------------------------------------------------*/

/* register subsystems here -------------------------------------------------*/
void registerDroneSubsystems()
{}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultDroneCommands()
{}

/* add any starting commands to the scheduler here --------------------------*/
void startDroneCommands()
{}

/* register io mappings here ------------------------------------------------*/
void registerDroneIoMappings()
{}

void initSubsystemCommands()
{
    registerDroneSubsystems();
    setDefaultDroneCommands();
    startDroneCommands();
    registerDroneIoMappings();
}

}  // namespace control

}  // namespace aruwsrc

#endif
