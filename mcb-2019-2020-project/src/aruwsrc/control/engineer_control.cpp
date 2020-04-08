#include "robot-type/robot_type.hpp"

#if defined(TARGET_ENGINEER)
namespace aruwsrc
{

namespace control
{

/* define subsystems --------------------------------------------------------*/

/* define commands ----------------------------------------------------------*/

/* register subsystems here -------------------------------------------------*/
void registerEngineerSubsystems()
{}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultEngineerCommands()
{}

/* add any starting commands to the scheduler here --------------------------*/
void startEngineerCommands()
{}

/* register io mappings here ------------------------------------------------*/
void registerEngineerIoMappings()
{}

void initSubsystemCommands()
{
    registerEngineerSubsystems();
    setDefaultEngineerCommands();
    startEngineerCommands();
    registerEngineerIoMappings();
}

}  // namespace control

}  // namespace aruwsrc

#endif
