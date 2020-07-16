#if defined(TARGET_HERO)

namespace aruwsrc
{
namespace control
{
/* define subsystems --------------------------------------------------------*/

/* define commands ----------------------------------------------------------*/

/* register subsystems here -------------------------------------------------*/
void registerHeroSubsystems() {}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultHeroCommands() {}

/* add any starting commands to the scheduler here --------------------------*/
void startHeroCommands() {}

/* register io mappings here ------------------------------------------------*/
void registerHeroIoMappings() {}

void initSubsystemCommands()
{
    registerHeroSubsystems();
    setDefaultHeroCommands();
    startHeroCommands();
    registerHeroIoMappings();
}

}  // namespace control

}  // namespace aruwsrc

#endif
