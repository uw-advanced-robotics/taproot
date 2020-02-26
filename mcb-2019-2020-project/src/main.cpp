#include <rm-dev-board-a/board.hpp>
#include <modm/processing/timer.hpp>

#include "src/aruwlib/control/controller_mapper.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "src/aruwlib/control/command_scheduler.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"
#include "src/aruwlib/algorithms/contiguous_float_test.hpp"
#include "src/aruwlib/communication/serial/ref_serial.hpp"
#include "src/aruwsrc/control/turret/turret_subsystem.hpp"
#include "src/aruwsrc/control/turret/turret_cv_command.hpp"
#include "src/aruwsrc/control/turret/turret_init_command.hpp"
#include "src/aruwsrc/control/turret/turret_manual_command.hpp"
#include "src/aruwsrc/control/example/example_comprised_command.hpp"
#include "src/aruwlib/errors/error_controller.hpp"
#include "src/aruwlib/communication/serial/xavier_serial.hpp"
#include "src/aruwlib/display/sh1106.hpp"

using namespace aruwsrc::chassis;
using namespace aruwsrc::control;
using namespace aruwlib::sensors;

#if defined(TARGET_SOLDIER)
TurretSubsystem turretSubsystem;
TurretCVCommand turretCVCommand(&turretSubsystem);
TurretInitCommand turretInitCommand(&turretSubsystem);
TurretManualCommand turretManualCommand(&turretSubsystem);

ChassisSubsystem soldierChassis;
ChassisDriveCommand chassisDriveCommand(&soldierChassis);
#else  // error
#error "select soldier robot type only"
#endif


int main()
{
    Board::initialize();

    Board::DisplaySpiMaster::connect<
        Board::DisplayMiso::Miso,
        Board::DisplayMosi::Mosi,
        Board::DisplaySck::Sck
    >();

    // SPI1 is on ABP2 which is at 90MHz; use prescaler 64 to get ~fastest baud rate below 1mHz max
    // 90MHz/64=~14MHz
    Board::DisplaySpiMaster::initialize<Board::SystemClock, 1406250_Hz>();

    aruwlib::display::Sh1106<
        Board::DisplaySpiMaster,
        Board::DisplayCommand,
        Board::DisplayReset,
        128, 64,
        false
    > display;
    display.initializeBlocking();
    display.setCursor(2, 1);
    display.setFont(modm::font::ScriptoNarrow);
    display << "ur code is shit" << modm::endl;
    display.update();

    aruwlib::algorithms::ContiguousFloatTest contiguousFloatTest;
    contiguousFloatTest.testCore();
    contiguousFloatTest.testBadBounds();
    contiguousFloatTest.testDifference();
    contiguousFloatTest.testRotationBounds();
    contiguousFloatTest.testShiftingValue();
    contiguousFloatTest.testWrapping();

    aruwlib::Remote::initialize();

    aruwlib::serial::RefSerial::getRefSerial().initialize();
    aruwlib::serial::XavierSerial::getXavierSerial().initialize();

    Mpu6500::init();

    #if defined(TARGET_SOLDIER)  // only soldier has the proper constants in for chassis code
    CommandScheduler::getMainScheduler().registerSubsystem(&soldierChassis);
    soldierChassis.setDefaultCommand(&chassisDriveCommand);

    CommandScheduler::getMainScheduler().registerSubsystem(&turretSubsystem);
    turretSubsystem.setDefaultCommand(&turretManualCommand);
    IoMapper::addHoldMapping(
        IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP, {}),
        &turretCVCommand);
    CommandScheduler::getMainScheduler().addCommand(&turretInitCommand);
    #endif

    // timers
    // arbitrary, taken from last year since this send time doesn't overfill
    // can bus
    modm::ShortPeriodicTimer motorSendPeriod(2);
    // update imu
    modm::ShortPeriodicTimer updateImuPeriod(2);

    while (1)
    {
        // do this as fast as you can
        aruwlib::can::CanRxHandler::pollCanData();
        aruwlib::serial::XavierSerial::getXavierSerial().updateSerial();
        aruwlib::serial::RefSerial::getRefSerial().updateSerial();

        aruwlib::Remote::read();

        if (updateImuPeriod.execute())
        {
            Mpu6500::read();
        }

        if (motorSendPeriod.execute())
        {
            aruwlib::errors::ErrorController::update();
            CommandScheduler::getMainScheduler().run();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        modm::delayMicroseconds(10);
    }
    return 0;
}
