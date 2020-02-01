#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
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

using namespace aruwsrc::chassis;

#if defined(TARGET_SOLDIER)
ChassisSubsystem soldierChassis;
#else  // error
#error "select soldier robot type only"
#endif

aruwlib::serial::RefSerial refereeSerial;

using namespace aruwlib::sensors;

int main()
{
    aruwlib::algorithms::ContiguousFloatTest contiguousFloatTest;
    contiguousFloatTest.testCore();
    contiguousFloatTest.testBadBounds();
    contiguousFloatTest.testDifference();
    contiguousFloatTest.testRotationBounds();
    contiguousFloatTest.testShiftingValue();
    contiguousFloatTest.testWrapping();

    Board::initialize();
    aruwlib::Remote::initialize();

    refereeSerial.initialize();

    Mpu6500::init();

    #if defined(TARGET_SOLDIER)  // only soldier has the proper constants in for chassis code
    modm::SmartPointer chassisDrive(new ChassisDriveCommand(&soldierChassis));
    CommandScheduler::registerSubsystem(&soldierChassis);
    soldierChassis.setDefaultCommand(chassisDrive);
    #endif

    // timers
    // arbitrary, taken from last year since this send time doesn't overfill
    // can bus
    modm::ShortPeriodicTimer motorSendPeriod(3);
    // update imu
    modm::ShortPeriodicTimer updateImuPeriod(2);

    while (1)
    {
        // do this as fast as you can
        aruwlib::can::CanRxHandler::pollCanData();
        refereeSerial.updateSerial();

        aruwlib::Remote::read();

        if (updateImuPeriod.execute())
        {
            Mpu6500::read();
        }

        if (motorSendPeriod.execute())
        {
            aruwlib::control::CommandScheduler::run();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        modm::delayMicroseconds(10);
    }
    return 0;
}
