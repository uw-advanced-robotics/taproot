#include <rm-dev-board-a/board.hpp>
#include <modm/processing/timer.hpp>

/* communication includes ---------------------------------------------------*/
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/aruwlib/communication/serial/xavier_serial.hpp"
#include "src/aruwlib/communication/serial/ref_serial.hpp"
#include "src/aruwlib/display/sh1106.hpp"

/* error handling includes --------------------------------------------------*/
#include "src/aruwlib/errors/error_controller.hpp"

/* control includes ---------------------------------------------------------*/
#include "src/aruwsrc/control/robot_control.hpp"
#include "src/aruwlib/control/command_scheduler.hpp"

/* define timers here -------------------------------------------------------*/
modm::ShortPeriodicTimer updateImuPeriod(2);
modm::ShortPeriodicTimer sendMotorTimeout(2);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
void initializeIo();
// Anything that you would like to be called place here. It will be called
// very frequently. Use ShortPeriodicTimers if you don't want something to be
// called as frequently.
void updateIo();

int main()
{
    Board::initialize();
    initializeIo();
    aruwsrc::control::initSubsystemCommands();

    while (1)
    {
        // do this as fast as you can
        updateIo();

        if (sendMotorTimeout.execute())
        {
            aruwlib::errors::ErrorController::update();
            aruwlib::control::CommandScheduler::getMainScheduler().run();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }
        modm::delayMicroseconds(10);
    }
    return 0;
}

void initializeIo()
{
    /// \todo this should be an init in the display class
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

    aruwlib::Remote::initialize();
    aruwlib::sensors::Mpu6500::init();

    aruwlib::serial::RefSerial::getRefSerial().initialize();
    aruwlib::serial::XavierSerial::getXavierSerial().initialize();
}

void updateIo()
{
    aruwlib::can::CanRxHandler::pollCanData();
    aruwlib::serial::XavierSerial::getXavierSerial().updateSerial();
    aruwlib::serial::RefSerial::getRefSerial().updateSerial();
    aruwlib::Remote::read();
    if (updateImuPeriod.execute())
    {
        aruwlib::sensors::Mpu6500::read();
    }
}
