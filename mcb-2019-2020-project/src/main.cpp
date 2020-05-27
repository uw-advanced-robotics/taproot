#include <aruwlib/rm-dev-board-a/board.hpp>

/* arch includes ------------------------------------------------------------*/
#include <aruwlib/architecture/periodic_timer.hpp>

/* communication includes ---------------------------------------------------*/
#include <aruwlib/motor/dji_motor_tx_handler.hpp>
#include <aruwlib/communication/sensors/mpu6500/mpu6500.hpp>
#include <aruwlib/communication/can/can_rx_listener.hpp>
#include <aruwlib/communication/remote.hpp>
#include <aruwlib/communication/serial/xavier_serial.hpp>
#include <aruwlib/communication/serial/ref_serial.hpp>
#include <aruwlib/display/sh1106.hpp>

/* error handling includes --------------------------------------------------*/
#include <aruwlib/errors/error_controller.hpp>

/* control includes ---------------------------------------------------------*/
#include "aruwsrc/control/robot_control.hpp"
#include <aruwlib/control/command_scheduler.hpp>

using namespace modm::literals;

/* define timers here -------------------------------------------------------*/
aruwlib::arch::PeriodicMilliTimer updateImuPeriod(2);
aruwlib::arch::PeriodicMilliTimer sendMotorTimeout(2);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
void initializeIo();
// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
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
        #ifndef ENV_SIMULATOR

        modm::delayMicroseconds(10);
        #endif
    }
    return 0;
}

void initializeIo()
{
#ifndef ENV_SIMULATOR
    /// \todo this should be an init in the display class
    Board::DisplaySpiMaster::connect<
        Board::DisplayMiso::Miso,
        Board::DisplayMosi::Mosi,
        Board::DisplaySck::Sck
    >();

    // SPI1 is on ABP2 which is at 90MHz; use prescaler 64 to get ~fastest baud rate below 1mHz max
    // 90MHz/64=~14MHz
    Board::DisplaySpiMaster::initialize<Board::SystemClock, 1406250_Hz>();
#endif
    aruwlib::display::Sh1106<
    #ifndef ENV_SIMULATOR
        Board::DisplaySpiMaster,
        Board::DisplayCommand,
        Board::DisplayReset,
    #endif
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
