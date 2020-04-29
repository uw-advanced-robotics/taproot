#include "aruwlib/rm-dev-board-a/board.hpp"
#include "pwm.hpp"

using namespace Board;

namespace aruwlib
{

namespace gpio
{
    void Pwm::init()
    {
        #ifndef ENV_SIMULATOR
        Timer8::connect<PWMOutPinW::Ch1, PWMOutPinX::Ch2,
                        PWMOutPinY::Ch3, PWMOutPinZ::Ch4>();
        Timer8::enable();
        Timer8::setMode(Timer8::Mode::UpCounter);

        Timer8::setPrescaler(Board::SystemClock::APB2_PRESCALER);
        Timer8::setOverflow(Board::SystemClock::PWM_RESOLUTION);
        #endif
        // Set all out pins to 0 duty
        WriteAll(0.0f);

        #ifndef ENV_SIMULATOR
        // Start the timer
        Timer8::start();
        Timer8::enableOutput();
        #endif
    }

    /*
     * Sets all Timer channels to the same duty
     */
    void Pwm::WriteAll(double duty) {
        #ifndef ENV_SIMULATOR
        Timer8::configureOutputChannel(1, Timer8::OutputCompareMode::Pwm,
            Board::SystemClock::PWM_RESOLUTION * duty);
        Timer8::configureOutputChannel(2, Timer8::OutputCompareMode::Pwm,
            Board::SystemClock::PWM_RESOLUTION * duty);
        Timer8::configureOutputChannel(3, Timer8::OutputCompareMode::Pwm,
            Board::SystemClock::PWM_RESOLUTION * duty);
        Timer8::configureOutputChannel(4, Timer8::OutputCompareMode::Pwm,
            Board::SystemClock::PWM_RESOLUTION * duty);
        #endif
    }

    /*
     * Sets the PWM duty for a specified pin
     */
    void Pwm::Write(double duty, Pin pin) {
        #ifndef ENV_SIMULATOR
        Timer8::configureOutputChannel(static_cast<int>(pin),
        Timer8::OutputCompareMode::Pwm, Board::SystemClock::PWM_RESOLUTION * duty);
        #endif
    }
}  // namespace gpio

}  // namespace aruwlib
