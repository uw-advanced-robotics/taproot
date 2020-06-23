#include "pwm.hpp"

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/rm-dev-board-a/board.hpp"

using namespace Board;

namespace aruwlib
{
namespace gpio
{
void Pwm::init()
{
#ifndef ENV_SIMULATOR
    Timer8::connect<PWMOutPinW::Ch1, PWMOutPinX::Ch2, PWMOutPinY::Ch3, PWMOutPinZ::Ch4>();
    Timer8::enable();
    Timer8::setMode(Timer8::Mode::UpCounter);

    Timer8::setPrescaler(Board::SystemClock::APB2_PRESCALER);
    Timer8::setOverflow(Board::SystemClock::PWM_RESOLUTION);
#endif
    // Set all out pins to 0 duty
    writeAll(0.0f);

#ifndef ENV_SIMULATOR
    // Start the timer
    Timer8::start();
    Timer8::enableOutput();
#endif
}

void Pwm::writeAll(float duty)
{
#ifndef ENV_SIMULATOR
    write(duty, Pin::W);
    write(duty, Pin::X);
    write(duty, Pin::Y);
    write(duty, Pin::Z);
#endif
}

void Pwm::write(float duty, Pin pin)
{
#ifndef ENV_SIMULATOR
    duty = aruwlib::algorithms::limitVal<float>(duty, 0.0f, 1.0f);
    Timer8::configureOutputChannel(
        static_cast<int>(pin),
        Timer8::OutputCompareMode::Pwm,
        Board::SystemClock::PWM_RESOLUTION * duty);
#endif
}
}  // namespace gpio

}  // namespace aruwlib
