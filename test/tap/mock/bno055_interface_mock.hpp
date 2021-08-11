#ifndef BNO055_INTERFACE_MOCK_HPP_
#define BNO055_INTERFACE_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/communication/sensors/bno055/bno055_interface.hpp"

namespace tap
{
namespace mock
{
class Bno055InterfaceMock : public sensors::Bno055Interface
{
public:
    Bno055InterfaceMock();
    virtual ~Bno055InterfaceMock();
    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(bool, update, (), (override));
};
}  // namespace mock
}  // namespace tap

#endif  // BNO055_INTERFACE_MOCK_HPP_
