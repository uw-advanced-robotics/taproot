#ifndef __CONTIGUOUS_FLOAT_TEST_HPP__
#define __CONTIGUOUS_FLOAT_TEST_HPP__

#include <modm/architecture/interface/assert.hpp>
#include "contiguous_float.hpp"

namespace aruwlib
{

namespace algorithms
{

/**
 * testing code for contiguos float
 */
class ContiguousFloatTest {
 public:
    void testCore() {
        ContiguousFloat testInstance(5, 0, 10);
        modm_assert(5 == testInstance.getValue(),
            "contiguous float", "core test", "wrapping failure");
    }

    void testWrapping() {
        ContiguousFloat testInstance(-4, 0, 10);
        modm_assert(6 == testInstance.getValue(),
            "contiguous float", "Test wrapping #1", "wrapping failure");

        testInstance.setValue(16);
        modm_assert(6 == testInstance.getValue(),
            "contiguous float", "Test wrapping #2", "wrapping failure");

        testInstance.setValue(28);
        modm_assert(8 == testInstance.getValue(),
            "contiguous float", "Test wrapping #3", "wrapping failure");
    }

    void testDifference() {
        ContiguousFloat testInstance(2, 0, 10);
        modm_assert(2 == testInstance.difference(4),
            "contiguous float", "Test difference #1", "wrapping failure");
        modm_assert(-1 == testInstance.difference(11),
            "contiguous float", "Test difference #2", "wrapping failure");

        testInstance.setValue(9);
        modm_assert(2 == testInstance.difference(11),
            "contiguous float", "Test difference #3", "wrapping failure");

        testInstance.setValue(10);
        modm_assert(1 == testInstance.difference(1),
            "contiguous float", "Test difference #4", "wrapping failure");
        testInstance.setValue(1);
        modm_assert(-1 == testInstance.difference(10),
            "contiguous float", "Test difference #5", "wrapping failure");
    }

    void testRotationBounds() {
        ContiguousFloat testInstance(150, -180, 180);

        modm_assert(40 == testInstance.difference(190),
            "contiguous float", "+40", "wrapping failure");
        modm_assert(40 == testInstance.difference(-170),
            "contiguous float", "+40 wrapped", "wrapping failure");

        modm_assert(40 == testInstance.difference(190),
            "contiguous float", "+40", "wrapping failure");
        modm_assert(40 == testInstance.difference(-170),
            "contiguous float", "+40 wrapped", "wrapping failure");

        testInstance.setValue(180);

        modm_assert(180 == testInstance.getValue(),
            "contiguous float", "180", "wrapping failure");
        modm_assert(0 == testInstance.difference(-180),
            "contiguous float", "NoDiff", "wrapping failure");

        ContiguousFloat testInstance2(40, -180, 180);
        modm_assert(-140 == testInstance2.difference(-100),
            "contiguous float", "+40", "wrapping failure");
    }

    void testShiftingValue() {
        ContiguousFloat testInstance(150, -180, 180);
        testInstance.shiftValue(40);
        modm_assert(-170 == testInstance.getValue(),
            "contiguous float", "+40", "wrapping failure");

        testInstance.shiftValue(40);
        modm_assert(-130 == testInstance.getValue(),
            "contiguous float", "+40 again", "wrapping failure");

        testInstance.shiftValue(360);
        modm_assert(-130 == testInstance.getValue(),
            "contiguous float", "+360 again", "wrapping failure");

        testInstance.shiftValue(0);
        modm_assert(-130 == testInstance.getValue(),
            "contiguous float", "+0", "wrapping failure");
    }

    void testBadBounds() {
        ContiguousFloat testInstance(150, 180, -180);
        modm_assert(-180 == testInstance.getLowerBound(),
            "contiguous float", "lower", "wrapping failure");
        modm_assert(180 == testInstance.getUpperBound(),
            "contiguous float", "upper", "wrapping failure");
    }
};

}  // namespace algorithms

}  // namespace aruwlib

#endif
