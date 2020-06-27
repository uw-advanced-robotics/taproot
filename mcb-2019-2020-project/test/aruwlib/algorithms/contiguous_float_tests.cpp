#include <aruwlib/algorithms/contiguous_float.hpp>

#include "catch/catch.hpp"

using namespace aruwlib::algorithms;

TEST_CASE("ContiguousFloat: Basic functionality", "[contiguous_float]")
{
    ContiguousFloat testInstance(5, 0, 10);
    REQUIRE(5 == testInstance.getValue());
}

TEST_CASE("ContiguousFloat: Wrapping behavior", "[contiguous_float]")
{
    ContiguousFloat testInstance(-4, 0, 10);
    REQUIRE(6 == testInstance.getValue());

    testInstance.setValue(16);
    REQUIRE(6 == testInstance.getValue());

    testInstance.setValue(28);
    REQUIRE(8 == testInstance.getValue());
}

TEST_CASE("ContiguousFloat: Difference", "[contiguous_float]")
{
    ContiguousFloat testInstance(2, 0, 10);
    REQUIRE(2 == testInstance.difference(4));
    REQUIRE(-1 == testInstance.difference(11));

    testInstance.setValue(9);
    REQUIRE(2 == testInstance.difference(11));

    testInstance.setValue(10);
    REQUIRE(1 == testInstance.difference(1));
    testInstance.setValue(1);
    REQUIRE(-1 == testInstance.difference(10));
}

TEST_CASE("ContiguousFloat: Rotation bounds", "[contiguous_float]")
{
    ContiguousFloat testInstance(150, -180, 180);

    REQUIRE(40 == testInstance.difference(190));
    REQUIRE(40 == testInstance.difference(-170));

    REQUIRE(40 == testInstance.difference(190));
    REQUIRE(40 == testInstance.difference(-170));

    testInstance.setValue(180);

    REQUIRE(180 == testInstance.getValue());
    REQUIRE(0 == testInstance.difference(-180));

    ContiguousFloat testInstance2(40, -180, 180);
    REQUIRE(-140 == testInstance2.difference(-100));
}

TEST_CASE("ContiguousFloat: Shifting value", "[contiguous_float]")
{
    ContiguousFloat testInstance(150, -180, 180);

    testInstance.shiftValue(40);
    REQUIRE(-170 == testInstance.getValue());

    testInstance.shiftValue(40);
    REQUIRE(-130 == testInstance.getValue());

    testInstance.shiftValue(360);
    REQUIRE(-130 == testInstance.getValue());

    testInstance.shiftValue(0);
    REQUIRE(-130 == testInstance.getValue());
}

TEST_CASE("ContiguousFloat: Bad bounds", "[contiguous_float]")
{
    ContiguousFloat testInstance(150, 180, -180);
    REQUIRE(-180 == testInstance.getLowerBound());
    REQUIRE(180 == testInstance.getUpperBound());
}