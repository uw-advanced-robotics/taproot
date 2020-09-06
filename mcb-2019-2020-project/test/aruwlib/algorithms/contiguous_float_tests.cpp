#include <aruwlib/algorithms/contiguous_float.hpp>
#include <gtest/gtest.h>

TEST(ContiguousFloat, Basic_functionality)
{
    aruwlib::algorithms::ContiguousFloat testInstance(5, 0, 10);
    EXPECT_EQ(5, testInstance.getValue());
}

TEST(ContiguousFloat, Wrapping_behavior)
{
    aruwlib::algorithms::ContiguousFloat testInstance(-4, 0, 10);
    EXPECT_EQ(6, testInstance.getValue());

    testInstance.setValue(16);
    EXPECT_EQ(6, testInstance.getValue());

    testInstance.setValue(28);
    EXPECT_EQ(8, testInstance.getValue());
}

TEST(ContiguousFloat, Difference)
{
    aruwlib::algorithms::ContiguousFloat testInstance(2, 0, 10);
    EXPECT_EQ(2, testInstance.difference(4));
    EXPECT_EQ(-1, testInstance.difference(11));

    testInstance.setValue(9);
    EXPECT_EQ(2, testInstance.difference(11));

    testInstance.setValue(10);
    EXPECT_EQ(1, testInstance.difference(1));
    testInstance.setValue(1);
    EXPECT_EQ(-1, testInstance.difference(10));
}

TEST(ContiguousFloat, Rotation_bounds)
{
    aruwlib::algorithms::ContiguousFloat testInstance(150, -180, 180);

    EXPECT_EQ(40, testInstance.difference(190));
    EXPECT_EQ(40, testInstance.difference(-170));

    EXPECT_EQ(40, testInstance.difference(190));
    EXPECT_EQ(40, testInstance.difference(-170));

    testInstance.setValue(180);

    EXPECT_EQ(180, testInstance.getValue());
    EXPECT_EQ(0, testInstance.difference(-180));

    aruwlib::algorithms::ContiguousFloat testInstance2(40, -180, 180);
    EXPECT_EQ(-140, testInstance2.difference(-100));
}

TEST(ContiguousFloat, Shifting_value)
{
    aruwlib::algorithms::ContiguousFloat testInstance(150, -180, 180);

    testInstance.shiftValue(40);
    EXPECT_EQ(-170, testInstance.getValue());

    testInstance.shiftValue(40);
    EXPECT_EQ(-130, testInstance.getValue());

    testInstance.shiftValue(360);
    EXPECT_EQ(-130, testInstance.getValue());

    testInstance.shiftValue(0);
    EXPECT_EQ(-130, testInstance.getValue());
}

TEST(ContiguousFloat, Bad_bounds)
{
    aruwlib::algorithms::ContiguousFloat testInstance(150, 180, -180);
    EXPECT_EQ(-180, testInstance.getLowerBound());
    EXPECT_EQ(180, testInstance.getUpperBound());
}