/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include "tap/algorithms/matrix_utils.hpp"

#include "arm_math.h"

using namespace tap::algorithms;

TEST(CMSISMat, default_constructor_matrix_zeroed_cmsis_mat_inited)
{
    CMSISMat<2, 1> m;

    EXPECT_FLOAT_EQ(0, m.data[0]);
    EXPECT_FLOAT_EQ(0, m.data[1]);
    EXPECT_EQ(m.data, m.matrix.pData);
    EXPECT_EQ(2, m.matrix.numRows);
    EXPECT_EQ(1, m.matrix.numCols);
}

TEST(CMSISMat, onearg_constructor_matrix_equal_to_passed_in_matrix)
{
    float arr[2 * 1];
    arr[0] = 10;
    arr[1] = 20;
    CMSISMat<2, 1> m(arr);

    EXPECT_FLOAT_EQ(10, m.data[0]);
    EXPECT_FLOAT_EQ(20, m.data[1]);
    EXPECT_EQ(m.data, m.matrix.pData);
    EXPECT_EQ(2, m.matrix.numRows);
    EXPECT_EQ(1, m.matrix.numCols);

    // Changing arr shouldn't affect stuff stored in m since we copied the
    // data
    arr[0] = 1;
    EXPECT_FLOAT_EQ(10, m.data[0]);
}

TEST(CMSISMat, addop_simple)
{
    CMSISMat<2, 1> a;
    CMSISMat<2, 1> b;

    a.data[0] = 1;
    a.data[1] = 2;

    b.data[0] = 3;
    b.data[1] = 4;

    CMSISMat c = a + b;

    EXPECT_FLOAT_EQ(4, c.data[0]);
    EXPECT_FLOAT_EQ(6, c.data[1]);
}

TEST(CMSISMat, subop_simple)
{
    CMSISMat<2, 1> a;
    CMSISMat<2, 1> b;

    a.data[0] = 1;
    a.data[1] = 2;

    b.data[0] = 3;
    b.data[1] = 4;

    CMSISMat c = a - b;

    EXPECT_FLOAT_EQ(-2, c.data[0]);
    EXPECT_FLOAT_EQ(-2, c.data[1]);
}

TEST(CMSISMat, multop_simple)
{
    CMSISMat<2, 1> a;
    CMSISMat<1, 2> b;

    a.data[0] = 1;
    a.data[1] = 2;

    b.data[0] = 3;
    b.data[1] = 4;

    CMSISMat c = a * b;

    EXPECT_FLOAT_EQ(3, c.data[0]);
    EXPECT_FLOAT_EQ(4, c.data[1]);
    EXPECT_FLOAT_EQ(6, c.data[2]);
    EXPECT_FLOAT_EQ(8, c.data[3]);
}

TEST(CMSISMat, constructIdentityMatrix_false_if_not_square)
{
    CMSISMat<4, 3> a;
    EXPECT_FALSE(a.constructIdentityMatrix());
}

#define FILL_INCR_COUNT(mat)                               \
    for (size_t i = 0; i < MODM_ARRAY_SIZE(mat.data); i++) \
    {                                                      \
        mat.data[i] = i;                                   \
    }

TEST(CMSISMat, constructIdentityMatrix_constructs_correct_identity_mat)
{
    CMSISMat<2, 2> a;

    FILL_INCR_COUNT(a);

    EXPECT_TRUE(a.constructIdentityMatrix());

    EXPECT_FLOAT_EQ(1, a.data[0]);
    EXPECT_FLOAT_EQ(0, a.data[1]);
    EXPECT_FLOAT_EQ(0, a.data[2]);
    EXPECT_FLOAT_EQ(1, a.data[3]);
}

TEST(CMSISMat, copyData_replaces_data)
{
    CMSISMat<3, 2> a;
    float data[3 * 2] = {};

    FILL_INCR_COUNT(a);
    data[1] = 10;

    a.copyData(data);

    for (size_t i = 0; i < MODM_ARRAY_SIZE(data); i++)
    {
        // If 1, check for "10", else check for "0"
        EXPECT_FLOAT_EQ((i == 1 ? 10.0f : 0.0f), a.matrix.pData[i]);
    }
}

TEST(CMSISMat, inverse_simple_test)
{
    CMSISMat<2, 2> a;
    float result[2 * 2];

    a.data[0] = 1;
    a.data[1] = 2;
    a.data[2] = 3;
    a.data[3] = 4;

    result[0] = -2;
    result[1] = 1;
    result[2] = 1.5;
    result[3] = -0.5;

    CMSISMat res = a.inverse();

    for (size_t i = 0; i < MODM_ARRAY_SIZE(result); i++)
    {
        EXPECT_FLOAT_EQ(result[i], res.data[i]);
    }
}
