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

#ifndef TAPROOT_CMSIS_MAT_HPP_
#define TAPROOT_CMSIS_MAT_HPP_

#include <array>
#include <cassert>
#include <cinttypes>
#include <iostream>

#include "modm/architecture/utils.hpp"

#include "arm_math.h"

namespace tap::algorithms
{
/**
 * Wraps an `arm_mat_instance_f32` and its associated array.
 * This is done to make it clear which arrays are associated
 * with which `arm_mat_instance_f32` instances.
 */
template <uint16_t ROWS, uint16_t COLS>
struct CMSISMat
{
    std::array<float, ROWS * COLS> data;
    arm_matrix_instance_f32 matrix;

    CMSISMat() : data(), matrix{ROWS, COLS, data.data()} {}

    CMSISMat(const float (&initialData)[ROWS * COLS])
    {
        copyData(initialData);
        arm_mat_init_f32(&matrix, ROWS, COLS, data.data());
    }

    // Delete the copy constructor, create a move constructor. This will
    // Avoid us doing costly copys but will still allow move semantics.
    CMSISMat(const CMSISMat &other) = delete;
    CMSISMat(CMSISMat &&other)
    {
        this->data = std::move(other.data);
        matrix.numRows = ROWS;
        matrix.numCols = COLS;
        matrix.pData = data.data();
    }

    CMSISMat &operator=(CMSISMat &) = delete;
    CMSISMat &operator=(CMSISMat &&other)
    {
        this->data = std::move(other.data);
        matrix.numRows = ROWS;
        matrix.numCols = COLS;
        matrix.pData = data.data();
        return *this;
    }

    inline void copyData(const float (&other)[ROWS * COLS])
    {
        for (size_t i = 0; i < data.size(); i++)
        {
            data[i] = other[i];
        }
    }

    /**
     * Construct identity matrix in the current CMSISMat
     */
    bool constructIdentityMatrix()
    {
        if (ROWS != COLS)
        {
            return false;
        }

        for (int i = 0; i < ROWS; i++)
        {
            for (int j = 0; j < COLS; j++)
            {
                data[i * COLS + j] = (i == j) ? 1 : 0;
            }
        }

        return true;
    }

    inline CMSISMat<COLS, ROWS> inverse()
    {
        CMSISMat<COLS, ROWS> ret;
        assert(ARM_MATH_SUCCESS == arm_mat_inverse_f32(&this->matrix, &ret.matrix));
        return ret;
    }

    inline CMSISMat<COLS, ROWS> transpose()
    {
        CMSISMat<COLS, ROWS> ret;
        assert(ARM_MATH_SUCCESS == arm_mat_transpose_f32(&this->matrix, &ret.matrix));
        return ret;
    }
};

template <uint16_t A_ROWS, uint16_t A_COLS, uint16_t B_ROWS, uint16_t B_COLS>
inline CMSISMat<A_ROWS, A_COLS> operator+(
    const CMSISMat<A_ROWS, A_COLS> &a,
    const CMSISMat<B_ROWS, B_COLS> &b)
{
    static_assert(
        A_ROWS == B_ROWS && A_COLS == B_COLS,
        "Invalid size of CMSISMat matricies in operator+");

    CMSISMat<A_ROWS, A_COLS> c;
    assert(ARM_MATH_SUCCESS == arm_mat_add_f32(&a.matrix, &b.matrix, &c.matrix));
    return c;
}

template <uint16_t A_ROWS, uint16_t A_COLS, uint16_t B_ROWS, uint16_t B_COLS>
inline CMSISMat<A_ROWS, A_COLS> operator-(
    const CMSISMat<A_ROWS, A_COLS> &a,
    const CMSISMat<B_ROWS, B_COLS> &b)
{
    static_assert(
        A_ROWS == B_ROWS && A_COLS == B_COLS,
        "Invalid size of CMSISMat matricies in operator-");

    CMSISMat<A_ROWS, A_COLS> c;
    assert(ARM_MATH_SUCCESS == arm_mat_sub_f32(&a.matrix, &b.matrix, &c.matrix));
    return c;
}

template <uint16_t ROWS, uint16_t COLS>
inline CMSISMat<ROWS, COLS> operator-(const CMSISMat<ROWS, COLS> &a)
{
    float scale(-1);
    CMSISMat<ROWS, COLS> c;
    assert(ARM_MATH_SUCCESS == arm_mat_scale_f32(&a.matrix, &scale, &c.matrix));
    return c;
}

template <uint16_t A_ROWS, uint16_t A_COLS, uint16_t B_ROWS, uint16_t B_COLS>
inline CMSISMat<A_ROWS, B_COLS> operator*(
    const CMSISMat<A_ROWS, A_COLS> &a,
    const CMSISMat<B_ROWS, B_COLS> &b)
{
    static_assert(A_COLS == B_ROWS, "Invalid size of CMSISMat matricies in operator*");

    CMSISMat<A_ROWS, B_COLS> c;
    assert(ARM_MATH_SUCCESS == arm_mat_mult_f32(&a.matrix, &b.matrix, &c.matrix));
    return c;
}

template <uint16_t ROWS, uint16_t COLS>
inline CMSISMat<ROWS, COLS> operator*(const CMSISMat<ROWS, COLS> &a, const float &b)
{
    CMSISMat<ROWS, COLS> c;
    assert(ARM_MATH_SUCCESS == arm_mat_scale_f32(&a.matrix, &b, &c.matrix));
    return c;
}

template <uint16_t ROWS, uint16_t COLS>
inline CMSISMat<ROWS, COLS> operator*(const float &b, const CMSISMat<ROWS, COLS> &a)
{
    CMSISMat<ROWS, COLS> c;
    assert(ARM_MATH_SUCCESS == arm_mat_scale_f32(&a.matrix, &b, &c.matrix));
    return c;
}

}  // namespace tap::algorithms

#endif  // TAPROOT_CMSIS_MAT_HPP_
