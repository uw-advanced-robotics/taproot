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

#ifndef PLATFORM_HOSTED
#include "arm_math.h"
#else
struct arm_matrix_instance_f32
{
    uint16_t numRows;
    uint16_t numCols;
    float *pData;
};
#endif

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
        matrix.numRows = ROWS;
        matrix.numCols = COLS;
        matrix.pData = data.data();
    }

    /**
     * Deep copy. Costly; use std::move to invoke move constructor whenever possible.
     */
    CMSISMat(const CMSISMat &other) : matrix{ROWS, COLS, data.data()}
    {
        memcpy(&this->data, &other.data, sizeof(this->data));
    }

    // Move semantics.
    CMSISMat(CMSISMat &&other)
    {
        this->data = std::move(other.data);
        matrix.numRows = ROWS;
        matrix.numCols = COLS;
        matrix.pData = data.data();
    }

    /**
     * Deep copy. Costly; use std::move to invoke move assignment whenever possible.
     */
    CMSISMat &operator=(const CMSISMat &other)
    {
        memcpy(&this->data, &other.data, sizeof(this->data));
        return *this;
    }

    // Move semantics.
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
     * Construct identity matrix in the current CMSISMat.
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

    inline CMSISMat<COLS, ROWS> inverse() const
    {
        CMSISMat<COLS, ROWS> ret;
#ifndef PLATFORM_HOSTED
        assert(ARM_MATH_SUCCESS == arm_mat_inverse_f32(&this->matrix, &ret.matrix));
#else
        ret.constructIdentityMatrix();
        CMSISMat<ROWS, COLS> modified(*this);
        float temp;

        for (int row = 0; row < ROWS; row++)
        {
            temp = modified.data[row * ROWS + row];
            for (int col = 0; col < COLS; col++)
            {
                modified.data[row * ROWS + col] /= temp;
                ret.data[row * ROWS + col] /= temp;
            }

            for (int otherRow = 0; otherRow < ROWS; otherRow++)
            {
                if (row == otherRow) continue;
                temp = modified.data[otherRow * ROWS + row];
                for (int col = 0; col < COLS; col++)
                {
                    modified.data[otherRow * ROWS + col] -= temp * modified.data[row * ROWS + col];
                    ret.data[otherRow * ROWS + col] -= temp * ret.data[row * ROWS + col];
                }
            }
        }

        // for i_maj in range(modify.shape[0]):
        // temp = modify[i_maj, i_maj]
        // for j in range(modify.shape[1]):
        //     modify[i_maj, j] /= temp
        //     ret[i_maj, j] /= temp

        // for i in range(modify.shape[0]):
        //     if i == i_maj:
        //         continue
        //     temp = modify[i, i_maj]
        //     for j in range(modify.shape[1]):
        //         modify[i, j] -= temp * modify[i_maj, j]
        //         ret[i, j] -= temp * ret[i_maj, j]
#endif
        return ret;
    }

    inline CMSISMat<COLS, ROWS> transpose() const
    {
        CMSISMat<COLS, ROWS> ret;
#ifndef PLATFORM_HOSTED
        assert(ARM_MATH_SUCCESS == arm_mat_trans_f32(&this->matrix, &ret.matrix));
#else
        for (int i = 0; i < ROWS; i++)
        {
            for (int j = 0; j < COLS; j++)
            {
                ret.data[i * ROWS + j] = data[j * ROWS + i];
            }
        }
#endif
        return ret;
    }
};

/* Begin definitions */
#ifndef PLATFORM_HOSTED
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
    assert(ARM_MATH_SUCCESS == arm_mat_scale_f32(&a.matrix, scale, &c.matrix));
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
inline CMSISMat<ROWS, COLS> operator*(const CMSISMat<ROWS, COLS> &a, float b)
{
    CMSISMat<ROWS, COLS> c;
    assert(ARM_MATH_SUCCESS == arm_mat_scale_f32(&a.matrix, b, &c.matrix));
    return c;
}

template <uint16_t ROWS, uint16_t COLS>
inline CMSISMat<ROWS, COLS> operator*(float b, const CMSISMat<ROWS, COLS> &a)
{
    CMSISMat<ROWS, COLS> c;
    assert(ARM_MATH_SUCCESS == arm_mat_scale_f32(&a.matrix, b, &c.matrix));
    return c;
}

template <uint16_t ROWS, uint16_t COLS>
inline CMSISMat<ROWS, COLS> operator/(const CMSISMat<ROWS, COLS> &a, float b)
{
    b = 1 / b;
    CMSISMat<ROWS, COLS> c;
    assert(ARM_MATH_SUCCESS == arm_mat_scale_f32(&a.matrix, b, &c.matrix));
    return c;
}
#else
template <uint16_t A_ROWS, uint16_t A_COLS, uint16_t B_ROWS, uint16_t B_COLS>
inline CMSISMat<A_ROWS, A_COLS> operator+(
    const CMSISMat<A_ROWS, A_COLS> &a,
    const CMSISMat<B_ROWS, B_COLS> &b)
{
    static_assert(
        A_ROWS == B_ROWS && A_COLS == B_COLS,
        "Invalid size of CMSISMat matricies in operator+");

    CMSISMat<A_ROWS, A_COLS> c;

    for (int i = 0; i < A_ROWS * A_COLS; i++)
    {
        c.data[i] = a.data[i] + b.data[i];
    }

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

    for (int i = 0; i < A_ROWS * A_COLS; i++)
    {
        c.data[i] = a.data[i] - b.data[i];
    }

    return c;
}

template <uint16_t ROWS, uint16_t COLS>
inline CMSISMat<ROWS, COLS> operator-(const CMSISMat<ROWS, COLS> &a)
{
    float scale(-1);
    CMSISMat<ROWS, COLS> c;

    for (int i = 0; i < ROWS * COLS; i++)
    {
        c.data[i] = scale * a.data[i];
    }

    return c;
}

template <uint16_t A_ROWS, uint16_t A_COLS, uint16_t B_ROWS, uint16_t B_COLS>
inline CMSISMat<A_ROWS, B_COLS> operator*(
    const CMSISMat<A_ROWS, A_COLS> &a,
    const CMSISMat<B_ROWS, B_COLS> &b)
{
    static_assert(A_COLS == B_ROWS, "Invalid size of CMSISMat matricies in operator*");

    CMSISMat<A_ROWS, B_COLS> c;  // Result matrix

    for (uint16_t i = 0; i < A_ROWS; i++)
    {
        for (uint16_t j = 0; j < B_COLS; j++)
        {
            c.data[i * B_COLS + j] = 0;

            for (uint16_t k = 0; k < A_COLS; k++)
            {
                c.data[i * B_COLS + j] += a.data[i * A_COLS + k] * b.data[k * B_COLS + j];
            }
        }
    }

    return c;
}

template <uint16_t ROWS, uint16_t COLS>
inline CMSISMat<ROWS, COLS> operator*(const CMSISMat<ROWS, COLS> &a, float b)
{
    CMSISMat<ROWS, COLS> c;

    for (int i = 0; i < ROWS * COLS; i++)
    {
        c.data[i] = b * a.data[i];
    }

    return c;
}

template <uint16_t ROWS, uint16_t COLS>
inline CMSISMat<ROWS, COLS> operator*(float b, const CMSISMat<ROWS, COLS> &a)
{
    CMSISMat<ROWS, COLS> c;

    for (int i = 0; i < ROWS * COLS; i++)
    {
        c.data[i] = b * a.data[i];
    }

    return c;
}

template <uint16_t ROWS, uint16_t COLS>
inline CMSISMat<ROWS, COLS> operator/(const CMSISMat<ROWS, COLS> &a, float b)
{
    b = 1 / b;
    CMSISMat<ROWS, COLS> c;

    for (int i = 0; i < ROWS * COLS; i++)
    {
        c.data[i] = b * a.data[i];
    }

    return c;
}
#endif

}  // namespace tap::algorithms

#endif  // TAPROOT_CMSIS_MAT_HPP_
