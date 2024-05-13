/**
 * @file matrices.hpp
 * @author Jakob Friedl (friedl.j@gmail.com)
 * @brief Templated utility functions and classes for matrix operations.
 * @version 0.2
 * @date 2023-10-08
 *
 * Copyright (c) 2023 Jakob Friedl
 *
 */

#ifndef MATRICES_H
#define MATRICES_H

#include <vector>
// TODO: Use pre allocated memory for vectors and matrices

namespace roboost
{
    namespace math
    {
        // Template definitions for Vector and Matrix to handle any numeric type T
        template <typename T>
        using Vector = std::vector<T>;

        template <typename T>
        using Matrix = std::vector<Vector<T>>;

        // Operator overloads for matrix-vector multiplication
        template <typename T>
        Vector<T> operator*(const Matrix<T>& m, const Vector<T>& v)
        {
            if (m.empty() || m[0].size() != v.size())
            {
                // throw std::invalid_argument("Matrix and vector dimensions must match.");
                // Consider logging or more specific error handling here
                // TODO: Add logging
            }

            Vector<T> result(m.size(), 0);
            for (std::size_t i = 0; i < m.size(); ++i)
            {
                for (std::size_t j = 0; j < v.size(); ++j)
                {
                    result[i] += m[i][j] * v[j];
                }
            }
            return result;
        }

        // Operator overload for vector-scalar multiplication
        template <typename T>
        Vector<T> operator*(const Vector<T>& v, T scalar)
        {
            Vector<T> result(v.size());
            for (std::size_t i = 0; i < v.size(); ++i)
            {
                result[i] = v[i] * scalar;
            }
            return result;
        }

        // Utility functions for creating zero-initialized vectors and matrices
        template <typename T>
        Vector<T> ZeroVector(std::size_t size)
        {
            return Vector<T>(size, T(0));
        }

        template <typename T>
        Matrix<T> ZeroMatrix(std::size_t size_x, std::size_t size_y)
        {
            return Matrix<T>(size_x, Vector<T>(size_y, T(0)));
        }

    } // namespace math
} // namespace roboost

#endif // MATRICES_H
