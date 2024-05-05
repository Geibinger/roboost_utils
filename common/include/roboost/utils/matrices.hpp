/**
 * @file matrices.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Utility functions for matrix operations.
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef MATRICES_H
#define MATRICES_H

#include <stdexcept>
#include <vector>

namespace roboost
{
    namespace math
    {

        typedef std::vector<double> Vector;
        typedef std::vector<Vector> Matrix;

        // Operator overloads
        Vector operator*(const Matrix& m, const Vector& v);
        Vector operator*(const Vector& v, double scalar);

        // Utility functions
        Vector Zero(size_t size);
        Matrix Zero(size_t size_x, size_t size_y);

    } // namespace math
} // namespace roboost

#endif // MATRICES_H
