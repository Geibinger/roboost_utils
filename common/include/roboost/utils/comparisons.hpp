/**
 * @file comparisons.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Utility functions for comparisons.
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef COMPARISONS_H
#define COMPARISONS_H

#include <cmath>

namespace roboost
{
    namespace comparisons
    {
        // Method to check if two floats are approximately equal
        template <typename T>
        bool approx_equal(T a, T b, T epsilon = std::numeric_limits<T>::epsilon())
        {
            return std::abs(a - b) < epsilon;
        }
    } // namespace comparisons
} // namespace roboost

#endif // COMPARISONS_H