/**
 * @file TimingUtils.hpp
 * @author Jakob Friedl
 * @brief Utilities for timing measurements and performance analysis.
 * @version 0.1
 * @date 2023-10-08
 * @copyright Copyright (c) 2023
 */

#ifndef TIMING_UTILS_HPP
#define TIMING_UTILS_HPP

#include <functional>
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <chrono>
// Define micros for non-Arduino platforms if not already defined
// inline unsigned long micros()
// {
//     static auto start = std::chrono::high_resolution_clock::now();
//     auto now = std::chrono::high_resolution_clock::now();
//     return std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
// }
#endif

namespace roboost
{
    namespace timing
    {

        /**
         * Measure the execution time of a function.
         * @tparam Func The function type.
         * @tparam Args The argument types.
         * @param func The function to be measured.
         * @param args The arguments to be passed to the function.
         * @return unsigned long The execution time in microseconds.
         */
        template <typename Func, typename... Args>
        unsigned long measureExecutionTime(Func func, Args&&... args)
        {
            unsigned long startTime = micros();
            func(std::forward<Args>(args)...);
            unsigned long endTime = micros();
            return endTime - startTime;
        }

    } // namespace timing
} // namespace roboost

#endif // TIMING_UTILS_HPP
