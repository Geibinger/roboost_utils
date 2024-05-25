/**
 * @file time_macros.hpp
 * @author Jakob Friedl
 * @brief Macros for time conversion.
 * @version 0.1
 * @date 2023-10-08
 * @copyright Copyright (c) 2023
 */

#ifndef TIME_MACROS_HPP
#define TIME_MACROS_HPP

#define TIMING_MS_TO_US(milliseconds) ((milliseconds)*1000LL)
#define TIMING_US_TO_MS(microseconds) ((microseconds) / 1000LL)
#define TIMING_S_TO_US(seconds) ((seconds)*1000000LL)
#define TIMING_US_TO_S(microseconds) ((microseconds) / 1000000LL)
#define TIMING_MS_TO_S(milliseconds) ((milliseconds) / 1000.0)
#define TIMING_S_TO_MS(seconds) ((seconds)*1000.0)
#define TIMING_MS_TO_NS(milliseconds) ((milliseconds)*1000000LL)
#define TIMING_US_TO_S_DOUBLE(microseconds) ((double)(microseconds) / 1000000.0)

#endif // TIME_MACROS_HPP
