/**
 * @file config.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Utility functions for configurations.
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef CONFIG_H
#define CONFIG_H
#include <chrono>
#include <string>

namespace roboost::description
{
    using namespace std::chrono_literals;

    // Config base struct
    struct Config
    {
        // Config constructor
        Config() = default;
        // Config destructor
        virtual ~Config() = default;

        // Serialize config
        virtual std::string serialize() const = 0;
    };

    // State base struct
    struct State
    {
        // State constructor
        State() = default;
        // State destructor
        virtual ~State() = default;

        std::chrono::microseconds timestamp = 0us;

        // Serialize state
        virtual std::string serialize() const = 0;
    };

} // namespace roboost::description

#endif // CONFIG_H