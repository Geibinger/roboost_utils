/**
 * @file logging.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Utility functions for logging.
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LOGGER_H
#define LOGGER_H

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <iostream>
#endif

#include <string>

namespace roboost
{
    namespace logging
    {

        // Abstract Logger interface
        class Logger
        {
        public:
            virtual void info(const std::string& message) = 0;
            virtual void warn(const std::string& message) = 0;
            virtual void error(const std::string& message) = 0;
            virtual void debug(const std::string& message) = 0;
            virtual ~Logger() {}
        };

#ifdef ARDUINO
        class SerialLogger : public Logger
        {
        private:
            usb_serial_class& serial;

            // Private constructor
            SerialLogger(usb_serial_class& serial) : serial(serial) {}

            // Prevent copying and assignment
            SerialLogger(const SerialLogger&) = delete;
            SerialLogger& operator=(const SerialLogger&) = delete;

        public:
            static SerialLogger& getInstance(usb_serial_class& serial)
            {
                static SerialLogger instance(serial);
                return instance;
            }

            void info(const std::string& message) override
            {
                serial.print("[INFO] ");
                serial.println(message);
            }

            void warn(const String& message) override
            {
                serial.print("[WARN] ");
                serial.println(message);
            }

            void error(const std::string& message) override
            {
                serial.print("[ERROR] ");
                serial.println(message);
            }

            void debug(const std::string& message) override
            {
                serial.print("[DEBUG] ");
                serial.println(message);
            }
        };
#else
        class ConsoleLogger : public Logger
        {
        private:
            // Private constructor
            ConsoleLogger() {}

            // Prevent copying and assignment
            ConsoleLogger(const ConsoleLogger&) = delete;
            ConsoleLogger& operator=(const ConsoleLogger&) = delete;

        public:
            static ConsoleLogger& getInstance()
            {
                static ConsoleLogger instance;
                return instance;
            }

            void info(const std::string& message) override { std::cout << "[INFO] " << message << std::endl; }

            void warn(const std::string& message) override { std::cout << "[WARN] " << message << std::endl; }

            void error(const std::string& message) override { std::cerr << "[ERROR] " << message << std::endl; }

            void debug(const std::string& message) override { std::cout << "[DEBUG] " << message << std::endl; }
        };
#endif // ARDUINO

    }; // namespace logging
};     // namespace roboost

#endif // LOGGER_H
