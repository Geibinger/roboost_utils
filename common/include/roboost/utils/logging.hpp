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
#ifdef TEENSYDUINO
            usb_serial_class* serial; // For Teensy
#else
            HardwareSerial* serial; // For ESP32 and other Arduino platforms
#endif

            SerialLogger() : serial(nullptr) {}

        public:
            static SerialLogger& getInstance()
            {
                static SerialLogger instance;
                return instance;
            }

#ifdef TEENSYDUINO
            void setSerial(usb_serial_class& serial) { this->serial = &serial; }
#else
            void setSerial(HardwareSerial& serial) { this->serial = &serial; }
#endif

            void info(const std::string& message) override
            {
                if (serial)
                {
                    serial->print("[INFO] ");
                    serial->println(message.c_str());
                }
            }

            void warn(const std::string& message) override
            {
                if (serial)
                {
                    serial->print("[WARN] ");
                    serial->println(message.c_str());
                }
            }

            void error(const std::string& message) override
            {
                if (serial)
                {
                    serial->print("[ERROR] ");
                    serial->println(message.c_str());
                }
            }

            void debug(const std::string& message) override
            {
                if (serial)
                {
                    serial->print("[DEBUG] ");
                    serial->println(message.c_str());
                }
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
