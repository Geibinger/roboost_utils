/**
 * @file HardwareTimer.hpp
 * @author Jakob Friedl
 * @brief Abstraction for hardware timer functionality across different platforms.
 * @version 0.1
 * @date 2023-10-08
 * @copyright Copyright (c) 2023
 */

#ifndef HARDWARE_TIMER_HPP
#define HARDWARE_TIMER_HPP

#include <Arduino.h>
#include <roboost/utils/time_macros.hpp>

#ifdef ESP32
#include <esp32-hal-timer.h>
#endif

#ifdef TEENSYDUINO
#include <IntervalTimer.h>
#endif

namespace roboost
{
    namespace timing
    {

        class HardwareTimer
        {
#ifdef ESP32
            hw_timer_t* timer = nullptr;
            bool autoReload;
            uint8_t timerNum;
#elif defined(TEENSYDUINO)
            IntervalTimer timer;
#endif
        public:
            explicit HardwareTimer(uint8_t timerNum = 0)
#ifdef ESP32
                : timerNum(timerNum), autoReload(true)
#endif
            {
            }

            void setup(uint32_t period, void (*isr)(), int priority = 1)
            {
#ifdef ESP32
                timer = timerBegin(timerNum, 80, autoReload); // Prescaler of 80, auto-reload true
                int intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;  // Default to level 1, can be adjusted based on priority
                switch (priority)
                {
                    case 1:
                        intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
                        break;
                    case 2:
                        intr_alloc_flags = ESP_INTR_FLAG_LEVEL2;
                        break;
                    case 3:
                        intr_alloc_flags = ESP_INTR_FLAG_LEVEL3;
                        break;
                    case 4:
                        intr_alloc_flags = ESP_INTR_FLAG_LEVEL4;
                        break;
                    case 5:
                        intr_alloc_flags = ESP_INTR_FLAG_LEVEL5;
                        break;
                        // Define other levels as needed
                }
                timerAttachInterruptFlag(timer, isr, true, intr_alloc_flags); // Attach ISR with interrupt flags
                timerAlarmWrite(timer, period, autoReload);
                timerAlarmEnable(timer);
#elif defined(TEENSYDUINO)
                timer.begin(isr, period);
#else
                // TODO: Implement a standard timer setup if needed
#endif
            }

            ~HardwareTimer()
            {
#ifdef ESP32
                if (timer)
                {
                    timerEnd(timer);
                }
#endif
#ifdef TEENSYDUINO
                timer.end();
#endif
            }
        };

    } // namespace timing
} // namespace roboost

#endif // HARDWARE_TIMER_HPP
