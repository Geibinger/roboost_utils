/**
 * @file RTOSTaskManager.hpp
 * @author Jakob Friedl
 * @brief RTOS task management and watchdog timer functionality.
 * @version 0.1
 * @date 2023-10-08
 * @copyright Copyright (c) 2023
 */

#ifndef RTOS_TASK_MANAGER_HPP
#define RTOS_TASK_MANAGER_HPP

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <roboost/utils/logging.hpp>
#include <vector>

namespace roboost
{
    namespace timing
    {

        class RTOSTaskManager
        {
            std::vector<TaskHandle_t> tasks_;
            std::vector<TimerHandle_t> timers_;
            roboost::logging::Logger& logger_;

            // Private constructor for singleton
            explicit RTOSTaskManager(roboost::logging::Logger& logger) : logger_(logger) {}

        public:
            // Deleted copy constructor and assignment operator to prevent multiple instances
            RTOSTaskManager(const RTOSTaskManager&) = delete;
            RTOSTaskManager& operator=(const RTOSTaskManager&) = delete;

            static RTOSTaskManager& get_instance(roboost::logging::Logger& logger)
            {
                static RTOSTaskManager instance(logger);
                return instance;
            }

            void createTask(void (*taskFunction)(void*), const char* name, uint16_t stackSize = 2048, void* parameters = nullptr, UBaseType_t priority = 1)
            {
                TaskHandle_t taskHandle = nullptr;
                xTaskCreate(taskFunction, name, stackSize, parameters, priority, &taskHandle);
                tasks_.push_back(taskHandle);
            }

            void createTimer(const char* name, unsigned long period_ms, UBaseType_t autoReload, void* timerId, TimerCallbackFunction_t timerCallback)
            {
                TimerHandle_t timerHandle = xTimerCreate("Timer", pdMS_TO_TICKS(period_ms), autoReload, timerId, timerCallback);
                timers_.push_back(timerHandle);
                xTimerStart(timerHandle, 0);
            }

            void deleteTask(TaskHandle_t taskHandle) { vTaskDelete(taskHandle); }

            void deleteTimer(TimerHandle_t timerHandle) { xTimerDelete(timerHandle, portMAX_DELAY); }
        };

    } // namespace timing
} // namespace roboost

#endif // RTOS_TASK_MANAGER_HPP
