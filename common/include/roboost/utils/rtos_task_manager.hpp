/**
 * @file TaskManager.hpp
 * @author Jakob Friedl
 * @brief Task management functionality using ChibiOS/RT on Teensy and FreeRTOS on ESP32.
 * @version 0.1
 * @date 2023-10-08
 */

#ifndef TASK_MANAGER_HPP
#define TASK_MANAGER_HPP

#include <roboost/utils/logging.hpp>
#include <vector>

#ifdef ESP32
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#elif defined(TEENSYDUINO)
#include "ChRt.h"
#endif

namespace roboost
{
    namespace timing
    {

        class TaskManager
        {
            std::vector<void*> tasks_;
            roboost::logging::Logger& logger_;

            TaskManager() : logger_(roboost::logging::Logger::get_instance()) {}

        public:
            TaskManager(const TaskManager&) = delete;
            TaskManager& operator=(const TaskManager&) = delete;

            static TaskManager& get_instance()
            {
                static TaskManager instance;
                return instance;
            }
#ifdef TEENSYDUINO
            void create_task(void(taskFunction)(void), const char* name, int stackSize){
#elif ESP32
            void create_task(TaskFunction_t taskFunction, const char* name, uint16_t stackSize = 2048, void* parameters = nullptr, UBaseType_t priority = 1)
            {
                TaskHandle_t taskHandle = nullptr;
                xTaskCreate(taskFunction, name, stackSize, parameters, priority, &taskHandle);
                tasks_.push_back(taskHandle);
                logger_.debug("Task created: " + std::string(name));
            }
#endif

                void delete_task(void* taskHandle){
#ifdef TEENSYDUINO
                    chThdTerminate((thread_t*)taskHandle); // Correctly terminate the thread
            chThdWait((thread_t*)taskHandle);              // Wait for the thread to finish
            auto it = std::find(tasks_.begin(), tasks_.end(), taskHandle);
            if (it != tasks_.end())
                tasks_.erase(it);
#elif ESP32
                vTaskDelete((TaskHandle_t)taskHandle);
#endif
            logger_.debug("Task deleted");
        }
    }; // namespace timing

} // namespace roboost
} // namespace roboost

#endif // TASK_MANAGER_HPP
