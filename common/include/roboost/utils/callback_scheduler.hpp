/**
 * @file callback_scheduler.hpp
 * @author Jakob Friedl
 * @brief Scheduler for managing multiple timer callbacks.
 * @version 0.1
 * @date 2023-10-08
 * @copyright Copyright (c) 2023
 */

#ifndef CALLBACK_SCHEDULER_HPP
#define CALLBACK_SCHEDULER_HPP

#include <roboost/utils/interval_callback.hpp>
#include <roboost/utils/logging.hpp>
#include <vector>

namespace roboost
{
    namespace timing
    {

        class CallbackScheduler
        {
            std::vector<IntervalCallback> callbacks_;
            roboost::logging::Logger& logger_;
            unsigned long lastUpdateTime_;
            unsigned long deltaTime_;

            // Private constructor for singleton
            explicit CallbackScheduler(roboost::logging::Logger& logger) : logger_(logger), lastUpdateTime_(0), deltaTime_(0) {}

        public:
            // Deleted copy constructor and assignment operator to prevent multiple instances
            CallbackScheduler(const CallbackScheduler&) = delete;
            CallbackScheduler& operator=(const CallbackScheduler&) = delete;

            static CallbackScheduler& get_instance(roboost::logging::Logger& logger)
            {
                static CallbackScheduler instance(logger);
                return instance;
            }

            unsigned long getDeltaTime() const { return deltaTime_; }

            unsigned long getLastUpdateTime() const { return lastUpdateTime_; }

            void update()
            {
                unsigned long currentTime = micros();
                deltaTime_ = (currentTime < lastUpdateTime_) ? (ULONG_MAX - lastUpdateTime_ + currentTime) : (currentTime - lastUpdateTime_);
                lastUpdateTime_ = currentTime;
                updateCallbacks(currentTime);
            }

            void addCallback(std::function<void()> callback, unsigned long interval, unsigned long timeout, const char* name) { callbacks_.emplace_back(callback, interval, timeout, name); }

            void addCallback(IntervalCallback callback) { callbacks_.push_back(callback); }

        private:
            void updateCallbacks(unsigned long currentTime)
            {
                for (auto& callback : callbacks_)
                {
                    callback.update(currentTime);
                }
            }
        };

    } // namespace timing
} // namespace roboost

#endif // CALLBACK_SCHEDULER_HPP
