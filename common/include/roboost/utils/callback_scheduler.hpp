/**
 * @file callback_scheduler.hpp
 * @author Jakob Friedl
 * @brief Scheduler for managing multiple timer callbacks.
 * @version 0.1
 * @date 2023-10-08
 * @copyright Copyright (c) 2023
 */

// TODO: Add documentation

#ifndef CALLBACK_SCHEDULER_HPP
#define CALLBACK_SCHEDULER_HPP

#include <roboost/utils/interval_callback.hpp>
#include <roboost/utils/logging.hpp>
#include <vector>

namespace roboost::timing
{
    class CallbackScheduler
    {
    public:
        CallbackScheduler(const CallbackScheduler&) = delete;
        CallbackScheduler& operator=(const CallbackScheduler&) = delete;

        static CallbackScheduler& get_instance()
        {
            static CallbackScheduler instance;
            return instance;
        }

        unsigned long get_delta_time() const { return deltaTime_; }

        unsigned long getLastUpdateTime() const { return lastUpdateTime_; }

        void update()
        {
            unsigned long currentTime = micros();
            deltaTime_ = (currentTime < lastUpdateTime_) ? (ULONG_MAX - lastUpdateTime_ + currentTime) : (currentTime - lastUpdateTime_);
            lastUpdateTime_ = currentTime;
            update_callbacks(currentTime);
        }

        void add_callback(std::function<void()> callback, unsigned long interval, unsigned long timeout, const char* name) { callbacks_.emplace_back(callback, interval, timeout, name); }

        void add_callback(IntervalCallback callback) { callbacks_.push_back(callback); }

    private:
        CallbackScheduler() : logger_(roboost::logging::Logger::get_instance()), lastUpdateTime_(0), deltaTime_(0) {}
        void update_callbacks(unsigned long currentTime)
        {
            for (auto& callback : callbacks_)
            {
                callback.update(currentTime);
            }
        }

        std::vector<IntervalCallback> callbacks_;
        roboost::logging::Logger& logger_;
        unsigned long lastUpdateTime_;
        unsigned long deltaTime_;
    };
} // namespace roboost::timing

#endif // CALLBACK_SCHEDULER_HPP
