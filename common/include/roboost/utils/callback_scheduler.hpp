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
#ifdef UNIT_TEST
    class TestCallbackScheduler
    {
    public:
        static TestCallbackScheduler& get_instance()
        {
            static TestCallbackScheduler instance;
            return instance;
        }

        uint32_t get_delta_time() const { return 10; }

        uint32_t get_update_frequency() const { return 1000000 / 10; }

        uint32_t get_last_update_time() const { return 0; }

        void update()
        {
            // No update logic needed for the test scheduler
        }

        void add_callback(std::function<void()> callback, uint32_t interval, uint32_t timeout, const char* name) { callbacks_.emplace_back(callback, interval, timeout, name); }

        void add_callback(IntervalCallback callback) { callbacks_.push_back(callback); }

    private:
        TestCallbackScheduler() = default;

        std::vector<IntervalCallback> callbacks_;
    };
    using CallbackScheduler = TestCallbackScheduler;
#else
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

        uint32_t get_delta_time() const { return delta_time_; }

        uint32_t get_update_frequency() const { return 1000000 / delta_time_; }

        uint32_t get_last_update_time() const { return last_update_time_; }

        void update()
        {
            uint32_t currentTime = micros();
            delta_time_ = (currentTime < last_update_time_) ? (ULONG_MAX - last_update_time_ + currentTime) : (currentTime - last_update_time_);
            last_update_time_ = currentTime;
            update_callbacks(currentTime);
        }

        void add_callback(std::function<void()> callback, uint32_t interval, uint32_t timeout, const char* name) { callbacks_.emplace_back(callback, interval, timeout, name); }

        void add_callback(IntervalCallback callback) { callbacks_.push_back(callback); }

    private:
        CallbackScheduler() : logger_(roboost::logging::Logger::get_instance()), last_update_time_(0), delta_time_(0) {}
        void update_callbacks(uint32_t current_time)
        {
            for (auto& callback : callbacks_)
            {
                callback.update(current_time);
            }
        }

        std::vector<IntervalCallback> callbacks_;
        roboost::logging::Logger& logger_;
        uint32_t last_update_time_;
        uint32_t delta_time_;
    };
#endif
} // namespace roboost::timing

#endif // CALLBACK_SCHEDULER_HPP
