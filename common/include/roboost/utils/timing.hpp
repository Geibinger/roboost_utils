/**
 * @file timing.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Time manager for utilizing hardware timers and centralized delta time calculation.
 * @version 0.1
 * @date 2023-10-08
 * @copyright Copyright (c) 2023
 *
 * Correction focuses on ensuring that after a missed deadline the next task execution time is set properly.
 */
#ifndef TIMING_HPP
#define TIMING_HPP

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <chrono>

inline unsigned long micros()
{
    static auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
}
#endif

#define TIMING_MS_TO_US(milliseconds) ((milliseconds)*1000LL)
#define TIMING_US_TO_MS(microseconds) ((microseconds) / 1000LL)
#define TIMING_S_TO_US(seconds) ((seconds)*1000000LL)
#define TIMING_US_TO_S(microseconds) ((microseconds) / 1000000LL)
#define TIMING_MS_TO_S(milliseconds) ((milliseconds) / 1000.0)
#define TIMING_S_TO_MS(seconds) ((seconds)*1000.0)
#define TIMING_MS_TO_NS(milliseconds) ((milliseconds)*1000000LL)

#define MICROS_TO_SECONDS_DOUBLE(microseconds) ((double)(microseconds) / 1000000.0)

#include <functional>
#include <limits.h>
#include <roboost/utils/logging.hpp>
#include <vector>

namespace roboost
{
    namespace timing
    {
        class Task
        {
            friend class TimingService;

        public:
            Task(std::function<void()> callback, unsigned long interval, unsigned long timeout, const char* name)
                : callback_(callback), interval_(interval), timeout_(timeout), lastScheduledRun_(micros()), name_(name), missed_deadlines_(0)
            {
            }

            void update(unsigned long currentTime)
            {
                if (currentTime - lastScheduledRun_ >= interval_)
                {
                    unsigned long startTime = micros();
                    callback_();
                    unsigned long endTime = micros();

                    if (endTime - lastScheduledRun_ > interval_)
                    {
                        lastScheduledRun_ = endTime;
                    }
                    else
                    {
                        lastScheduledRun_ += interval_;
                    }

                    if (endTime - startTime > timeout_)
                    {
                        missed_deadlines_ = (missed_deadlines_ == UINT32_MAX) ? 0 : missed_deadlines_ + 1;
                    }
                }
            }

            uint32_t getMissedDeadlines() const { return missed_deadlines_; }

        private:
            std::function<void()> callback_;
            unsigned long interval_;
            unsigned long timeout_;
            unsigned long lastScheduledRun_;
            const char* name_;
            uint32_t missed_deadlines_;
        };

#ifdef UNIT_TEST
        // MockTimingService for unit testing
        class TimingService
        {
        public:
            static TimingService& get_instance(roboost::logging::Logger& logger)
            {
                static TimingService instance(logger);
                return instance;
            }

            void reset() { lastUpdateTime_ = micros(); }

            void update()
            {
                lastUpdateTime_ += deltaTime_;
                updateTasks(lastUpdateTime_);
            }

            void setDeltaTime(unsigned long deltaTime) { deltaTime_ = deltaTime; }

            unsigned long getDeltaTime() const { return deltaTime_; }

        private:
            TimingService(roboost::logging::Logger& logger) : lastUpdateTime_(0), deltaTime_(TIMING_MS_TO_US(10)), logger_(logger) {}
            unsigned long lastUpdateTime_;
            unsigned long deltaTime_;
            std::vector<Task> tasks_;
            roboost::logging::Logger& logger_;

            void updateTasks(unsigned long currentTime)
            {
                for (auto& task : tasks_)
                {
                    task.update(currentTime);
                }
            }
        };
#else
        class TimingService
        {
        private:
            unsigned long lastUpdateTime_;
            unsigned long deltaTime_;
            std::vector<Task> tasks_;
            roboost::logging::Logger* logger_; // Use pointer for logger to ensure flexibility

            TimingService() : lastUpdateTime_(0), deltaTime_(0), logger_(nullptr) {}

        public:
            static TimingService& get_instance()
            {
                static TimingService instance;
                return instance;
            }

            void setLogger(roboost::logging::Logger& logger) { this->logger_ = &logger; }

            void update()
            {
                unsigned long currentTime = micros();
                deltaTime_ = (currentTime < lastUpdateTime_) ? (ULONG_MAX - lastUpdateTime_ + currentTime) : (currentTime - lastUpdateTime_);
                lastUpdateTime_ = currentTime;
                updateTasks(currentTime);
            }

            unsigned long getDeltaTime() const { return deltaTime_; }

        private:
            void updateTasks(unsigned long currentTime)
            {
                for (auto& task : tasks_)
                {
                    task.update(currentTime);
                }
            }
        };
#endif

    } // namespace timing
} // namespace roboost

#endif // TIMING_HPP
