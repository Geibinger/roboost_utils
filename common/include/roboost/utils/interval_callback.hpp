/**
 * @file interval_callback.hpp
 * @author Jakob Friedl
 * @brief Callback management for timed execution.
 * @version 0.1
 * @date 2023-10-08
 * @copyright Copyright (c) 2023
 */

#ifndef INTERVAL_CALLBACK_HPP
#define INTERVAL_CALLBACK_HPP

#include <functional>
#include <limits.h>
#include <roboost/utils/timing_utils.hpp>

namespace roboost
{
    namespace timing
    {

        class IntervalCallback
        {
        public:
            IntervalCallback(std::function<void()> callback, unsigned long interval, unsigned long timeout, const char* name)
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

                    if (endTime - startTime > timeout_)
                    {
                        missed_deadlines_ = (missed_deadlines_ == UINT32_MAX) ? 0 : missed_deadlines_ + 1;
                        // Log missed deadline
                    }

                    lastScheduledRun_ = (endTime - lastScheduledRun_ > interval_) ? endTime : lastScheduledRun_ + interval_;
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

    } // namespace timing
} // namespace roboost

#endif // INTERVAL_CALLBACK_HPP
