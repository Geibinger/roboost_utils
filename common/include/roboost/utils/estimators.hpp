/**
 * @file estimators.hpp
 * @author Jakob Friedl
 * @brief Estimator classes for various control and estimation tasks.
 * @version 0.1
 * @date 2023-10-08
 * @copyright Copyright (c) 2023
 */

#ifndef ESTIMATORS_HPP
#define ESTIMATORS_HPP

#include <cmath>
#include <roboost/utils/callback_scheduler.hpp>
#include <roboost/utils/controllers.hpp>
#include <roboost/utils/filters.hpp>

namespace roboost::estimators
{

    /**
     * @brief Abstract base class for all estimators.
     */
    class Estimator
    {
    public:
        virtual ~Estimator() = default;

        /**
         * @brief Update the estimator with a new input value.
         * @param input The input measurement.
         * @return float The estimated output.
         */
        virtual float update(float input) = 0;

        /**
         * @brief Reset the estimator to its initial state.
         */
        virtual void reset() = 0;

        /**
         * @brief Get the output value.
         * @return float The output value.
         */
        float get_output() const { return output_; }

    protected:
        float output_;
    };

    class NoEstimator : public Estimator
    {
    public:
        NoEstimator() { output_ = 0.0; }

        float update(float input) override
        {
            output_ = input;
            return output_;
        }

        void reset() override { output_ = 0.0; }
    };

    /**
     * @brief Phase-Locked Loop (PLL) Estimator.
     */
    class PLL : public Estimator
    {
    public:
        PLL(float gain, float dampingFactor, float naturalFrequency)
            : gain_(gain), dampingFactor_(dampingFactor), naturalFrequency_(naturalFrequency), phaseError_(0.0), integrator_(0.0), differentiator_(0.0)
        {
            output_ = naturalFrequency_;
        }

        float update(float inputPhase) override
        {
            // Phase detector
            phaseError_ = inputPhase - previousPhase_;
            previousPhase_ = inputPhase;

            // Loop filter (simple PI controller here)
            integrator_ += phaseError_;
            differentiator_ = phaseError_ - previousError_;
            previousError_ = phaseError_;

            float vcoInput = gain_ * (integrator_ + dampingFactor_ * differentiator_);

            // VCO simulation (simplified)
            output_ = naturalFrequency_ + vcoInput;

            return output_;
        }

        void reset() override
        {
            phaseError_ = 0.0;
            previousPhase_ = 0.0;
            previousError_ = 0.0;
            integrator_ = 0.0;
            differentiator_ = 0.0;
        }

    private:
        float gain_;
        float dampingFactor_;
        float naturalFrequency_;
        float phaseError_;
        float previousPhase_;
        float previousError_;
        float integrator_;
        float differentiator_;
        float outputFrequency_;
    };

    /**
     * @brief Tracking Loop Estimator utilizing PID control.
     */
    class TrackingLoop : public Estimator
    {
    public:
        TrackingLoop(float kp, float ki, float kd, float max_integral, roboost::filters::FilterBase<float>& derivative_filter) : pid_(kp, ki, kd, max_integral, derivative_filter), setpoint_(0.0)
        {
            output_ = 0.0;
        }

        float update(float input) override
        {
            // The setpoint is often zero in tracking loops if the goal is error minimization
            output_ = pid_.update(setpoint_, input);
            return output_;
        }

        void reset() override { pid_.reset(); }

    private:
        roboost::controllers::PIDController<float> pid_;
        float setpoint_; // This could be a parameter if dynamic adjustment is needed
    };

    class IncrementalEncoderVelocityEstimator : public Estimator
    {
    public:
        IncrementalEncoderVelocityEstimator(float sampling_period, float relative_accuracy, uint16_t max_backward_steps)
            : sampling_period_(sampling_period), relative_accuracy_(relative_accuracy), max_backward_steps_(max_backward_steps), delta_time_(0.0), delta_position_(0.0)
        {
            // Initialization code
        }

        float update(float current_position_radians) override
        {
            // Get current time in microseconds
            uint32_t current_time = timing::CallbackScheduler::get_instance().get_last_update_time();

            // Store current position and time
            position_history_.emplace_back(current_position_radians, current_time);

            // Remove old entries if history exceeds maximum size
            if (position_history_.size() > max_backward_steps_)
            {
                position_history_.erase(position_history_.begin());
            }

            // Ensure there are enough samples to calculate velocity
            if (position_history_.size() < 2)
            {
                return 0.0;
            }

            // Calculate the number of steps to look back for the required accuracy
            uint16_t j = determine_backward_steps();

            // Calculate velocity
            auto [previous_position, previous_time] = position_history_[position_history_.size() - 1 - j];
            delta_position_ = current_position_radians - previous_position;
            delta_time_ = (current_time - previous_time) / 1000000.0; // Convert to seconds

            // Return estimated velocity (radians per second)
            output_ = delta_position_ / delta_time_;
            return output_;
        }

        void reset() override
        {
            position_history_.clear();
            output_ = 0.0;
        }

        float get_delta_time() const { return delta_time_; }
        float get_delta_position() const { return delta_position_; }

    private:
        float delta_time_;
        float delta_position_;
        float sampling_period_;
        float relative_accuracy_;
        uint16_t max_backward_steps_;
        std::vector<std::pair<float, uint32_t>> position_history_; // Stores pairs of position and time

        uint16_t determine_backward_steps()
        {
            // Search for the number of steps to achieve the desired relative accuracy
            uint16_t j = 1;
            float min_relative_error = std::numeric_limits<float>::max();
            uint16_t best_j = 1;

            for (uint16_t i = 1; i <= max_backward_steps_ && i < position_history_.size(); ++i)
            {
                auto [prev_position, prev_time] = position_history_[position_history_.size() - 1 - i];
                float delta_position = position_history_.back().first - prev_position;
                float delta_time = (position_history_.back().second - prev_time) / 1000000.0;

                float velocity = delta_position / delta_time;
                float relative_error = std::abs(velocity * sampling_period_ / delta_position);

                if (relative_error < min_relative_error)
                {
                    min_relative_error = relative_error;
                    best_j = i;
                }

                if (relative_error <= relative_accuracy_)
                {
                    break;
                }
            }

            return best_j;
        }
    };

} // namespace roboost::estimators

#endif // ESTIMATORS_HPP
