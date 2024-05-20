/**
 * @file controllers.hpp
 * @brief Utility functions and classes for controllers.
 * @version 0.2
 * @date 2024-05-17
 */

#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <functional>
#include <roboost/utils/callback_scheduler.hpp>
#include <roboost/utils/filters.hpp>

namespace roboost::controllers
{
    /**
     * @brief PID controller for regulating processes.
     *
     * @tparam T The numeric type, defaulting to float.
     */
    template <typename T = float>
    class PIDController
    {
    public:
        PIDController(T kp, T ki, T kd, T max_integral, filters::FilterBase<T>& derivative_filter)
            : kp_(kp), ki_(ki), kd_(kd), max_integral_(max_integral), integral_(T(0)), previous_error_(T(0)), derivative_filter_(derivative_filter)
        {
        }

        /**
         * @brief Update the PID controller.
         *
         * @param setpoint The desired setpoint.
         * @param input The current input value.
         * @return T The control output.
         */
        T update(T setpoint, T input)
        {
            T error = setpoint - input;
            integral_ += error;
            integral_ = std::clamp(integral_, -max_integral_, max_integral_);

            T update_frequency = timing::CallbackScheduler::get_instance().get_update_frequency();
            T derivative = 0;
            if (update_frequency > 0)
            {
                derivative = derivative_filter_.update((error - previous_error_) * update_frequency);
            }
            previous_error_ = error;

            output_ = (kp_ * error + (ki_ * integral_ / update_frequency) + kd_ * derivative);
            return output_;
        }

        /**
         * @brief Reset the PID controller.
         */
        void reset()
        {
            integral_ = T(0);
            previous_error_ = T(0);
            derivative_filter_.reset();
        }

        /**
         * @brief Get the control output.
         *
         * @return T The control output.
         */
        T get_output() const { return output_; }

        // Getters and Setters
        T get_kp() const { return kp_; }
        T get_ki() const { return ki_; }
        T get_kd() const { return kd_; }
        T get_max_integral() const { return max_integral_; }
        T get_integral() const { return integral_; }
        T get_derivative() const { return derivative_filter_.get_output(); }
        T get_previous_error() const { return previous_error_; }

        void set_kp(T kp) { kp_ = kp; }
        void set_ki(T ki) { ki_ = ki; }
        void set_kd(T kd) { kd_ = kd; }
        void set_max_integral(T max_integral) { max_integral_ = max_integral; }

    private:
        T kp_;
        T ki_;
        T kd_;
        T max_integral_;
        T integral_;
        T previous_error_;
        T output_;
        filters::FilterBase<T>& derivative_filter_;
    };

} // namespace roboost::controllers

#endif // CONTROLLERS_H
