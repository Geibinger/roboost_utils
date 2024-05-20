/**
 * @file controllers.hpp
 * @brief Utility functions and classes for controllers.
 * @version 0.2
 * @date 2024-05-17
 *
 */

#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <functional>
#include <roboost/utils/callback_scheduler.hpp>
#include <roboost/utils/filters.hpp>

namespace roboost::controllers
{
    template <typename Derived, typename T>
    class Controller
    {
    public:
        T update(T setpoint, T input) { return static_cast<Derived*>(this)->update(setpoint, input); }

        void reset() { static_cast<Derived*>(this)->reset(); }

        T get_output() const { return static_cast<const Derived*>(this)->get_output(); }

    protected:
        T output_;
    };

    template <typename T, typename Filter>
    class PIDController : public Controller<PIDController<T, Filter>, T>
    {
    public:
        PIDController(T kp, T ki, T kd, T max_integral, Filter& derivative_filter)
            : kp_(kp), ki_(ki), kd_(kd), max_integral_(max_integral), integral_(T(0)), previous_error_(T(0)), derivative_filter_(derivative_filter)
        {
        }

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

            this->output_ = (kp_ * error + (ki_ * integral_ / update_frequency) + kd_ * derivative);
            return this->output_;
        }

        void reset()
        {
            integral_ = T(0);
            previous_error_ = T(0);
            derivative_filter_.reset();
        }

        T get_output() const { return this->output_; }

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
        Filter& derivative_filter_;
    };

    template <typename Filter>
    class FastPIDController : public Controller<FastPIDController<Filter>, int32_t>
    {
    public:
        FastPIDController(int32_t kp, int32_t ki, int32_t kd, int32_t max_integral, Filter& derivative_filter)
            : kp_(kp), ki_(ki), kd_(kd), max_integral_(max_integral), integral_(0), previous_error_(0), derivative_filter_(derivative_filter)
        {
        }

        int32_t update(int32_t setpoint, int32_t input)
        {
            int32_t error = setpoint - input;
            integral_ += error;
            integral_ = std::clamp(integral_, -max_integral_, max_integral_);

            uint32_t update_frequency = timing::CallbackScheduler::get_instance().get_update_frequency();
            int32_t derivative = 0;
            if (update_frequency > 0)
            {
                derivative = derivative_filter_.update((error - previous_error_) * update_frequency);
            }
            previous_error_ = error;

            // Calculate the PID output using integer arithmetic
            this->output_ = (kp_ * error + (ki_ * integral_ / update_frequency) + kd_ * derivative) / SCALE_FACTOR;
            return this->output_;
        }

        void reset()
        {
            integral_ = 0;
            previous_error_ = 0;
        }

        int32_t get_output() const { return this->output_; }
        int32_t get_kp() const { return kp_; }
        int32_t get_ki() const { return ki_; }
        int32_t get_kd() const { return kd_; }
        int32_t get_max_integral() const { return max_integral_; }
        int32_t get_integral() const { return integral_; }
        int32_t get_derivative() const { return derivative_filter_.get_output(); }
        int32_t get_previous_error() const { return previous_error_; }

        void set_kp(int32_t kp) { kp_ = kp; }
        void set_ki(int32_t ki) { ki_ = ki; }
        void set_kd(int32_t kd) { kd_ = kd; }
        void set_max_integral(int32_t max_integral) { max_integral_ = max_integral; }

    private:
        static constexpr int32_t SCALE_FACTOR = 1 << 10; // 2^10 = 1024

        int32_t kp_; // Proportional gain -> 2^10 times the typical value
        int32_t ki_;
        int32_t kd_;
        int32_t max_integral_;
        int32_t integral_;
        int32_t previous_error_;
        Filter& derivative_filter_;
    };

} // namespace roboost::controllers

#endif // CONTROLLERS_H
