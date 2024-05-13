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
#include <roboost/utils/controllers.hpp>
#include <roboost/utils/filters.hpp>

namespace roboost
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
         * @return double The estimated output.
         */
        virtual double update(double input) = 0;

        /**
         * @brief Reset the estimator to its initial state.
         */
        virtual void reset() = 0;
    };

    /**
     * @brief Phase-Locked Loop (PLL) Estimator.
     */
    class PLL : public Estimator
    {
    public:
        PLL(double gain, double dampingFactor, double naturalFrequency)
            : gain_(gain), dampingFactor_(dampingFactor), naturalFrequency_(naturalFrequency), phaseError_(0.0), outputFrequency_(0.0), integrator_(0.0), differentiator_(0.0)
        {
        }

        double update(double inputPhase) override
        {
            // Phase detector
            phaseError_ = inputPhase - previousPhase_;
            previousPhase_ = inputPhase;

            // Loop filter (simple PI controller here)
            integrator_ += phaseError_;
            differentiator_ = phaseError_ - previousError_;
            previousError_ = phaseError_;

            double vcoInput = gain_ * (integrator_ + dampingFactor_ * differentiator_);

            // VCO simulation (simplified)
            outputFrequency_ = naturalFrequency_ + vcoInput;

            return outputFrequency_;
        }

        void reset() override
        {
            phaseError_ = 0.0;
            previousPhase_ = 0.0;
            previousError_ = 0.0;
            integrator_ = 0.0;
            differentiator_ = 0.0;
            outputFrequency_ = naturalFrequency_; // Assume it starts at the natural frequency
        }

    private:
        double gain_;
        double dampingFactor_;
        double naturalFrequency_;
        double phaseError_;
        double previousPhase_;
        double previousError_;
        double integrator_;
        double differentiator_;
        double outputFrequency_;
    };

    /**
     * @brief Tracking Loop Estimator utilizing PID control.
     */
    class TrackingLoop : public Estimator
    {
    public:
        TrackingLoop(double kp, double ki, double kd, double max_integral, roboost::filters::Filter& derivative_filter) : pid_(kp, ki, kd, max_integral, derivative_filter)
        {
            setpoint_ = 0.0; // Default setpoint could be adjusted if necessary
        }

        double update(double input) override
        {
            // The setpoint is often zero in tracking loops if the goal is error minimization
            double output = pid_.update(setpoint_, input);
            return output;
        }

        void reset() override { pid_.reset(); }

    private:
        roboost::controllers::PIDController pid_;
        double setpoint_; // This could be a parameter if dynamic adjustment is needed
    };

} // namespace roboost

#endif // ESTIMATORS_HPP
