/**
 * @file filters.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Utility functions and classes for filtering.
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef FILTERS_H
#define FILTERS_H

#include <algorithm>
#include <deque>
#include <roboost/utils/constants.h>

namespace roboost
{
    namespace filters
    {
        /**
         * @brief Abstract base class for filters.
         *
         */
        class Filter
        {
        public:
            /**
             * @brief Update the filter.
             *
             * @param input The input value.
             * @return double The filtered value.
             */
            virtual double update(double input) = 0;

            /**
             * @brief Reset the filter.
             *
             */
            virtual void reset() = 0;

            double get_output() const { return output_; }

        protected:
            double output_;
        };

        class NoFilter : public Filter
        {
        public:
            double update(double input)
            {
                output_ = input;
                return output_;
            }

            void reset() {}
        };

        /**
         * @brief Implementation and definiton of low pass filter class.
         *
         */

        class LowPassFilter : public Filter
        {
        public:
            /**
             * @brief Construct a new Low Pass Filter object
             *
             * @param cutoff_frequency The cutoff frequency of the filter.
             * @param sampling_time The sampling time of the filter.
             */
            LowPassFilter(double cutoff_frequency, double sampling_time);

            /**
             * @brief Update the filter.
             *
             * @param input The input value.
             * @return double The filtered value.
             */
            double update(double input);

            /**
             * @brief Reset the filter.
             *
             */
            void reset();

            /**
             * @brief Get the cutoff frequency.
             *
             * @return double The cutoff frequency.
             */
            double get_cutoff_frequency() const;

            /**
             * @brief Get the sampling time.
             *
             * @return double The sampling time.
             */
            double get_sampling_time() const;

            /**
             * @brief Set the cutoff frequency.
             *
             * @param cutoff_frequency The cutoff frequency.
             */
            void set_cutoff_frequency(double cutoff_frequency);

            /**
             * @brief Set the sampling time.
             *
             * @param sampling_time The sampling time.
             */
            void set_sampling_time(double sampling_time);

        private:
            double cutoff_frequency_;
            double sampling_time_;
            double alpha_;
        };

        /**
         * @brief Implementation and definiton of moving average filter class.
         *
         */
        class MovingAverageFilter : public Filter
        {
        public:
            /**
             * @brief Construct a new Moving Average Filter object
             *
             * @param window_size The size of the filter window.
             */
            MovingAverageFilter(int window_size);

            /**
             * @brief Update the filter.
             *
             * @param input The input value.
             * @return double The filtered value.
             */
            double update(double input);

            /**
             * @brief Reset the filter.
             *
             */
            void reset();

        private:
            std::deque<double> input_history_;
            int window_size_;
        };

        /**
         * @brief Implementation and definiton of median filter class.
         *
         */
        class MedianFilter : public Filter
        {
        public:
            /**
             * @brief Construct a new Median Filter object
             *
             * @param window_size The size of the filter window.
             */
            MedianFilter(int window_size);

            /**
             * @brief Update the filter.
             *
             * @param input The input value.
             * @return double The filtered value.
             */
            double update(double input);

            /**
             * @brief Reset the filter.
             *
             */
            void reset();

        private:
            std::deque<double> input_history_;
            int window_size_;
        };

        /**
         * @brief Implementation and definiton of exponential moving average filter
         * class.
         *
         */
        class ExponentialMovingAverageFilter : public Filter
        {
        public:
            /**
             * @brief Construct a new Exponential Moving Average Filter object
             *
             * @param alpha The alpha value of the filter.
             */
            ExponentialMovingAverageFilter(double alpha);

            /**
             * @brief Update the filter.
             *
             * @param input The input value.
             * @return double The filtered value.
             */
            double update(double input);

            /**
             * @brief Reset the filter.
             *
             */
            void reset();

        private:
            double alpha_;
        };

        class RateLimitingFilter
        {
        public:
            RateLimitingFilter(double max_rate_per_second, double update_rate) : max_increment_(max_rate_per_second / update_rate), current_setpoint_(0.0) {}

            double update(double target, double current_position)
            {
                double increment = target - current_setpoint_;
                // Limit the increment to the maximum allowed step
                increment = std::max(std::min(increment, max_increment_), -max_increment_);
                current_setpoint_ += increment;

                // Optional: add smoothing or further processing here
                // TODO: add smoothing
                return current_setpoint_;
            }

            void reset() { current_setpoint_ = 0.0; }

        private:
            double max_increment_;
            double current_setpoint_;
        };

    } // namespace filters
} // namespace roboost

#endif // FILTERS_H