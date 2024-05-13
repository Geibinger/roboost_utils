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
#include <memory>

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
         * @brief Chained Filter class that allows chaining multiple filters together.
         */
        class ChainedFilter : public Filter
        {
        public:
            ChainedFilter() = default;

            /**
             * @brief Add a filter to the chain.
             * @param filter A unique pointer to a Filter object.
             */
            void addFilter(std::unique_ptr<Filter> filter) { filters_.push_back(std::move(filter)); }

            /**
             * @brief Update the filter chain with the input value passed through each filter.
             * @param input The input value to the first filter in the chain.
             * @return double The output from the last filter in the chain.
             */
            double update(double input) override
            {
                double result = input;
                for (auto& filter : filters_)
                {
                    result = filter->update(result);
                }
                output_ = result;
                return output_;
            }

            /**
             * @brief Reset all filters in the chain.
             */
            void reset() override
            {
                for (auto& filter : filters_)
                {
                    filter->reset();
                }
            }

        private:
            std::vector<std::unique_ptr<Filter>> filters_; // Holds all the filters in the chain
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

        class RateLimitingFilter : public Filter
        {
        public:
            RateLimitingFilter(double max_rate_per_second, double update_rate) : max_increment_(max_rate_per_second / update_rate) { output_ = 0.0; }

            double update(double target, double current_position)
            {
                double increment = target - output_;
                // Limit the increment to the maximum allowed step
                increment = std::max(std::min(increment, max_increment_), -max_increment_);
                output_ += increment;

                return output_;
            }

            void reset() { output_ = 0.0; }

        private:
            double max_increment_;
        };

        /**
         * @brief Implementation and definiton of IIR filter class.
         *
         */
        class IIRFilter : public Filter
        {
        public:
            /**
             * @brief Construct a new IIR Filter object
             * @param a0 The a0 coefficient of the IIR filter.
             * @param a1 The a1 coefficient of the IIR filter.
             * @param a2 The a2 coefficient of the IIR filter.
             * @param b1 The b1 coefficient of the IIR filter.
             * @param b2 The b2 coefficient of the IIR filter.
             */
            IIRFilter(double a0, double a1, double a2, double b1, double b2) : a0_(a0), a1_(a1), a2_(a2), b1_(b1), b2_(b2), x1_(0.0), x2_(0.0), y1_(0.0), y2_(0.0) {}

            /**
             * @brief Update the filter with a new input value.
             * @param input The input value.
             * @return double The filtered value.
             */
            double update(double input) override
            {
                double output = a0_ * input + a1_ * x1_ + a2_ * x2_ - b1_ * y1_ - b2_ * y2_;
                x2_ = x1_; // Update the delayed x inputs
                x1_ = input;
                y2_ = y1_; // Update the delayed y outputs
                y1_ = output;
                output_ = output; // Store the output
                return output;
            }

            /**
             * @brief Reset the filter's history.
             */
            void reset() override { x1_ = x2_ = y1_ = y2_ = 0.0; }

        private:
            double a0_, a1_, a2_, b1_, b2_; // Filter coefficients
            double x1_, x2_;                // Delayed input samples
            double y1_, y2_;                // Delayed output samples
        };

    } // namespace filters
} // namespace roboost

#endif // FILTERS_H