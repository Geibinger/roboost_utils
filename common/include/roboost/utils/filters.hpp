/**
 * @file filters.hpp
 * @brief Utility functions and classes for filtering.
 * @version 0.2
 * @date 2024-05-17
 */

#ifndef FILTERS_H
#define FILTERS_H

#include <algorithm>
#include <deque>
#include <memory>
#include <numeric>
#include <vector>

namespace roboost::filters
{
    /**
     * @brief Abstract base class for filters.
     *
     * @tparam T The numeric type, defaulting to float.
     */
    template <typename T = float>
    class FilterBase
    {
    public:
        virtual ~FilterBase() = default;

        /**
         * @brief Update the filter.
         *
         * @param input The input value.
         * @return T The filtered value.
         */
        virtual T update(T input) = 0;

        /**
         * @brief Update the filter with target and current position.
         *
         * @param target The target value.
         * @param current_position The current position.
         * @return T The filtered value.
         */
        virtual T update(T target, T current_position) { return update(target); }

        /**
         * @brief Reset the filter.
         */
        virtual void reset() = 0;

        /**
         * @brief Get the output value.
         *
         * @return T The output value.
         */
        virtual T get_output() const = 0;
    };

    /**
     * @brief No filtering, just pass the input to the output.
     *
     * @tparam T The numeric type, defaulting to float.
     *
     * Use this when you do not want to apply any filtering.
     */
    template <typename T = float>
    class NoFilter : public FilterBase<T>
    {
    public:
        T update(T input) override
        {
            output_ = input;
            return output_;
        }

        void reset() override {}

        T get_output() const override { return output_; }

    private:
        T output_;
    };

    /**
     * @brief Chain multiple filters together.
     *
     * @tparam T The numeric type, defaulting to float.
     *
     * Use this when you want to apply multiple filters sequentially.
     */
    template <typename T = float>
    class ChainedFilter : public FilterBase<T>
    {
    public:
        void addFilter(std::unique_ptr<FilterBase<T>> filter) { filters_.push_back(std::move(filter)); }

        T update(T input) override
        {
            T result = input;
            for (auto& filter : filters_)
            {
                result = filter->update(result);
            }
            output_ = result;
            return output_;
        }

        void reset() override
        {
            for (auto& filter : filters_)
            {
                filter->reset();
            }
        }

        T get_output() const override { return output_; }

    private:
        std::vector<std::unique_ptr<FilterBase<T>>> filters_;
        T output_;
    };

    /**
     * @brief Low-pass filter to smooth out high-frequency noise.
     *
     * @tparam T The numeric type, defaulting to float.
     *
     * Use this to filter out high-frequency noise from a signal.
     */
    template <typename T = float>
    class LowPassFilter : public FilterBase<T>
    {
    public:
        LowPassFilter(T cutoff_frequency, T sampling_time) : cutoff_frequency_(cutoff_frequency), sampling_time_(sampling_time) { computeAlpha(); }

        T update(T input) override
        {
            output_ += alpha_ * (input - output_);
            return output_;
        }

        void reset() override { output_ = T(0); }

        T get_output() const override { return output_; }

        void set_cutoff_frequency(T cutoff_frequency)
        {
            cutoff_frequency_ = cutoff_frequency;
            computeAlpha();
        }

        void set_sampling_time(T sampling_time)
        {
            sampling_time_ = sampling_time;
            computeAlpha();
        }

    private:
        void computeAlpha() { alpha_ = sampling_time_ / (sampling_time_ + 1.0 / (2.0 * 3.141592653589793 * cutoff_frequency_)); }

        T cutoff_frequency_;
        T sampling_time_;
        T alpha_;
        T output_ = T(0);
    };

    /**
     * @brief Moving average filter to smooth out noise.
     *
     * @tparam T The numeric type, defaulting to float.
     *
     * Use this to reduce noise in a signal by averaging over a window of values.
     */
    template <typename T = float>
    class MovingAverageFilter : public FilterBase<T>
    {
    public:
        MovingAverageFilter(int window_size) : window_size_(window_size) {}

        T update(T input) override
        {
            input_history_.push_back(input);
            if (input_history_.size() > window_size_)
            {
                input_history_.pop_front();
            }
            T sum = std::accumulate(input_history_.begin(), input_history_.end(), T(0));
            output_ = sum / input_history_.size();
            return output_;
        }

        void reset() override { input_history_.clear(); }

        T get_output() const override { return output_; }

    private:
        std::deque<T> input_history_;
        int window_size_;
        T output_ = T(0);
    };

    /**
     * @brief Median filter to reduce noise by taking the median of a window of values.
     *
     * @tparam T The numeric type, defaulting to float.
     *
     * Use this to reduce noise in a signal, especially useful when the noise includes outliers.
     */
    template <typename T = float>
    class MedianFilter : public FilterBase<T>
    {
    public:
        MedianFilter(int window_size) : window_size_(window_size) {}

        T update(T input) override
        {
            input_history_.push_back(input);
            if (input_history_.size() > window_size_)
            {
                input_history_.pop_front();
            }
            std::vector<T> sorted_history(input_history_.begin(), input_history_.end());
            std::sort(sorted_history.begin(), sorted_history.end());
            output_ = sorted_history[sorted_history.size() / 2];
            return output_;
        }

        void reset() override { input_history_.clear(); }

        T get_output() const override { return output_; }

    private:
        std::deque<T> input_history_;
        int window_size_;
        T output_ = T(0);
    };

    /**
     * @brief Exponential moving average filter for smoothing data.
     *
     * @tparam T The numeric type, defaulting to float.
     *
     * Use this to apply an exponential moving average to a signal, useful for giving more weight to recent values.
     */
    template <typename T = float>
    class ExponentialMovingAverageFilter : public FilterBase<T>
    {
    public:
        ExponentialMovingAverageFilter(T alpha) : alpha_(alpha) {}

        T update(T input) override
        {
            output_ = alpha_ * input + (1 - alpha_) * output_;
            return output_;
        }

        void reset() override { output_ = T(0); }

        T get_output() const override { return output_; }

    private:
        T alpha_;
        T output_ = T(0);
    };

    /**
     * @brief Rate limiting filter to limit the rate of change of a signal.
     *
     * @tparam T The numeric type, defaulting to float.
     *
     * Use this to ensure that the rate of change of a signal does not exceed a specified limit.
     */
    template <typename T = float>
    class RateLimitingFilter : public FilterBase<T>
    {
    public:
        RateLimitingFilter(T max_rate_per_second, T update_rate) : max_increment_(max_rate_per_second / update_rate) { output_ = T(0); }

        T update(T input) override
        {
            T increment = input - output_;
            increment = std::max(std::min(increment, max_increment_), -max_increment_);
            output_ += increment;
            return output_;
        }

        T update(T target, T current_position) override
        {
            T increment = target - output_;
            increment = std::max(std::min(increment, max_increment_), -max_increment_);
            output_ += increment;
            return output_;
        }

        void reset() override { output_ = T(0); }

        T get_output() const override { return output_; }

    private:
        T max_increment_;
        T output_ = T(0);
    };

    /**
     * @brief Infinite impulse response (IIR) filter for digital signal processing.
     *
     * @tparam T The numeric type, defaulting to float.
     *
     * Use this for filtering signals where specific frequency response characteristics are required.
     */
    template <typename T = float>
    class IIRFilter : public FilterBase<T>
    {
    public:
        IIRFilter(T a0, T a1, T a2, T b1, T b2) : a0_(a0), a1_(a1), a2_(a2), b1_(b1), b2_(b2), x1_(T(0)), x2_(T(0)), y1_(T(0)), y2_(T(0)) {}

        T update(T input)
        {
            T output = a0_ * input + a1_ * x1_ + a2_ * x2_ - b1_ * y1_ - b2_ * y2_;
            x2_ = x1_;
            x1_ = input;
            y2_ = y1_;
            y1_ = output;
            this->output_ = output;
            return this->output_;
        }

        void reset() { x1_ = x2_ = y1_ = y2_ = T(0); }

        T get_output() const { return this->output_; }

    private:
        T a0_, a1_, a2_, b1_, b2_;
        T x1_, x2_;
        T y1_, y2_;
    };

} // namespace roboost::filters

#endif // FILTERS_H
