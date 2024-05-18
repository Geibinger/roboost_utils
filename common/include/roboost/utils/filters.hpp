/**
 * @file filters.hpp
 * @author
 * @brief Utility functions and classes for filtering.
 * @version 0.2
 * @date 2024-05-17
 *
 * @copyright
 *
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
     * @brief Base class for CRTP filters.
     *
     * @tparam Derived The derived filter class.
     * @tparam T The numeric type.
     */
    template <typename Derived, typename T>
    class FilterBase
    {
    public:
        /**
         * @brief Update the filter.
         *
         * @param input The input value.
         * @return T The filtered value.
         */
        T update(T input) { return static_cast<Derived*>(this)->update(input); }

        /**
         * @brief Update the filter with target and current position.
         *
         * @param target The target value.
         * @param current_position The current position.
         * @return T The filtered value.
         */
        T update(T target, T current_position) { return static_cast<Derived*>(this)->update(target, current_position); }

        /**
         * @brief Reset the filter.
         */
        void reset() { static_cast<Derived*>(this)->reset(); }

        /**
         * @brief Get the output value.
         *
         * @return T The output value.
         */
        T get_output() const { return static_cast<const Derived*>(this)->get_output(); }

    protected:
        T output_;
    };

    template <typename T>
    class NoFilter : public FilterBase<NoFilter<T>, T>
    {
    public:
        T update(T input)
        {
            this->output_ = input;
            return this->output_;
        }

        void reset() {}

        T get_output() const { return this->output_; }
    };

    template <typename T>
    class ChainedFilter : public FilterBase<ChainedFilter<T>, T>
    {
    public:
        void addFilter(std::unique_ptr<FilterBase<ChainedFilter<T>, T>> filter) { filters_.push_back(std::move(filter)); }

        T update(T input)
        {
            T result = input;
            for (auto& filter : filters_)
            {
                result = filter->update(result);
            }
            this->output_ = result;
            return this->output_;
        }

        void reset()
        {
            for (auto& filter : filters_)
            {
                filter->reset();
            }
        }

        T get_output() const { return this->output_; }

    private:
        std::vector<std::unique_ptr<FilterBase<ChainedFilter<T>, T>>> filters_;
    };

    template <typename T>
    class LowPassFilter : public FilterBase<LowPassFilter<T>, T>
    {
    public:
        LowPassFilter(T cutoff_frequency, T sampling_time) : cutoff_frequency_(cutoff_frequency), sampling_time_(sampling_time) { computeAlpha(); }

        T update(T input)
        {
            this->output_ += alpha_ * (input - this->output_);
            return this->output_;
        }

        void reset() { this->output_ = T(0); }

        T get_output() const { return this->output_; }

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
    };

    template <typename T>
    class MovingAverageFilter : public FilterBase<MovingAverageFilter<T>, T>
    {
    public:
        MovingAverageFilter(int window_size) : window_size_(window_size) {}

        T update(T input)
        {
            input_history_.push_back(input);
            if (input_history_.size() > window_size_)
            {
                input_history_.pop_front();
            }
            T sum = std::accumulate(input_history_.begin(), input_history_.end(), T(0));
            this->output_ = sum / input_history_.size();
            return this->output_;
        }

        void reset() { input_history_.clear(); }

        T get_output() const { return this->output_; }

    private:
        std::deque<T> input_history_;
        int window_size_;
    };

    template <typename T>
    class MedianFilter : public FilterBase<MedianFilter<T>, T>
    {
    public:
        MedianFilter(int window_size) : window_size_(window_size) {}

        T update(T input)
        {
            input_history_.push_back(input);
            if (input_history_.size() > window_size_)
            {
                input_history_.pop_front();
            }
            std::vector<T> sorted_history(input_history_.begin(), input_history_.end());
            std::sort(sorted_history.begin(), sorted_history.end());
            this->output_ = sorted_history[sorted_history.size() / 2];
            return this->output_;
        }

        void reset() { input_history_.clear(); }

        T get_output() const { return this->output_; }

    private:
        std::deque<T> input_history_;
        int window_size_;
    };

    template <typename T>
    class ExponentialMovingAverageFilter : public FilterBase<ExponentialMovingAverageFilter<T>, T>
    {
    public:
        ExponentialMovingAverageFilter(T alpha) : alpha_(alpha) {}

        T update(T input)
        {
            this->output_ = alpha_ * input + (1 - alpha_) * this->output_;
            return this->output_;
        }

        void reset() { this->output_ = T(0); }

        T get_output() const { return this->output_; }

    private:
        T alpha_;
    };

    template <typename T>
    class RateLimitingFilter : public FilterBase<RateLimitingFilter<T>, T>
    {
    public:
        RateLimitingFilter(T max_rate_per_second, T update_rate) : max_increment_(max_rate_per_second / update_rate) { this->output_ = T(0); }

        T update(T input)
        {
            T increment = input - this->output_;
            increment = std::max(std::min(increment, max_increment_), -max_increment_);
            this->output_ += increment;
            return this->output_;
        }

        T update(T target, T current_position)
        {
            T increment = target - this->output_;
            increment = std::max(std::min(increment, max_increment_), -max_increment_);
            this->output_ += increment;
            return this->output_;
        }

        void reset() { this->output_ = T(0); }

        T get_output() const { return this->output_; }

    private:
        T max_increment_;
    };

    template <typename T>
    class IIRFilter : public FilterBase<IIRFilter<T>, T>
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
