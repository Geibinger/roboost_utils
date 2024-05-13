/**
 * @file filters.cpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Utility function and class definitions for filtering.
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <math.h>
#include <roboost/utils/filters.hpp>

using namespace roboost::filters;

LowPassFilter::LowPassFilter(double cutoff_frequency, double sampling_time) : cutoff_frequency_(cutoff_frequency), sampling_time_(sampling_time)
{
    alpha_ = sampling_time_ / (sampling_time_ + 1.0f / (M_2_PI * cutoff_frequency_));
    output_ = 0.0f;
    // time_constant_ = 1.0f / (2.0f * PI * cutoff_frequency_);
}

double LowPassFilter::update(double input)
{
    output_ = alpha_ * input + (1.0f - alpha_) * output_;
    return output_;
}

void LowPassFilter::reset() { output_ = 0.0f; }

double LowPassFilter::get_cutoff_frequency() const { return cutoff_frequency_; }

double LowPassFilter::get_sampling_time() const { return sampling_time_; }

void LowPassFilter::set_cutoff_frequency(double cutoff_frequency)
{
    cutoff_frequency_ = cutoff_frequency;
    alpha_ = sampling_time_ / (sampling_time_ + 1.0f / (M_2_PI * cutoff_frequency_));
}

void LowPassFilter::set_sampling_time(double sampling_time)
{
    sampling_time_ = sampling_time;
    alpha_ = sampling_time_ / (sampling_time_ + 1.0f / (M_2_PI * cutoff_frequency_));
}

MovingAverageFilter::MovingAverageFilter(int window_size) : window_size_(window_size) {}

double MovingAverageFilter::update(double input)
{
    input_history_.push_front(input);

    if (input_history_.size() > window_size_)
        input_history_.pop_back();

    double sum = 0.0f;
    for (double value : input_history_)
        sum += value;

    output_ = sum / input_history_.size();
    return output_;
}

void MovingAverageFilter::reset() { input_history_.clear(); }

MedianFilter::MedianFilter(int window_size) : window_size_(window_size) {}

double MedianFilter::update(double input)
{
    input_history_.push_front(input);

    if (input_history_.size() > window_size_)
        input_history_.pop_back();

    std::sort(input_history_.begin(), input_history_.end());

    if (input_history_.size() % 2 == 0)
        output_ = (input_history_[input_history_.size() / 2 - 1] + input_history_[input_history_.size() / 2]) / 2.0f;
    else
        output_ = input_history_[input_history_.size() / 2];

    return output_;
}

void MedianFilter::reset() { input_history_.clear(); }

ExponentialMovingAverageFilter::ExponentialMovingAverageFilter(double alpha) : alpha_(alpha) {}

double ExponentialMovingAverageFilter::update(double input)
{
    output_ = alpha_ * input + (1.0f - alpha_) * output_;
    return output_;
}

void ExponentialMovingAverageFilter::reset() { output_ = 0.0f; }