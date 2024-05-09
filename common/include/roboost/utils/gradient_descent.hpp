#include <cmath>
#include <functional>
#include <vector>

class GradientDescent
{
public:
    std::vector<double> parameters;
    double learning_rate;
    std::function<double(const std::vector<double>&)> cost_function;
    std::vector<double> param_min, param_max;

    GradientDescent(const std::vector<double>& init_params, double lr, std::function<double(const std::vector<double>&)> cost_func, const std::vector<double>& min_params,
                    const std::vector<double>& max_params)
        : parameters(init_params), learning_rate(lr), cost_function(cost_func), param_min(min_params), param_max(max_params)
    {
    }

    void compute_gradient(std::vector<double>& gradients)
    {
        double h = 1e-8;
        for (size_t i = 0; i < parameters.size(); ++i)
        {
            std::vector<double> params_plus_h = parameters;
            params_plus_h[i] += h;
            double cost_at_params = cost_function(parameters);
            double cost_at_params_plus_h = cost_function(params_plus_h);
            gradients[i] = (cost_at_params_plus_h - cost_at_params) / h;
        }
    }

    void update_parameters(const std::vector<double>& gradients)
    {
        for (size_t i = 0; i < parameters.size(); ++i)
        {
            parameters[i] -= learning_rate * gradients[i];
            if (!param_min.empty() && !param_max.empty())
            {
                parameters[i] = std::max(param_min[i], std::min(param_max[i], parameters[i]));
            }
        }
    }

    void optimize(int iterations)
    {
        std::vector<double> gradients(parameters.size(), 0.0);
        for (int i = 0; i < iterations; ++i)
        {
            compute_gradient(gradients);
            update_parameters(gradients);
        }
    }

    // Getter methods to access internal state
    const std::vector<double>& getParameters() const { return parameters; }

    double getParameterAtIndex(size_t index) const
    {
        if (index < parameters.size())
        {
            return parameters[index];
        }
        return 0; // Return a sensible default or handle error
    }
};
