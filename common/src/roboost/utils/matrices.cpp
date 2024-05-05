#include <roboost/utils/matrices.hpp>

using namespace roboost::math;

// Operator overloads for matrix-vector multiplication
Vector roboost::math::operator*(const Matrix& m, const Vector& v)
{
    if (m.empty() || m[0].size() != v.size())
    {
        throw std::invalid_argument("Matrix and vector dimensions must match.");
    }
    Vector result(m.size(), 0.0);
    for (size_t i = 0; i < m.size(); i++)
    {
        for (size_t j = 0; j < v.size(); j++)
        {
            result[i] += m[i][j] * v[j];
        }
    }
    return result;
}

// Operator overload for vector-scalar multiplication
Vector roboost::math::operator*(const Vector& v, double scalar)
{
    Vector result(v.size());
    for (size_t i = 0; i < v.size(); i++)
    {
        result[i] = v[i] * scalar;
    }
    return result;
}

// Function to create a zero-initialized vector
Vector Zero(size_t size) { return Vector(size, 0.0); }

Matrix Zero(size_t size_x, size_t size_y) { return Matrix(size_x, Vector(size_y, 0.0)); }