#ifndef MATRICES_HPP
#define MATRICES_HPP

#include <array>

namespace roboost
{
    namespace math
    {

        // Define vector and matrix types
        template <std::size_t N>
        using Vector = std::array<float, N>;

        template <std::size_t Rows, std::size_t Cols>
        using Matrix = std::array<std::array<float, Cols>, Rows>;

        // initialize a matrix with zeros
        template <std::size_t Rows, std::size_t Cols>
        Matrix<Rows, Cols> zeros()
        {
            Matrix<Rows, Cols> mat = {0};
            return mat;
        }

        // initialize a vector with zeros
        template <std::size_t N>
        Vector<N> zeros()
        {
            Vector<N> vec = {0};
            return vec;
        }

        // Matrix-vector multiplication
        template <std::size_t Rows, std::size_t Cols>
        Vector<Rows> multiply(const Matrix<Rows, Cols>& mat, const Vector<Cols>& vec)
        {
            Vector<Rows> result = {0};
            for (std::size_t i = 0; i < Rows; ++i)
            {
                for (std::size_t j = 0; j < Cols; ++j)
                {
                    result[i] += mat[i][j] * vec[j];
                }
            }
            return result;
        }

        // Matrix-matrix multiplication
        template <std::size_t Rows, std::size_t Cols, std::size_t Inner>
        Matrix<Rows, Cols> multiply(const Matrix<Rows, Inner>& mat1, const Matrix<Inner, Cols>& mat2)
        {
            Matrix<Rows, Cols> result = {0};
            for (std::size_t i = 0; i < Rows; ++i)
            {
                for (std::size_t j = 0; j < Cols; ++j)
                {
                    for (std::size_t k = 0; k < Inner; ++k)
                    {
                        result[i][j] += mat1[i][k] * mat2[k][j];
                    }
                }
            }
            return result;
        }

        // Scale a vector by a scalar
        template <std::size_t N>
        Vector<N> scale(const Vector<N>& vec, float scalar)
        {
            Vector<N> result;
            for (std::size_t i = 0; i < N; ++i)
            {
                result[i] = vec[i] * scalar;
            }
            return result;
        }

        // Add two vectors
        template <std::size_t N>
        Vector<N> add(const Vector<N>& vec1, const Vector<N>& vec2)
        {
            Vector<N> result;
            for (std::size_t i = 0; i < N; ++i)
            {
                result[i] = vec1[i] + vec2[i];
            }
            return result;
        }

        // Subtract two vectors
        template <std::size_t N>
        Vector<N> subtract(const Vector<N>& vec1, const Vector<N>& vec2)
        {
            Vector<N> result;
            for (std::size_t i = 0; i < N; ++i)
            {
                result[i] = vec1[i] - vec2[i];
            }
            return result;
        }

        // Transpose a matrix
        template <std::size_t Rows, std::size_t Cols>
        Matrix<Cols, Rows> transpose(const Matrix<Rows, Cols>& mat)
        {
            Matrix<Cols, Rows> result = {0};
            for (std::size_t i = 0; i < Rows; ++i)
            {
                for (std::size_t j = 0; j < Cols; ++j)
                {
                    result[j][i] = mat[i][j];
                }
            }
            return result;
        }

    } // namespace math
} // namespace roboost

#endif // MATRICES_HPP
