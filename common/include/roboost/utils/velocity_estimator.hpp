// TODO

#ifndef VELOCITY_ESTIMATOR_HPP
#define VELOCITY_ESTIMATOR_HPP

#include <vector>

namespace roboost
{
    namespace numeric
    {
        class VelocityEstimator
        {
        private:
            std::vector<double> times;
            std::vector<double> positions;
            int windowSize;

        public:
            VelocityEstimator(int windowSize) : windowSize(windowSize) {}

            void addSample(double time, double position)
            {
                times.push_back(time);
                positions.push_back(position);
                if (times.size() > windowSize)
                {
                    times.erase(times.begin());
                    positions.erase(positions.begin());
                }
            }

            double computeVelocity()
            {
                double sumTime = 0, sumPosition = 0, sumTimePos = 0, sumTimeSquared = 0;
                double avgTime = 0, avgPosition = 0;

                for (int i = 0; i < times.size(); ++i)
                {
                    sumTime += times[i];
                    sumPosition += positions[i];
                }
                avgTime = sumTime / times.size();
                avgPosition = sumPosition / times.size();

                for (int i = 0; i < times.size(); ++i)
                {
                    sumTimePos += (times[i] - avgTime) * (positions[i] - avgPosition);
                    sumTimeSquared += (times[i] - avgTime) * (times[i] - avgTime);
                }

                return sumTimePos / sumTimeSquared;
            }
        };

    } // namespace numeric
} // namespace roboost

#endif // VELOCITY_ESTIMATOR_HPP
