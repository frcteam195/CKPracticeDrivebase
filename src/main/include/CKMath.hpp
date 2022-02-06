#pragma once

namespace ck
{
    namespace math
    {
        template <typename T>
        inline int signum(T val)
        {
            return (T(0) < val) - (val < T(0));
        }

        template <typename T>
        T normalizeWithDeadband(T val, T deadband) {
            val = (std::fabs(val) > std::fabs(deadband)) ? val : 0.0;

            if (val != 0)
            {
                val = signum(val) * ((std::fabs(val) - deadband) / (1.0 - deadband));
            }

            return (std::fabs(val) > std::fabs(deadband)) ? val : 0.0;
        }
    }
}