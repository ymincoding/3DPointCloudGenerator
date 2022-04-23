
#include <cmath>
#include <limits>

namespace core {

template <typename T>
T scalarProduct(int size, T const* array0, T const* array1)
{
    T result{0};
    for (int i = 0; i < size; ++i)
        result += (*array0++) * (*array1++);
    return result;
}

//----------------------------------------------------------------------------------------------------------------------

template <typename T>
T computeSSD(int size, T const* array0, T const* array1)
{
    T sum{0};
    T diff{0};
    for (int i = 0; i < size; ++i)
    {
        diff = (*array0++) - (*array1++);
        sum += diff * diff;
    }
    return sum;
}

//----------------------------------------------------------------------------------------------------------------------

template <typename T>
inline bool equal(T a, T b)
{
    return std::fabs(a - b) <= std::numeric_limits<T>::epsilon();
}

//----------------------------------------------------------------------------------------------------------------------

template <typename T>
inline bool zero(T a)
{
    return std::fabs(a) <= std::numeric_limits<T>::epsilon();
}

//----------------------------------------------------------------------------------------------------------------------

template <typename T>
inline bool lessThan(T a, T b)
{
    return a < (b - std::numeric_limits<T>::epsilon());
}

//----------------------------------------------------------------------------------------------------------------------

template <typename T>
inline bool lessThanOrEqual(T a, T b)
{
    return a <= (b + std::numeric_limits<T>::epsilon());
}

//----------------------------------------------------------------------------------------------------------------------

template <typename T>
inline bool greaterThan(T a, T b)
{
    return a > (b + std::numeric_limits<T>::epsilon());
}

} // namespace core
