
#ifndef SS22_PROJECT_TEST_CONSTANTS_H_
#define SS22_PROJECT_TEST_GEN_CONSTANTS_H_

#include <limits>

namespace core {

const double Pi{3.14159265358979323846264338327950288419716939937510582097};

// 2*PI
const double TwoPi{6.2831853071795864769252867665590057683943387987502116419};

// PI/2
const double HalfPi{1.5707963267948966192313216916397514420985846996875529105};

const double EPS{std::numeric_limits<double>::epsilon()};

const double ToDeg{180. / Pi};
const double ToRad{Pi / 180.};

const double PiInverted(1. / Pi);
const double TwoPiInverted(.5 / Pi);

} // namespace core

#endif // SS22_PROJECT_TEST_CONSTANTS_H_
