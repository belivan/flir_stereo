#ifndef OBJECT_DETECTION_COMMON_MATH_H
#define OBJECT_DETECTION_COMMON_MATH_H

#include <cstdlib>
#include <limits>

namespace object_detection {

// https://en.cppreference.com/w/cpp/algorithm/clamp
template <typename T>
constexpr const T clamp(const T& v, const T& lo, const T& hi) {
  return std::max(std::min(v, hi), lo);
}

// SubT values that we'll be comparing (positions, confidences, etc) are
// typically < 1000, so this epsilon is valid. See these links for more
// discussion and theory:
// https://stackoverflow.com/questions/17333/what-is-the-most-effective-way-for-float-and-double-comparison
// https://stackoverflow.com/questions/48133572/what-can-stdnumeric-limitsdoubleepsilon-be-used-for
template <typename T>
bool almost_equal(const T a, const T b) {
  return std::abs(a - b) < (std::numeric_limits<T>::epsilon() * 1000);
}
}  // namespace object_detection
#endif  // OBJECT_DETECTION_COMMON_MATH_H
