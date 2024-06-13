#ifndef OBJECT_DETECTION_COMMON_RANGES_H
#define OBJECT_DETECTION_COMMON_RANGES_H

#include <vector>

namespace object_detection {

// https://codereview.stackexchange.com/questions/106773/dividing-a-range-into-n-sub-ranges
template <typename Iterator>
std::vector<std::pair<Iterator, Iterator>> make_ranges(Iterator begin,
                                                       Iterator end, size_t n) {
  std::vector<std::pair<Iterator, Iterator>> ranges;
  if (n == 0) {
    return ranges;
  }
  ranges.reserve(n);

  auto dist = std::distance(begin, end);
  if (dist == 0) {
    return ranges;
  }

  n = std::min<size_t>(n, dist);
  auto chunk = dist / n;
  auto remainder = dist % n;

  for (size_t i = 0; i < n - 1; ++i) {
    auto next_end = std::next(begin, chunk + (remainder ? 1 : 0));
    ranges.emplace_back(begin, next_end);

    begin = next_end;
    if (remainder) remainder -= 1;
  }

  // last chunk
  ranges.emplace_back(begin, end);
  return ranges;
}

}  // namespace object_detection

#endif  // OBJECT_DETECTION_COMMON_RANGES_H
