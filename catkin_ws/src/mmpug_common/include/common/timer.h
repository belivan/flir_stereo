#ifndef OBJECT_DETECTION_COMMON_TIMER_H
#define OBJECT_DETECTION_COMMON_TIMER_H

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

namespace object_detection {

// Simple timer class for profiling execution time of functions.
//
// A summary of execution time (min, max, average, number of calls) is printed
// to stderr upon destruction of this timer.
//
// Example:
//   Timer timer;
//   timer.tic("section 1");
//   timer.tic("section 2");
//   ...
//   timer.toc("section 2");
//
//   timer.tic("section 3");
//   ...
//   timer.toc("section 3");
//   timer.toc("section 1");
class Timer {
  struct TimeInfoStart {
    std::chrono::high_resolution_clock::time_point wall_start;
    std::clock_t clock_start;
  };

  struct TimeInfo {
    uint32_t count;
    uint64_t elapsed_us_wall;
    uint64_t min_us_wall = std::numeric_limits<uint64_t>::max();
    uint64_t max_us_wall;
    std::clock_t elapsed_us_clock;
    std::clock_t min_us_clock = std::numeric_limits<std::clock_t>::max();
    std::clock_t max_us_clock;
  };

 public:
  Timer(const std::string& name = "[No name specified]") : name(name){};

  void set_name(const std::string& new_name) { name = new_name; }

  void tic(const std::string& section) {
    starts[section].wall_start = std::chrono::high_resolution_clock::now();
    starts[section].clock_start = std::clock();
  }

  void toc(const std::string& section) {
    if (starts.count(section) == 0) {
      // toc called before tic() ever called. Warn user.
      std::cerr << "toc for section " << section << " called before tic."
                << std::endl;
      return;
    }

    const auto current = std::chrono::high_resolution_clock::now();
    time_infos[section].count++;
    const uint64_t elapsed_wall =
        std::chrono::duration_cast<std::chrono::microseconds>(
            current - starts[section].wall_start)
            .count();
    time_infos[section].elapsed_us_wall += elapsed_wall;
    time_infos[section].min_us_wall =
        std::min(time_infos[section].min_us_wall, elapsed_wall);
    time_infos[section].max_us_wall =
        std::max(time_infos[section].max_us_wall, elapsed_wall);

    const auto elapsed_clock = static_cast<std::clock_t>(1e6) *
                               (std::clock() - starts[section].clock_start) /
                               CLOCKS_PER_SEC;
    time_infos[section].elapsed_us_clock += elapsed_clock;
    time_infos[section].min_us_clock =
        std::min(time_infos[section].min_us_clock, elapsed_clock);
    time_infos[section].max_us_clock =
        std::max(time_infos[section].max_us_clock, elapsed_clock);
  }

  ~Timer() {
    auto flags = std::cerr.flags();
    std::cerr << std::setprecision(3) << std::fixed;

    const size_t width = 12;
    const size_t name_width = std::accumulate(
        begin(time_infos), end(time_infos), width,
        [](const size_t i,
           const std::unordered_map<std::string, TimeInfo>::value_type& pair) {
          return std::max(i, pair.first.length());
        });

    // Set up the titlebar
    std::cerr << "Timer: " << name << "\n";
    std::cerr << std::setw(name_width) << "";

    for (int i = 0; i < 4; ++i) {
      eprint_fixed("Wall");
    }
    for (int i = 0; i < 4; ++i) {
      eprint_fixed("Clock");
    }
    std::cerr << "\n";

    std::cerr << std::setw(name_width) << "";
    for (int i = 0; i < 2; ++i) {  // Once for Wall, once for Clock
      eprint_fixed("Avg Hz");
      eprint_fixed("Min ms");
      eprint_fixed("Avg ms");
      eprint_fixed("Max ms");
    }
    eprint_fixed("Count");
    std::cerr << "\n";

    std::vector<std::string> sections;
    sections.reserve(time_infos.size());
    for (const auto& pair : time_infos) {
      sections.push_back(pair.first);
    }
    std::sort(sections.begin(), sections.end());

    for (const auto& section : sections) {
      const auto& time_info = time_infos.at(section);
      std::cerr << std::setw(name_width) << section;
      // Wall time
      eprint_fixed(1.0e6 /
                   (time_info.elapsed_us_wall / (double)time_info.count));
      eprint_fixed(time_info.min_us_wall / 1000.0);
      eprint_fixed((time_info.elapsed_us_wall / (double)time_info.count) /
                   1000.0);
      eprint_fixed(time_info.max_us_wall / 1000.0);
      // Clock time
      eprint_fixed(1.0e6 /
                   (time_info.elapsed_us_clock / (double)time_info.count));
      eprint_fixed(time_info.min_us_clock / 1000.0);
      eprint_fixed((time_info.elapsed_us_clock / (double)time_info.count) /
                   1000.0);
      eprint_fixed(time_info.max_us_clock / 1000.0);

      eprint_fixed(time_info.count);
      std::cerr << "\n";
    }

    std::cerr.flags(flags);
    std::cerr << std::flush;
  }

 private:
  const size_t FIXED_WIDTH = 12;

  template <typename T>
  void eprint_fixed(const T& t) {
    std::cerr << std::setw(FIXED_WIDTH) << t;
    return;
  }

  std::string name;
  std::unordered_map<std::string, TimeInfoStart> starts;
  std::unordered_map<std::string, TimeInfo> time_infos;
};

// Threadsafe version of Timer
class ThreadsafeTimer : public Timer {
 public:
  // Inherit constructors from Timer class so we don't have to retype them.
  using Timer::Timer;

  void set_name(const std::string& new_name) {
    std::lock_guard<std::mutex> timer_guard(timer_lock);
    Timer::set_name(new_name);
  }

  void tic(const std::string& section) {
    std::lock_guard<std::mutex> timer_guard(timer_lock);
    Timer::tic(section);
  }

  void toc(const std::string& section) {
    std::lock_guard<std::mutex> timer_guard(timer_lock);
    Timer::toc(section);
  }

 private:
  std::mutex timer_lock;
};

}  // namespace object_detection
#endif  // OBJECT_DETECTION_COMMON_TIMER_H
