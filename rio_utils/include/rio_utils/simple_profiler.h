#pragma once

#include <ros/console.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace rio {
struct RuntimeStatistics {
  float min_ms = 0.0f;
  float max_ms = 0.0f;
  float mean_ms = 0.0f;
  float total_ms = 0.0f;

  std::string toStringMs() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << "mean_ms=" << mean_ms
       << ", max_ms=" << max_ms << ", min_ms=" << min_ms
       << ", total_s=" << total_ms;
    return ss.str();
  }

  std::string toStringUs() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << "mean_us=" << mean_ms * 1.0e3f
       << ", max_us=" << max_ms * 1.0e3f << ", min_us=" << min_ms * 1.0e3f
       << ", total_us=" << total_ms * 1.0e3f;
    return ss.str();
  }
};

struct ProfileData {
  int id = -1;
  RuntimeStatistics statistics;
  std::vector<float> execution_ms = {};
};

/**
 * @brief The SimpleProfiler class provides a lighweight tool for runtime
 * evaluation
 */
class SimpleProfiler {
 public:
  /**
   * @brief SimpleProfiler constructor
   * @param is_on   enables runtime measurements
   */
  SimpleProfiler(const bool is_on = true);

  /**
   * @brief Starts a runtime measurement of the provided key
   */
  void start(const std::string& key);

  /**
   * @brief Stops a runtime measurement of the provided key
   * @returns true if successful (=the measurement for the given key was
   * started)
   */
  bool stop(const std::string& key);

  /**
   * @brief Stops a runtime measurement of the provided key returning the
   * runtime in [ms]
   */
  float stopWithRuntimeMs(const std::string& key);

  /**
   * @brief Gets the runtime statistics (=mean, max, min) runtimes for this key
   */
  RuntimeStatistics getStatistics(const std::string& key);

  /**
   * @brief Returns a summary of all runtime measurements as string
   * @param indent   size of whitespace indet
   */
  std::string toString(const uint indent = 0);

  /**
   * @brief Returns a summary of all runtime measurements as string formatted to
   * a markdown table
   */
  std::string toMarkdownTable();

  /**
   * @brief Returns the sum of all stopped runtimes
   */
  float getTotalRuntime();

 private:
  /**
   * @brief Helper function to calculate the runtime statistics for a given key
   */
  RuntimeStatistics calculateProfileStatistics(const std::string& key);

  const std::string kPrefix;

  bool is_on_;
  int next_id_;
  std::map<std::string, std::chrono::system_clock::time_point> start_times_;
  std::unordered_map<std::string, ProfileData> profile_data_;
};

}  // namespace rio
