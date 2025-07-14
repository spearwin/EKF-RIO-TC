#pragma once

#include <rio_utils/data_types.h>
#include <sensor_msgs/FluidPressure.h>

namespace rio {
/**
 * @brief The BaroAltimeter class converts fluid pressure into relative height
 * measurements
 */
class BaroAltimeter {
 public:
  /**
   * @brief Pressure to negative height conversion
   * @param pressure_msg   fluid pressure ros message containing the air
   * pressure in Pascal
   * @returns the negative height in [m]
   */
  Real calculate_rel_neg_height(
      const sensor_msgs::FluidPressure& pressure_msg) {
    return (R * T_0) / (g_0 * M) * std::log(pressure_msg.fluid_pressure / P_0);
  }

 private:
  const std::string kPrefix = "[BaroAltimeter]: ";
  const Real g_0 = 9.80665;
  const Real M = 0.0289644;
  const Real R = 8.3144598;

  const Real T_0 = 288.15;
  const Real P_0 = 101320;
};

}  // namespace rio
