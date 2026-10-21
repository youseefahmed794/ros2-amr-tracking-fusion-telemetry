#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace amr_control
{
inline double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline std::array<double, 5> reference(double t)
{
  constexpr double scale = 3.0, w = 0.25;
  const double x = scale * std::sin(w * t);
  const double y = scale * std::sin(w * t) * std::cos(w * t);
  const double dx = scale * w * std::cos(w * t);
  const double dy = scale * w * std::cos(2 * w * t);
  const double ddx = -scale * w * w * std::sin(w * t);
  const double ddy = -2 * scale * w * w * std::sin(2 * w * t);
  const double v = std::hypot(dx, dy);
  return {x, y, std::atan2(dy, dx), v, (dx * ddy - dy * ddx) / std::max(v * v, 1e-6)};
}

struct Gains
{
  double kx{1.4}, ky{2.8}, ktheta{2.0}, ki_v{0.3};
  double integral_limit{0.4}, v_max{1.5}, omega_max{2.0}, accel_max{2.5};
};

class Tracker
{
public:
  explicit Tracker(Gains gains = {}) : gains_(gains)
  {
    for (const double value : {gains.kx, gains.ky, gains.ktheta, gains.ki_v,
        gains.integral_limit, gains.v_max, gains.omega_max, gains.accel_max})
    {
      if (!std::isfinite(value) || value < 0) {
        throw std::invalid_argument("Gains and limits must be finite and nonnegative");
      }
    }
  }

  void reset() { integral_ = 0; previous_v_ = 0; }

  std::array<double, 2> step(const std::array<double, 3> & pose, double t, double dt)
  {
    if (!std::isfinite(t) || !std::isfinite(dt) || dt <= 1e-5 || dt > 0.2 ||
      !std::all_of(pose.begin(), pose.end(), [](double v) {return std::isfinite(v);}))
    {
      reset();
      return {0.0, 0.0};
    }
    const auto ref = reference(t);
    const double dx = ref[0] - pose[0], dy = ref[1] - pose[1];
    const double ex = std::cos(pose[2]) * dx + std::sin(pose[2]) * dy;
    const double ey = -std::sin(pose[2]) * dx + std::cos(pose[2]) * dy;
    const double et = normalize_angle(ref[2] - pose[2]);
    integral_ = std::clamp(integral_ + ex * dt, -gains_.integral_limit, gains_.integral_limit);
    const double raw = ref[3] * std::cos(et) + gains_.kx * ex + gains_.ki_v * integral_;
    const double v = std::clamp(std::clamp(raw, previous_v_ - gains_.accel_max * dt,
        previous_v_ + gains_.accel_max * dt), -gains_.v_max, gains_.v_max);
    const double omega = std::clamp(ref[4] + ref[3] *
        (gains_.ky * ey + gains_.ktheta * std::sin(et)), -gains_.omega_max, gains_.omega_max);
    previous_v_ = v;
    return {v, omega};
  }

private:
  Gains gains_;
  double integral_{0.0}, previous_v_{0.0};
};
}  // namespace amr_control
