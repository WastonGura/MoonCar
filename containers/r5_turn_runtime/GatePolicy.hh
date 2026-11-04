#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
namespace mooncar {
inline bool PureTurn(const std::array<double,4> &targets) {
  for (double value:targets) if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite wheel target");
  const double left=(targets[0]+targets[2])*.5;
  const double right=(targets[1]+targets[3])*.5;
  return std::abs(left+right)<=1e-12 && std::abs(right-left)>1e-12;
}
}
