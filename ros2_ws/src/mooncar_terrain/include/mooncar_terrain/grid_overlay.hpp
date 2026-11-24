#pragma once
#include <algorithm>
namespace mooncar_terrain {
// Sparse terrain evidence adds risk. Missing/free overlay cells cannot erase
// another source's hazard or declare unobserved space traversable.
inline unsigned char mergeRisk(unsigned char original, unsigned char overlay) {
  if (overlay == 255 || overlay == 0) return original;
  return original == 255 ? overlay : std::max(original, overlay);
}
}
