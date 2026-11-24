#include "GatePolicy.hh"
#include <cassert>
#include <limits>
int main() {
  using mooncar::PureTurn;
  assert(!PureTurn({0,0,0,0}));
  assert(!PureTurn({1,1,1,1}));
  assert(!PureTurn({-1,-1,-1,-1}));
  assert(PureTurn({-1,1,-1,1}));
  assert(PureTurn({1,-1,1,-1}));
  assert(PureTurn({-.960268221788626,.960268221788626,-1.039731778211374,1.039731778211374}));
  assert(!PureTurn({.5,1.5,.5,1.5}));
  bool rejected=false;
  try {PureTurn({std::numeric_limits<double>::quiet_NaN(),0,0,0});}
  catch(const std::invalid_argument &) {rejected=true;}
  assert(rejected);
}
