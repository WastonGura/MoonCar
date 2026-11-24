#pragma once
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>

namespace mooncar {
struct FeedbackLease {
  std::int64_t sequence{0},stamp{0};
  double factor{1.};
  bool valid{false};
  bool Receive(const std::string &message) {
    std::istringstream input(message);int version=0,approved=0;
    std::int64_t incoming=0,seconds=0,nanos=0;double front=1.;std::string trailing;
    if (!(input>>version>>incoming>>seconds>>nanos>>front>>approved) || input>>trailing ||
        version!=1 || incoming<=sequence || seconds<0 || seconds>86400 || nanos<0 || nanos>=1000000000 ||
        !std::isfinite(front) || front<.95 || front>1.05 || (approved!=0 && approved!=1)) {
      valid=false;return false;
    }
    const auto time=seconds*1000000000+nanos;
    if (sequence && time<=stamp) {valid=false;return false;}
    sequence=incoming;stamp=time;factor=front;valid=approved==1;return true;
  }
  bool Fresh(std::int64_t now) const {
    return valid && now>=stamp && now-stamp<=250000000;
  }
};
}
