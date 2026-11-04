#pragma once
#include <cmath>
#include <stdexcept>
namespace mooncar {
struct LateralLaw {
  double normal,mu,weight,fast,slow;
  void Validate() const {
    if(!std::isfinite(normal+mu+weight+fast+slow) || normal<=0 || mu<=0 ||
       weight<=0 || weight>=1 || slow<=0 || fast<slow)
      throw std::invalid_argument("Invalid positive lateral mixture");
  }
  double Slope() const {return mu*(weight*fast+(1-weight)*slow);}
  double Ratio(double lambda) const {
    return mu*(weight*(-std::expm1(-fast*std::abs(lambda)))+
              (1-weight)*(-std::expm1(-slow*std::abs(lambda))));
  }
  double Compliance(double lateral,double rolling) const {
    if(!std::isfinite(lateral+rolling) || rolling<0)
      throw std::invalid_argument("Invalid local wheel velocity");
    const double speed=std::abs(lateral);
    if(rolling==0)return speed/(normal*mu);
    const double lambda=speed/rolling;
    if(lambda<1e-8)return rolling/(normal*Slope());
    return speed/(normal*Ratio(lambda));
  }
};
}
