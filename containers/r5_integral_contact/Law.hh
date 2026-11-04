#pragma once
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
namespace mooncar {
struct IntegralLaw {
  double normal{},mu{},cohesion{},extent{};
  std::vector<double> pressure;
  static std::pair<double,double> Kernel(double z) {
    if(z<0 || !std::isfinite(z))throw std::invalid_argument("Invalid shear kernel");
    if(z<.25){
      double left=0,right=0,term=z;
      for(int n=1;n<18;++n){left+=term/((n+1)*(n+2));right+=term/(n+2);term*=-z/(n+1);}
      return {left,right};
    }
    const double phi0=-std::expm1(-z)/z,phi1=(phi0-std::exp(-z))/z;
    return {.5-phi0+phi1,.5-phi1};
  }
  void Validate() const {
    if(!std::isfinite(normal+mu+cohesion+extent) || normal<=0 || mu<=0 ||
       cohesion<=0 || cohesion>=mu || extent<=0 || pressure.size()<5 || pressure.size()%2!=1)
      throw std::invalid_argument("Invalid continuous contact law");
    double area=0;
    for(std::size_t i=0;i<pressure.size();++i){
      if(!std::isfinite(pressure[i]) || pressure[i]<0 || std::abs(pressure[i]-pressure[pressure.size()-1-i])>1e-12)
        throw std::invalid_argument("Pressure must be positive and symmetric");
      if(i)area+=(pressure[i-1]+pressure[i])*.5/(pressure.size()-1);
    }
    if(std::abs(area-1)>1e-12 || pressure.front()!=0 || pressure.back()!=0)
      throw std::invalid_argument("Pressure normalization/entry endpoints invalid");
  }
  static IntegralLaw Load(const std::string &path) {
    std::ifstream file(path);std::string version;std::size_t n=0;IntegralLaw law;
    if(!(file>>version) || version!="mooncar_piecewise_integral_v1" || !(file>>law.normal>>law.mu>>law.cohesion>>law.extent>>n) || n>4097)
      throw std::runtime_error("Missing or invalid frozen contact profile");
    law.pressure.resize(n);
    for(auto &v:law.pressure)if(!(file>>v))throw std::runtime_error("Truncated contact profile");
    std::string extra;if(file>>extra)throw std::runtime_error("Unexpected contact profile fields");
    law.Validate();return law;
  }
  double Slope() const {return mu*extent/2;}
  double Ratio(double lambda) const {
    if(std::isnan(lambda))throw std::invalid_argument("NaN lateral ratio");
    const double z=std::abs(lambda)*extent;
    if(!std::isfinite(z))return mu;
    const auto uniform=Kernel(z);double result=cohesion*(uniform.first+uniform.second);
    const double h=1./(pressure.size()-1);const auto hat=Kernel(z*h);double q=0;
    for(std::size_t i=0;i+1<pressure.size();++i){
      const double y=z*(i*h),e=std::exp(-y),d=-std::expm1(-y);
      q+=h*((pressure[i]+pressure[i+1])*.5*d+e*(pressure[i]*hat.first+pressure[i+1]*hat.second));
    }
    return result+(mu-cohesion)*q;
  }
  double Compliance(double lateral,double rolling) const {
    if(!std::isfinite(lateral+rolling) || rolling<0)throw std::invalid_argument("Invalid local wheel speed");
    const double speed=std::abs(lateral);
    if(rolling==0)return speed/(normal*mu);
    const double lambda=speed/rolling;
    if(lambda<1e-8)return rolling/(normal*Slope());
    return speed/(normal*Ratio(lambda));
  }
};
}
