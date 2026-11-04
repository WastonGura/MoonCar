#include "Law.hh"
#include <iomanip>
#include <iostream>
#include <limits>
int main(int argc,char **argv){
  if(argc!=4)throw std::runtime_error("Expected front/rear profiles and probe output");
  std::ofstream out(argv[3]);out<<std::setprecision(17)<<"axle,lambda,ratio,coefficient,initial_slope\n";
  if(!out)throw std::runtime_error("Math probe unavailable");
  for(int axle=0;axle<2;++axle){
    const auto law=mooncar::IntegralLaw::Load(argv[axle+1]);double previous=0;
    for(int i=-1;i<=720;++i){
      const double lambda=i<0 ? 0 : std::pow(10.,-12+i/30.);
      const double q=law.Ratio(lambda),c=law.Compliance(.03*lambda,.03);
      if(!std::isfinite(q+c) || q<0 || q>law.mu+1e-14 || q<previous-1e-14 || c<0)
        throw std::runtime_error("Bounds, monotonicity or finite compliance failed");
      if(std::abs(law.Ratio(-lambda)-q)>1e-14 || law.Compliance(-.03*lambda,.03)!=c)
        throw std::runtime_error("Resisting force symmetry failed");
      out<<axle<<','<<lambda<<','<<q<<','<<c<<','<<law.Slope()<<'\n';previous=q;
    }
    if(std::abs(law.Ratio(1e-10)/1e-10/law.Slope()-1)>1e-8 ||
       std::abs((law.mu-law.Ratio(1e7))*1e7/(law.cohesion/law.extent)-1)>1e-4 ||
       law.Ratio(std::numeric_limits<double>::infinity())!=law.mu ||
       law.Compliance(0,0)!=0 || law.Compliance(.02,0)!=.02/(law.normal*law.mu))
      throw std::runtime_error("Initial tangent, algebraic tail or zero-spin limit failed");
    bool rejected=false;try{law.Compliance(.01,-1);}catch(const std::invalid_argument &){rejected=true;}
    if(!rejected)throw std::runtime_error("Invalid rolling input accepted");
  }
  out.close();if(out.fail())throw std::runtime_error("Math probe close failed");
  std::cout<<"Integral math properties passed, 1444 cross-language probes exported\n";
}
