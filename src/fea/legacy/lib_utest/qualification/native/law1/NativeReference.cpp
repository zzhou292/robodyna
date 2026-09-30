#include "NativeReference.h"
#include <algorithm>
#include <cmath>
namespace tl::qualification::law1 {
extern "C" void law1_point_physical(const double*,double,const double*,const double*,double,double,double*);
extern "C" void law1_section(const double*,double,const double*,double,double,const double*,double*);
namespace {
template<class T> bool Finite(const T& a) {for(double x:a)if(!std::isfinite(x))return false;return true;}
bool Basic(double e,double nu,double rho,double gs,double layer,double thickness) {
  return std::isfinite(e)&&e>0&&std::isfinite(nu)&&nu>=0&&nu<.5&&std::isfinite(rho)&&rho>0&&
    std::isfinite(gs)&&gs>0&&std::isfinite(layer)&&layer>0&&std::isfinite(thickness)&&thickness>=1.e-30;
}
}
bool Evaluate(const PointInput& in,PointResult* out) noexcept {
  if(!out||!Basic(in.young,in.nu,in.rho,in.gs,in.layer_thickness,in.reported_thickness)||
     !Finite(in.stress)||!Finite(in.increment))return false;
  const double basic[]{in.young,in.nu,in.rho};std::array<double,10> values{};
  law1_point_physical(basic,in.gs,in.stress.data(),in.increment.data(),in.layer_thickness,in.reported_thickness,values.data());
  if(!Finite(values)||values[5]<1.e-30)return false;
  PointResult next;std::copy_n(values.begin(),5,next.stress.begin());next.thickness=values[5];
  std::copy_n(values.begin()+6,4,next.coefficients.begin());*out=next;return true;
}
bool Evaluate(const SectionInput& in,SectionResult* out) noexcept {
  if(!out||!Basic(in.young,in.nu,in.rho,in.gs,in.reference_thickness,in.reported_thickness)||
     !Finite(in.stress)||!Finite(in.increment))return false;
  const double basic[]{in.young,in.nu,in.rho};std::array<double,27> values{};
  law1_section(basic,in.gs,in.increment.data(),in.reference_thickness,in.reported_thickness,in.stress.data(),values.data());
  if(!Finite(values))return false;
  // Declared wrapper scope requires every running physical thickness positive.
  for(unsigned i=23;i<27;++i)if(values[i]<1.e-30)return false;
  SectionResult next;std::copy_n(values.begin(),15,next.stress.begin());
  std::copy_n(values.begin()+15,5,next.force.begin());std::copy_n(values.begin()+20,3,next.moment.begin());
  next.thickness=values[23];*out=next;return true;
}
} // namespace tl::qualification::law1
