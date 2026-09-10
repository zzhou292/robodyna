#pragma once
#include "lib_src/elements/qeph/QephLayeredLaw1.h"
#include "lib_src/elements/t3/T3LayeredLaw1.h"
#include "../shell_layered_j2/native_recurrence/LayeredNativePath.h"
#include "native/NativeLaw1Reference.h"
#include <cstring>
#include <stdexcept>
namespace layered_law1_force_test {
namespace path=layered_j2_test::recurrence;
namespace q=tl::fea::qeph;namespace t=tl::fea::t3;namespace sec=tl::fea::sections;
namespace nq=tl::qualification::qeph;namespace nt=tl::qualification::t3;
namespace native=tl::qualification::layered_law1_native;
namespace mat=tl::material;
using Parameters=mat::ShellElasticLaw1PointParameters;
template<class Input> Parameters Material(Input& in,double e=200e9) {
  in.young_modulus=e;in.poisson_ratio=.3;in.density=7890;in.thickness=.002;
  Parameters p;if(!mat::PrepareShellElasticLaw1Point(in.young_modulus,in.poisson_ratio,in.density,p))
    throw std::runtime_error("Invalid fixture material");return p;
}
template<class T> auto Bytes(const T& v) {std::array<unsigned char,sizeof(T)> b{};std::memcpy(b.data(),&v,sizeof v);return b;}
template<class ForceTrial> double ForceDifference(const ForceTrial& a,const ForceTrial& b) {
  double worst=0;
  for(unsigned i=0;i<a.internal_force.size();++i) {
    const auto& x=a.internal_force[i];const auto& y=b.internal_force[i];
    worst=std::max(worst,std::abs(x.x-y.x));worst=std::max(worst,std::abs(x.y-y.y));
    worst=std::max(worst,std::abs(x.z-y.z));
  }return worst;
}
inline void Points(const sec::ShellLayeredLaw1History& h,const native::Points& n) {
  for(unsigned p=0;p<3;++p)for(unsigned c=0;c<5;++c)
    path::cv::source::Close(h.point[p].stress[c],n[p][c],1.e-8);
}
} // namespace layered_law1_force_test
