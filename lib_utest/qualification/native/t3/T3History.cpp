#include "T3History.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace tl::qualification::t3 {
namespace {
bool SameBits(double a,double b) noexcept {
  std::uint64_t x,y; static_assert(sizeof(x)==sizeof(a));
  std::memcpy(&x,&a,sizeof(x)); std::memcpy(&y,&b,sizeof(y)); return x==y;
}
template<std::size_t N> bool Finite(const std::array<double,N>& a) noexcept {
  for(double x:a) if(!std::isfinite(x)) return false;
  return true;
}
}
namespace detail {
bool ValidHistoryValues(const HistoryValues& h) noexcept {
  // Same native integer-product EM30 as constant_mod, without a clamp.
  constexpr double ep10=100000.*100000.,ep20=ep10*ep10,em30=1./(ep20*ep10);
  return Finite(h.stress)&&Finite(h.material_stress)&&Finite(h.bending_stress)&&
      Finite(h.strain_curvature)&&Finite(h.internal_work)&&
      std::isfinite(h.thickness)&&h.thickness>=em30&&
      std::isfinite(h.equivalent_strain_rate)&&h.equivalent_strain_rate>=0&&h.active==1;
}
void PackHistory(const HistoryValues& h,std::array<double,kHistoryValues>& out) noexcept {
  std::copy(h.stress.begin(),h.stress.end(),out.begin());
  std::copy(h.material_stress.begin(),h.material_stress.end(),out.begin()+5);
  std::copy(h.bending_stress.begin(),h.bending_stress.end(),out.begin()+10);
  std::copy(h.strain_curvature.begin(),h.strain_curvature.end(),out.begin()+13);
  out[21]=h.thickness; out[22]=h.internal_work[0]; out[23]=h.internal_work[1];
  out[24]=h.equivalent_strain_rate; out[25]=h.active;
}
HistoryValues UnpackHistory(const double* p) noexcept {
  HistoryValues h;
  std::copy_n(p,5,h.stress.begin()); std::copy_n(p+5,5,h.material_stress.begin());
  std::copy_n(p+10,3,h.bending_stress.begin()); std::copy_n(p+13,8,h.strain_curvature.begin());
  h.thickness=p[21]; h.internal_work={p[22],p[23]}; h.equivalent_strain_rate=p[24]; h.active=p[25];
  return h;
}
}
bool History::matches_reference(const Reference& r) const noexcept {
  if(!prepared_||!r.prepared()) return false;
  const auto& a=r.data().input; const auto& b=reference_input_;
  for(unsigned n=0;n<3;++n)
    if(a.node_ids[n]!=b.node_ids[n]||!SameBits(a.position[n].x,b.position[n].x)||
       !SameBits(a.position[n].y,b.position[n].y)||!SameBits(a.position[n].z,b.position[n].z)) return false;
  return SameBits(a.density,b.density)&&SameBits(a.young_modulus,b.young_modulus)&&
      SameBits(a.poisson_ratio,b.poisson_ratio)&&SameBits(a.thickness,b.thickness);
}
Status PreparePrescribedHistory(const Reference& r,const HistoryValues& h,HistoryStamp stamp,History& out) noexcept {
  if(!r.prepared()||!detail::ValidHistoryValues(h)||!std::isfinite(stamp.time)||stamp.time<0)
    return Status::kInvalidInput;
  History next; next.data_=h; next.stamp_=stamp; next.reference_input_=r.data().input; next.prepared_=true;
  out=next; return Status::kSuccess;
}
Status InitializeHistory(const Reference& r,HistoryStamp stamp,History& out) noexcept {
  if(!r.prepared()) return Status::kInvalidInput;
  HistoryValues initial; initial.thickness=r.data().input.thickness;
  return PreparePrescribedHistory(r,initial,stamp,out);
}
} // namespace tl::qualification::t3
