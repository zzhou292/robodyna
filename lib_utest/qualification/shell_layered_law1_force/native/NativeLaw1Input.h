#pragma once
#include "NativeLaw1Reference.h"
#include "../../shell_layered_j2/native_recurrence/NativeLayeredInput.h"
namespace tl::qualification::layered_law1_native::detail {
using layered_native::detail::SectionContext;
using layered_native::detail::Disjoint;
inline bool Finite(const Points& points) noexcept {
  for(const auto& p:points)for(double v:p)if(!std::isfinite(v))return false;
  return true;
}
template<class R,class H,class I,class O>
bool OutputDisjoint(const R& r,const H& h,const I& in,const O& out) noexcept {
  return Disjoint(&out,sizeof out,&r,sizeof r)&&Disjoint(&out,sizeof out,&h,sizeof h)&&
    Disjoint(&out,sizeof out,&in,sizeof in);
}
} // namespace tl::qualification::layered_law1_native::detail
