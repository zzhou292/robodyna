#pragma once
#include "ShellBatchPlasticityBinding.h"
#include <cstring>

namespace tl::fea::shell_plasticity_binding_detail {
using Status=ShellPlasticityBindingStatus;
inline ShellPlasticityBindingReport Error(Status status,const char* message,
    std::size_t entry=NoShellBindingNode,ShellBindingFamily family=ShellBindingFamily::None) noexcept {
  return {status,entry,family,message};
}
inline bool Same(double a,double b) noexcept { return std::memcmp(&a,&b,sizeof a)==0; }
inline bool Same(const material::TabulatedShellPlasticityRate& a,
                 const material::TabulatedShellPlasticityRate& b) noexcept {
  return a.enabled==b.enabled&&Same(a.cowper_symonds_c_per_s,b.cowper_symonds_c_per_s)&&
    Same(a.cowper_symonds_p,b.cowper_symonds_p)&&Same(a.cutoff_hz,b.cutoff_hz);
}
template<class Range,class Id>
std::size_t Find(const Range& range,std::size_t count,std::uint64_t id,Id field) noexcept {
  for(std::size_t i=0;i<count;++i) if(field(range[i])==id) return i;
  return NoShellBindingNode;
}
template<class Reference>
bool Matches(const Reference& r,const ShellPlasticityMaterialInput& m,
             const ShellPlasticitySectionInput& s) noexcept {
  return Same(r.young_modulus,m.young_pa)&&Same(r.poisson_ratio,m.poisson_ratio)&&
    Same(r.density,m.density_kg_m3)&&Same(r.thickness,s.thickness_m);
}
} // namespace tl::fea::shell_plasticity_binding_detail
