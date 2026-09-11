// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../elements/ShellBatchBinding.h"
#include <cstdint>

namespace tl::fea {
enum class CoefficientOrder {
  PreparedSI_Q_T_B_Type25_Type13_V1,
  PreparedSI_Q_T_B_Type25_Type13_ElementMass_V2,
  PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_V3,
  PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_Law44_Law90_V4
};
enum class CoefficientProducer { None,Qeph,T3,Qbat,Type25,Type13,NodeTotals,ElementMass,Solid18,Solid24,Solid6z,Solid18Law44,Solid18Law90 };
enum class CoefficientStatus {
  Success,AlreadyInitialized,InvalidInput,ResourceLimit,IdentityMismatch,
  DuplicateIdentity,PositionMismatch,NonfiniteResult
};
struct CoefficientReport {
  CoefficientStatus status=CoefficientStatus::Success;
  const char* message="OK";
  CoefficientProducer producer=CoefficientProducer::None;
  std::size_t parent=SIZE_MAX,local=SIZE_MAX,node=SIZE_MAX;
  explicit operator bool() const noexcept {return status==CoefficientStatus::Success;}
};
struct CoefficientLimits {
  std::size_t max_nodes=2048,max_shell_parents=2048;
  std::size_t max_type25_connections=1024,max_type13_connections=1024;
  std::size_t max_host_bytes=16*1024*1024;
  std::size_t max_element_mass_records=4096;
  std::size_t max_solid_parents=16384;
  static constexpr CoefficientLimits Vehicle() noexcept {
    return {524288,524288,4096,8192,1024*1024*1024};
  }
};
struct CoefficientPair {double mass=0,isotropic_inertia=0;};
struct Type13CoefficientPartition {
  double mass=0,isotropic_inertia=0,added_inertia=0;
};
struct NodalCoefficientTotals {
  // Authoritative totals: never reconstructed from the diagnostic subtotals.
  double mass=0,isotropic_inertia=0;
  ShellBindingMass shell{};
  CoefficientPair type25{};
  Type13CoefficientPartition type13{};
  double element_mass=0; // Additive physical nodal mass; scalar J is untouched.
  double solid18_mass=0,solid24_mass=0,solid6z_mass=0;
  double solid18_law44_mass=0,solid18_law90_mass=0;
};
struct CoefficientOccurrences {
  std::uint64_t qeph=0,t3=0,qbat=0,type25=0,type13=0,element_mass=0;
  std::uint64_t solid18=0,solid24=0,solid6z=0;
  std::uint64_t solid18_law44=0,solid18_law90=0;
};
inline bool HasCoefficientProducer(const CoefficientOccurrences& n) noexcept {
  return n.qeph||n.t3||n.qbat||n.type25||n.type13||n.element_mass||
    n.solid18||n.solid24||n.solid6z||n.solid18_law44||n.solid18_law90;
}
struct NodalCoefficientNode {
  NodalCoefficientTotals coefficients{};
  CoefficientOccurrences occurrences{};
};
struct NodalCoefficientScope {
  std::size_t qeph_parents=0,t3_parents=0,qbat_parents=0;
  std::size_t type25_connections=0,type13_connections=0;
  std::size_t element_mass_records=0;
  std::size_t solid18_parents=0,solid24_parents=0,solid6z_parents=0;
  std::size_t solid18_law44_parents=0,solid18_law90_parents=0;
  CoefficientOccurrences occurrences{};
  std::size_t covered_nodes=0,uncovered_nodes=0;
};
} // namespace tl::fea
