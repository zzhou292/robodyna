// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TiedClassification.h"
#include "../../../lib_utils/BoundedArena.h"
#include "../../../lib_utils/SourceIdentityIndex.h"
#include <array>
#include <climits>
#include <utility>

namespace tl::constraints::tied_shell {
struct ClassificationResult::Data {
  ClassificationPhase phase=ClassificationPhase::Empty;
  std::uint64_t source=0, kinset_warnings=0, penalty_warnings=0;
  std::size_t node_count=0, interface_count=0, slave_count=0;
  std::size_t owned=0, startup=0;
  std::unique_ptr<ClassificationNode[]> nodes;
  std::unique_ptr<ClassifiedInterface[]> interfaces;
  std::unique_ptr<std::int32_t[]> irupt;
  std::unique_ptr<std::uint32_t[]> slaves;
  std::array<std::int32_t,8192> itf{};
};
namespace classification_detail {
using Report=ClassificationReport;
using Status=ClassificationStatus;
using Index=tl::util::SourceIdentityIndex<1>;
// Bounds native five-block indexing, section linked records and six-direction
// warning counters even when a caller increases its configured count limits.
inline constexpr std::size_t NativeCountCap=INT_MAX/64;
inline Report Fail(Status status,std::size_t row=SIZE_MAX,std::size_t member=SIZE_MAX) noexcept {
  return {status,row,member};
}
template<class T> bool Span(ClassificationView<T> view) noexcept {
  if(!view.count) return true;
  const auto address=reinterpret_cast<std::uintptr_t>(view.data);
  return view.data && address%alignof(T)==0 && view.count<=SIZE_MAX/sizeof(T) &&
         address<=UINTPTR_MAX-view.count*sizeof(T);
}
inline bool Mask(std::int32_t value) noexcept { return value>=0 && value<8192; }
inline bool Directions(std::int32_t value) noexcept { return value>=0 && value%10<=7; }
struct Counts {
  std::size_t slaves=0, occurrences=0, owned=0, startup=0;
};
Report Preflight(const ClassificationInput&,const ClassificationResult&,ClassificationLimits,std::size_t,Counts&) noexcept;
Report Preflight(const RigidRegistrationInput&,const ClassificationResult&,ClassificationLimits,std::size_t,Counts&) noexcept;
Report CheckContext(const ClassificationContext&);
Report CheckRoles(const ClassificationInput&) noexcept;
Report CheckRoles(const RigidRegistrationInput&) noexcept;
Report CheckInterfaceIdentities(const ClassificationInput&);
Report CheckGroupIdentities(const RigidRegistrationInput&);

// Only ITF can change in these callers. Every other KINCOD table is exactly its
// KININI bit decoder; keeping ITF explicit preserves the original global effect.
inline int Decode(int kind,int code,const std::array<std::int32_t,8192>& itf) noexcept {
  return kind==2 ? itf[code] : ((code&kind)!=0);
}
bool KinSet(int kind,int direction,NativeKinematics&,NativeKinematics& scratch,
            std::array<std::int32_t,8192>&,std::uint64_t& warnings) noexcept;

template<class Data> void Initialize(const ClassificationContext& in,Counts counts,Data& out) {
  out.source=in.source_instance_id;
  out.node_count=in.nodes.count;
  out.owned=counts.owned;
  out.startup=counts.startup;
  out.nodes=std::make_unique<ClassificationNode[]>(out.node_count);
  for(std::size_t i=0;i<out.node_count;++i) out.nodes[i]=in.nodes.data[i];
  for(int i=0;i<8192;++i) out.itf[i]=(i&2)!=0;
}
template<class Data> Report RunClassification(const ClassificationInput&,Data&);
template<class Data> Report RunRigidRegistration(const RigidRegistrationInput&,Data&);
} // namespace classification_detail
} // namespace tl::constraints::tied_shell
