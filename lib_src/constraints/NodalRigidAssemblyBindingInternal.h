// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidAssemblyBinding.h"
#include "../assembly/NodalDomainIdentity.h"
#include "lib_utils/BoundedArena.h"
#include "lib_utils/SourceIdentityIndex.h"
#include <algorithm>
#include <new>

namespace tl::fea::rigid_binding_detail {
using Report=RigidBindingReport;
using S=RigidBindingStatus;
inline Report Fail(S s,const char* message,std::size_t g=SIZE_MAX,std::size_t m=SIZE_MAX) {
  return {s,message,g,m};
}
struct Layout {
  util::ArenaRegion groups,members,lookup;
  std::size_t arena=0,owned=0,startup=0;
};
struct Storage {
  explicit Storage(const rigid::NodalRigidPartAssemblyModel& p):parts(p) {}
  explicit Storage(const NodalCoefficientLedger& source):empty_coefficients(source),empty_scope(true) {}
  rigid::NodalRigidPartAssemblyModel parts;
  NodalCoefficientLedger empty_coefficients;
  bool empty_scope=false;
  util::HostArena arena;
  RigidBindingGroup* groups=nullptr;
  RigidBindingMember* members=nullptr;
  std::size_t* lookup=nullptr;
  std::size_t group_count=0,member_count=0,owned=0,startup=0;
  std::uint64_t plain_source=0;
};
Report Forecast(const rigid::NodalRigidPartAssemblyModel&,const NodalRigidGroupModel*,
    RigidBindingLimits,std::size_t header,Layout&) noexcept;
Report Bind(Storage&,const NodalRigidGroupModel*);
} // namespace tl::fea::rigid_binding_detail
