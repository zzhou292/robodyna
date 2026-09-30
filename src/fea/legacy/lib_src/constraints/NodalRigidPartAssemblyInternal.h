// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidPartAssemblyModel.h"
#include "../../lib_utils/BoundedArena.h"
#include "../../lib_utils/SourceIdentityIndex.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace tl::fea::rigid::part_assembly_detail {
using S=PartAssemblyStatus;
using Report=PartAssemblyReport;
inline Report Fail(S s,const char* m,std::size_t p=SIZE_MAX,std::size_t n=SIZE_MAX) noexcept {
  return {s,m,p,n};
}
inline bool Same(double a,double b) noexcept {return std::memcmp(&a,&b,sizeof(a))==0;}
inline bool Finite(Vec3 a) noexcept {
  return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);
}
struct Layout {
  util::ArenaRegion members,originals,roots,lookup;
  std::size_t arena=0,retained=0,startup=0;
};
struct Storage {
  explicit Storage(const NodalCoefficientLedger& c):coefficients(c) {}
  NodalCoefficientLedger coefficients;
  NodalRigidPartTopology topology;
  NodalRigidSourceUnits units{};
  util::HostArena arena;
  PartAssemblyMember* members=nullptr;
  PartAssemblyOriginal* originals=nullptr;
  PartAssemblyRoot* roots=nullptr;
  std::size_t* lookup=nullptr;
  std::size_t retained=0,startup=0;
};
Report Preflight(const NodalRigidPartTopology&,const NodalCoefficientLedger&,
                 NodalRigidSourceUnits,PartAssemblyLimits,std::size_t,Layout&) noexcept;
Report Clone(const NodalRigidPartTopology&,NodalRigidPartTopology&);
bool SameTopology(const NodalRigidPartTopology&,const NodalRigidPartTopology&) noexcept;
Report MapAndAssemble(Storage&);
} // namespace tl::fea::rigid::part_assembly_detail
