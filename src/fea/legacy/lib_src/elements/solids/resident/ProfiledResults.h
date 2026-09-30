// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Results.h"
#include "controlled/Observation.h"
#include "lib_src/elements/solid24/controlled_hourglass/Types.h"
#include <new>
namespace tl::fea::solids {
struct NativeHistory24 {
  solid24::controlled_hourglass::HistoryValues values;
  double distortion_energy=0;
};
struct NativeHistory90 {
  solid18::total_strain::HistoryValues values;
  double distortion_energy=0;
};
// Numeric snapshots contain no reference objects, curve views or device pointers.
// An inactive legacy payload cannot be accessed as native force-like HG history.
template<class Legacy,class Native> class NumericHistory {
  union Values {Legacy legacy;Native native;Values()noexcept:legacy{}{}} values_;
  ResultProfile profile_=ResultProfile::Legacy;
 public:
  NumericHistory()noexcept=default;
  void SetLegacy(const Legacy& value)noexcept {
    if(profile_!=ResultProfile::Legacy)::new(static_cast<void*>(&values_.legacy)) Legacy(value);else values_.legacy=value;
    profile_=ResultProfile::Legacy;
  }
  void SetNative(const Native& value)noexcept {
    if(profile_!=ResultProfile::NativeControlled)::new(static_cast<void*>(&values_.native)) Native(value);else values_.native=value;
    profile_=ResultProfile::NativeControlled;
  }
  ResultProfile profile()const noexcept{return profile_;}
  const Legacy* legacy()const noexcept{return profile_==ResultProfile::Legacy?&values_.legacy:nullptr;}
  const Native* native()const noexcept{return profile_==ResultProfile::NativeControlled?&values_.native:nullptr;}
};
struct MaterialObservationSI {
  double stress_pa[6]{};
  double density_kg_m3=0,internal_energy_density_j_m3=0,bulk_pressure_pa=0;
};
struct SIForceObservation8 {
  solid18::Vec3 rhs_force_n[8]{};
  NodalStiffness stiffness;
  ControlledObservation response; // SI work/stiffness; its units stamp describes native operands.
};
struct ProfiledResult24 {
  NumericHistory<solid24::HistoryValues,NativeHistory24> history;
  tlfea::contact::radioss_type25::UnitScale history_units{1,1,1};
  solid24::HistoryStamp stamp;
  SIForceObservation8 cache;
  MaterialObservationSI material_si;
};
struct ProfiledResult18Law90 {
  NumericHistory<solid18::total_strain::HistoryValues,NativeHistory90> history;
  tlfea::contact::radioss_type25::UnitScale history_units{1,1,1};
  solid18::HistoryStamp stamp;
  SIForceObservation8 cache;
  solid18::total_strain::GlobalHistory global_si;
  MaterialObservationSI material_si[8]{};
};
static_assert(std::is_trivially_copyable_v<ProfiledResult24>&&std::is_trivially_copyable_v<ProfiledResult18Law90>);
struct ProfiledResultBuffers {
  Result18* solid18=nullptr;std::size_t count18=0;
  ProfiledResult24* solid24=nullptr;std::size_t count24=0;
  Result6z* solid6z=nullptr;std::size_t count6z=0;
  Result18Law44* solid18_law44=nullptr;std::size_t count18_law44=0;
  ProfiledResult18Law90* solid18_law90=nullptr;std::size_t count18_law90=0;
};
} // namespace tl::fea::solids
