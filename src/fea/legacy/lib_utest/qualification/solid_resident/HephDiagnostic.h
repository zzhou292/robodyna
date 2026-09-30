// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeChecks.h"
#include <cstdlib>
#include <iomanip>
#include <iostream>

namespace solid_resident_test {
inline bool HephDiagnosticEnabled() { return std::getenv("TL_SOLID_DIAG24")!=nullptr; }
// Test-only observation of the same actual owner packet. No candidate/load,
// history, constitutive formula or comparison tolerance is changed.
inline void HephDiagnostic(const tl::fea::solid24::PrescribedInterval& interval,
    const tl::fea::solid24::History& accepted,const tl::fea::solid24::ForceTrial& host,
    const s::Result24& gpu,const heph_test::NativeTrial& native,std::uint64_t epoch) {
  const auto flags=std::cout.flags();
  const auto precision=std::cout.precision();
  std::cout<<std::hexfloat<<std::setprecision(17);
  std::cout<<"HEPH_DIAG epoch "<<epoch<<" phase "<<interval.base_time_s<<' '
           <<interval.dt_s<<' '<<interval.sample_index<<'\n';
  if (epoch) {
    for (unsigned n=0;n<8;++n) {
      const auto& x=interval.position_m[n];
      const auto& v=interval.velocity_m_s[n];
      std::cout<<"HEPH_INPUT "<<n<<' '<<x.x<<' '<<x.y<<' '<<x.z<<' '
               <<v.x<<' '<<v.y<<' '<<v.z<<'\n';
    }
    const auto& h=accepted.values().material;
    std::cout<<"HEPH_BASE";
    for (double v:h.stress_pa) std::cout<<' '<<v;
    std::cout<<' '<<h.density_kg_m3<<' '<<h.internal_energy_density_j_m3<<' '
             <<h.bulk_pressure_pa<<'\n';
  }
  const auto hv=heph_test::Values(host);
  const auto gv=Values(gpu);
  for (unsigned k=0;k<hv.size();++k) {
    std::cout<<"HEPH_VALUE "<<k<<" host "<<hv[k]<<" native "<<native.values[k];
    if (k<21) std::cout<<" gpu "<<gv[k];
    else if (k>=22&&k<46) std::cout<<" gpu "<<gv[k-1];
    else if (k>=151) std::cout<<" gpu "<<gv[45+k-151];
    std::cout<<'\n';
  }
  std::cout.flags(flags);
  std::cout.precision(precision);
}
} // namespace solid_resident_test
