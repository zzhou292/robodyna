// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_utest/qualification/extended_solid_model/Fixture.h"
#include "lib_utest/qualification/extended_solid_model/ResidentConfig.h"
#include "lib_src/elements/solids/resident/MaterialUpload.h"
#include "lib_src/elements/solids/resident/Measure.h"
#include <cstring>
#include <limits>
namespace extended_resident_test {
namespace fe = tl::fea;
namespace s = fe::solids;
namespace d = s::batch_detail;
using Fixture = extended_model_test::Fixture;
inline s::BatchConfig Config(std::size_t nodes) {
  auto config = extended_model_test::ResidentConfig(nodes);
  config.profile = s::BatchProfile::PhysicalCinExtendedLaw44Law90V2;
  return config;
}
template<class Traits> void Fill(const typename Traits::Parent& parent,
    typename Traits::Interval& interval, double scale) {
  for (unsigned n = 0; n < Traits::nodes; ++n) {
    auto x = parent.reference.input().position_m[n];
    const fe::solid18::Vec3 v{.001*x.x,-.002*x.y,.003*x.z};
    x.x *= scale; x.y *= scale; x.z *= scale;
    Traits::Node(interval,n,x,v);
  }
}
} // namespace extended_resident_test
