// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_utest/qualification/finalizer_local_control/QbatSupport.h"
#include "ReferenceMeasurement.h"
#include "lib_src/elements/qbat/mapped/measurement/Read.h"
namespace qbat_read_tile_test {
namespace old=final_control_test::qbat;
namespace f=qbat_measurement_test;
namespace q=tl::fea::qbat;
namespace b=q::batch_detail;
namespace m=q::mapped;
namespace tile=m::measurement;
using old::Seed;
using old::Poison;
using old::Same;
inline void CompareHost(f::Fixture& fixture,const q::BatchDiagnostics& seed) {
  const auto view=fixture.input.Prepared(2);
  fixture.host->control=Poison();
  m::read_tile_reference::FinalizeMeasurement(*fixture.host,view,seed,1);
  const auto expected=fixture.host->control;
  fixture.host->control=Poison();m::FinalizeMeasurement(*fixture.host,view,seed,1);
  Same(fixture.host->control,expected);
}
} // namespace qbat_read_tile_test
