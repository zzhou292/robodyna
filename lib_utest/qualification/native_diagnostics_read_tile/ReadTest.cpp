// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../native_contact_diagnostics/TestSupport.h"
#include "lib_src/collision/radioss_type25/runtime/diagnostics/Read.h"
#include <limits>
#include <cmath>
namespace native_diagnostic_test {
TEST(NativeDiagnosticReadTile, MaskedRowsAdmitNullUnreadOperands) {
  rd::Device device;std::uint32_t flag=0;device.positive_flags=&flag;
  const auto value=rd::diagnostics::Read(device,0);
  EXPECT_EQ(value.positive,0u);EXPECT_EQ(value.contact_active,0u);
  EXPECT_EQ(Bits(value.elastic_energy),Bits(0.));
  EXPECT_EQ(Bits(value.damping_work),Bits(0.));EXPECT_EQ(Bits(value.friction_work),Bits(0.));
}
TEST(NativeDiagnosticReadTile, PermutationAndNonbooleanFlagsPreserveExactReadFields) {
  Input input(3);input.flags={2,0,UINT32_MAX};input.slots={2,UINT32_MAX,0};
  input.response[2].normal.elastic_energy=-0.;input.response[2].normal.damping_work=std::nan("17");
  input.response[0].friction_work=-std::numeric_limits<double>::infinity();
  rd::Device device;device.positive_flags=input.flags.data();device.sorted_slots=input.slots.data();
  device.responses=input.response.data();rd::diagnostics::Tile tile{};
  for(unsigned i=0;i<3;++i) {
    const auto value=rd::diagnostics::Read(device,i);rd::diagnostics::Store(tile,i,value);
    EXPECT_EQ(value.positive,input.flags[i]!=0);EXPECT_EQ(tile.positive[i],value.positive);
    if(!input.flags[i])continue;
    const auto& source=input.response[input.slots[i]];
    EXPECT_EQ(tile.contact_active[i],source.contact_active);
    EXPECT_EQ(Bits(tile.elastic_energy[i]),Bits(source.normal.elastic_energy));
    EXPECT_EQ(Bits(tile.damping_work[i]),Bits(source.normal.damping_work));
    EXPECT_EQ(Bits(tile.friction_work[i]),Bits(source.friction_work));
  }
}
} // namespace native_diagnostic_test
