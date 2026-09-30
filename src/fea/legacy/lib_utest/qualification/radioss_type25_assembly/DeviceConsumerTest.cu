// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25AssemblyDevice.h"
#include "../radioss_type25_friction/CudaFixture.h"
namespace ass = tlfea::contact::radioss_type25::assembly;
using Type25IncidenceConsumer = type25_friction_test::FrictionCuda;
// Links production C++/CUDA and GTest only; no native/oracle library.
TEST_F(Type25IncidenceConsumer, DynamicDeviceCsrHasIndependentProductionLinkClosure) {
  struct Maps { ass::Connectivity row; std::uint32_t end; };
  const Maps host{{{0,1,1,1},0},1};
  type25_friction_test::Drain drain{stream};
  auto* maps=static_cast<Maps*>(input);
  ASSERT_EQ(cudaMemcpyAsync(maps,&host,sizeof(host),cudaMemcpyHostToDevice,stream),cudaSuccess);
  ass::DeviceIncidenceBuilder builder;
  ASSERT_EQ(builder.Initialize({1,2,1,1u<<20},stream),ass::IncidenceStatus::Ok);
  ASSERT_EQ(builder.Stage({&maps->row,{&maps->end,1,1},2,{1,2,3,4,5}}),ass::IncidenceStatus::Ok);
  const auto view=builder.view(); ASSERT_TRUE(builder.IsCurrent(view));
  std::uint32_t offsets[3],ranks[5];
  ASSERT_EQ(cudaMemcpy(offsets,view.incidence().offsets,sizeof(offsets),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(ranks,view.incidence().occurrences,sizeof(ranks),cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(offsets[0],0u); EXPECT_EQ(offsets[1],2u); EXPECT_EQ(offsets[2],5u);
  const std::uint32_t expected[]{0,4,1,2,3};
  for(unsigned i=0;i<5;++i) EXPECT_EQ(ranks[i],expected[i]);
}
