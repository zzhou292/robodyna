// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <limits>
namespace type25_seed_test {
TEST(NodalSeed, EmptyChannelsInitializeEveryDeclaredNodeAndExactCaps) {
  seed::Input input; input.node_count=7;
  SeedFixture fixture;
  ASSERT_EQ(fixture.Build(input).status,seed::Status::Ok);
  for(const auto& node:fixture.nodes)SameSeed(node,{});
  EXPECT_EQ(fixture.forecast.output_bytes,7*sizeof(n::NativeNodalSeed));
  auto limits=seed::Limits{}; limits.scratch_bytes=fixture.forecast.scratch_bytes;
  seed::Forecast forecast;
  EXPECT_EQ(seed::Preflight(input,limits,forecast).status,seed::Status::Ok);
  --limits.scratch_bytes;
  EXPECT_EQ(seed::Preflight(input,limits,forecast).status,seed::Status::ResourceLimit);
  input.volumes=reinterpret_cast<const n::NativeVolumeOccurrence*>(1);
  input.volume_count=seed::Limits{}.volume_occurrences+1;
  EXPECT_EQ(seed::Preflight(input,{},forecast).status,seed::Status::ResourceLimit);
}
TEST(NodalSeed, LateInvalidRowsAndArithmeticOverflowNeverPublishThenRetry) {
  std::vector<n::NativeVolumeOccurrence> volume{{0,2,6},{1,3,12},{0,5,15}};
  std::vector<n::NativeStiffnessOccurrence> direct{{0,7},{1,9},{0,11}};
  seed::Input input{3,volume.data(),volume.size(),direct.data(),direct.size()};
  SeedFixture fixture;ASSERT_EQ(fixture.Build(input).status,seed::Status::Ok);
  const auto prior=fixture.nodes;
  const auto unchanged=[&]{for(std::size_t i=0;i<prior.size();++i)SameSeed(fixture.nodes[i],prior[i]);};
  auto execute=[&]{return seed::Accumulate(input,{},fixture.scratch.data(),fixture.forecast.scratch_bytes,
      {fixture.nodes.data(),fixture.nodes.size()});};
  direct.back().node=3;
  auto bad=execute();EXPECT_EQ(bad.status,seed::Status::InvalidInput);EXPECT_EQ(bad.occurrence,2u);unchanged();
  direct.back().node=0;
  volume.back().bulk_volume=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(execute().status,seed::Status::InvalidInput);unchanged();
  volume.back().bulk_volume=15;
  volume[0].volume=volume[2].volume=std::numeric_limits<double>::max();
  bad=execute();EXPECT_EQ(bad.status,seed::Status::NonfiniteResult);EXPECT_EQ(bad.channel,seed::Channel::Volume);
  EXPECT_EQ(bad.occurrence,2u);unchanged();
  volume[0].volume=2;volume[2].volume=5;
  direct[0].stiffness=direct[2].stiffness=std::numeric_limits<double>::max();
  bad=execute();EXPECT_EQ(bad.status,seed::Status::NonfiniteResult);EXPECT_EQ(bad.channel,seed::Channel::Stiffness);unchanged();
  direct[0].stiffness=7;direct[2].stiffness=11;
  EXPECT_EQ(execute().status,seed::Status::Ok);unchanged();
}
TEST(NodalSeed, AliasedAndShortStorageRejectBeforeMutation) {
  std::vector<n::NativeVolumeOccurrence> volume{{0,2,3},{1,5,7},{2,11,13}};
  seed::Input input{3,volume.data(),volume.size(),nullptr,0};
  SeedFixture fixture;ASSERT_EQ(fixture.Build(input).status,seed::Status::Ok);
  const auto prior=fixture.nodes;
  const auto out=seed::Output{fixture.nodes.data(),fixture.nodes.size()};
  EXPECT_EQ(seed::Accumulate(input,{},fixture.scratch.data(),fixture.forecast.scratch_bytes-1,out).status,
      seed::Status::InvalidInput);
  // Legal void scratch aliases actual live read/input or output storage; no
  // unrelated typed object is read through a cast in these rejected requests.
  ASSERT_EQ(reinterpret_cast<std::uintptr_t>(volume.data())%alignof(std::max_align_t),0u);
  EXPECT_EQ(seed::Accumulate(input,{},volume.data(),volume.size()*sizeof(volume[0]),out).status,
      seed::Status::InvalidInput);
  EXPECT_EQ(seed::Accumulate(input,{},fixture.nodes.data(),fixture.forecast.output_bytes,out).status,
      seed::Status::InvalidInput);
  for(std::size_t i=0;i<prior.size();++i)SameSeed(fixture.nodes[i],prior[i]);
  EXPECT_EQ(volume[0].volume,2);
}
TEST(ShellSeed, LegacyAndExplicitZeroSeedKeepAllFieldBitsAndForecast) {
  const auto input=MixedCase();const auto rows=PreparePort(input);
  ShellFixture old(rows,input.nodes,shell::Population::OrdinaryShellsOnly);
  old.SelectMain(1); // Ordinary quad; its incident triangle stays noncontact.
  shell::Forecast old_forecast;
  ASSERT_EQ(shell::Preflight(old.input,{},old_forecast).status,shell::Status::Ok);
  ASSERT_TRUE(old.scratch.Initialize(old_forecast.scratch_bytes));
  ASSERT_EQ(shell::Build(old.input,{},old.scratch.data(),old_forecast.scratch_bytes,old.Output()).status,shell::Status::Ok);
  std::vector<n::NativeNodalSeed> zeros(input.nodes);
  ShellFixture seeded(rows,input.nodes,shell::Population::PhysicalShellsWithNodalSeed);
  seeded.SelectMain(1);
  ASSERT_EQ(seeded.Build({zeros.data(),zeros.size()}).status,shell::Status::Ok);
  ASSERT_EQ(old.main_values.size(),1u);
  EXPECT_TRUE(tl::math::SameScalarBits(old.main_values[0],seeded.main_values[0]));
  EXPECT_TRUE(seeded.nodes[0].on_main_surface);
  EXPECT_FALSE(seeded.nodes[14].on_main_surface);
  EXPECT_EQ(seeded.nodes[0].shell_incidence_count,2);
  EXPECT_EQ(old_forecast.output_bytes,seeded.forecast.output_bytes);
  EXPECT_EQ(old_forecast.scratch_bytes,seeded.forecast.scratch_bytes);
  for(std::size_t i=0;i<input.nodes;++i) {
    const auto& a=old.nodes[i];const auto& b=seeded.nodes[i];
    EXPECT_TRUE(tl::math::SameScalarBits(a.young_thickness_sum,b.young_thickness_sum));
    EXPECT_TRUE(tl::math::SameScalarBits(a.stiffness,b.stiffness));
    EXPECT_TRUE(tl::math::SameScalarBits(a.unscaled_half_gap,b.unscaled_half_gap));
    EXPECT_TRUE(tl::math::SameScalarBits(a.main_gap,b.main_gap));
    EXPECT_EQ(a.shell_incidence_count,b.shell_incidence_count);EXPECT_EQ(a.on_main_surface,b.on_main_surface);
    EXPECT_TRUE(tl::math::SameScalarBits(old.values[i].stiffness,seeded.values[i].stiffness));
    EXPECT_TRUE(tl::math::SameScalarBits(old.values[i].gap,seeded.values[i].gap));
  }
}
TEST(ShellSeed, MissingMismatchedNegativeAndAliasedSeedsRejectWithoutPublishing) {
  const auto input=MixedCase();const auto rows=PreparePort(input);
  std::vector<n::NativeNodalSeed> seeds(input.nodes,{1,3,5});
  ShellFixture fixture(rows,input.nodes,shell::Population::PhysicalShellsWithNodalSeed);
  ASSERT_EQ(fixture.Build({seeds.data(),seeds.size()}).status,shell::Status::Ok);
  const auto prior=fixture.nodes;const auto secondary=fixture.values;
  auto execute=[&](n::NativeNodalSeedView view,void* scratch,std::size_t bytes){
    return shell::Build(fixture.input,view,{},scratch,bytes,fixture.Output());};
  EXPECT_EQ(execute({},fixture.scratch.data(),fixture.forecast.scratch_bytes).status,shell::Status::InvalidInput);
  EXPECT_EQ(execute({seeds.data(),seeds.size()-1},fixture.scratch.data(),fixture.forecast.scratch_bytes).status,shell::Status::InvalidInput);
  seeds.back().volume=-1;
  EXPECT_EQ(execute({seeds.data(),seeds.size()},fixture.scratch.data(),fixture.forecast.scratch_bytes).status,shell::Status::InvalidInput);
  seeds.back().volume=1;
  // Extend a real live seed allocation so it can legally be the whole proposed
  // scratch span; the new seed read range must reject its overlap before zeroing.
  std::vector<n::NativeNodalSeed> alias((fixture.forecast.scratch_bytes+sizeof(n::NativeNodalSeed)-1)/sizeof(n::NativeNodalSeed),{1,3,5});
  ASSERT_EQ(reinterpret_cast<std::uintptr_t>(alias.data())%alignof(std::max_align_t),0u);
  EXPECT_EQ(execute({alias.data(),input.nodes},alias.data(),alias.size()*sizeof(alias[0])).status,shell::Status::InvalidInput);
  EXPECT_EQ(alias.front().existing_stiffness,5);
  for(std::size_t i=0;i<input.nodes;++i) {
    EXPECT_TRUE(tl::math::SameScalarBits(fixture.nodes[i].stiffness,prior[i].stiffness));
    EXPECT_TRUE(tl::math::SameScalarBits(fixture.values[i].stiffness,secondary[i].stiffness));
  }
  fixture.input.profile.population=shell::Population::OrdinaryShellsOnly;
  EXPECT_EQ(execute({seeds.data(),seeds.size()},fixture.scratch.data(),fixture.forecast.scratch_bytes).status,shell::Status::InvalidInput);
  fixture.input.profile.population=shell::Population::PhysicalShellsWithNodalSeed;
  seeds.back().volume=1e-30;seeds.back().bulk_volume=std::numeric_limits<double>::max();
  EXPECT_EQ(execute({seeds.data(),seeds.size()},fixture.scratch.data(),fixture.forecast.scratch_bytes).status,shell::Status::NonfiniteResult);
  EXPECT_TRUE(tl::math::SameScalarBits(fixture.nodes.back().stiffness,prior.back().stiffness));
  seeds.back()={1,3,5};EXPECT_EQ(execute({seeds.data(),seeds.size()},fixture.scratch.data(),fixture.forecast.scratch_bytes).status,shell::Status::Ok);
}
} // namespace type25_seed_test
