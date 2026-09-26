#include "Fixture.h"
#include "../radioss_type25_fixed_main_startup/Fixture.h"
#include <cfenv>
#include <limits>
namespace type25_interface_surface_test {
TEST(Type25InterfaceHost, ExactCapsAndDescriptorFirstFailurePreservePublishedInterface) {
  auto c=Mixed();Built b(c);ASSERT_EQ(b.report.status,f::Status::Ok);
  const auto bytes=Bytes(b.output);const auto prior=b.result;
  f::Limits cap;cap.output_bytes=b.forecast.output_bytes-1;
  EXPECT_EQ(f::Build(c.Input(),cap,b.output,b.scratch,&b.result).status,f::Status::ResourceLimit);
  EXPECT_EQ(Bytes(b.output),bytes);EXPECT_EQ(b.result.primary,prior.primary);
  cap={};cap.scratch_bytes=b.forecast.scratch_bytes-1;
  EXPECT_EQ(f::Build(c.Input(),cap,b.output,b.scratch,&b.result).status,f::Status::ResourceLimit);
  EXPECT_EQ(Bytes(b.output),bytes);
  cap={};cap.output_bytes=b.forecast.output_bytes;cap.scratch_bytes=b.forecast.scratch_bytes;
  EXPECT_EQ(f::Build(c.Input(),cap,b.output,b.scratch,&b.result).status,f::Status::Ok);
  auto in=c.Input();in.raw_face_count=SIZE_MAX;in.raw_faces=reinterpret_cast<const old::s::Face*>(1);
  f::Forecast unchanged=b.forecast;
  EXPECT_EQ(f::Preflight(in,{},unchanged).status,f::Status::ResourceLimit);
  EXPECT_EQ(unchanged.output_bytes,b.forecast.output_bytes);
  EXPECT_EQ(unchanged.scratch_bytes,b.forecast.scratch_bytes);
}
TEST(Type25InterfaceHost, LateCoordinatesAndForeignRawIdentityLeaveOutputUnchanged) {
  auto c=Mixed();Built b(c);ASSERT_EQ(b.report.status,f::Status::Ok);
  const auto bytes=Bytes(b.output);const auto prior=b.result;
  c.points.back().x=std::numeric_limits<double>::infinity();
  EXPECT_EQ(f::Build(c.Input(),{},b.output,b.scratch,&b.result).status,f::Status::InvalidInput);
  EXPECT_EQ(Bytes(b.output),bytes);EXPECT_EQ(b.result.raw_to_primary,prior.raw_to_primary);
  c=Mixed();++c.raw.back().source.element_id;
  EXPECT_EQ(f::Build(c.Input(),{},b.output,b.scratch,&b.result).status,f::Status::InvalidInput);
  EXPECT_EQ(Bytes(b.output),bytes);
  c=Mixed();c.raw.back().raw_role=0;
  EXPECT_NE(f::Build(c.Input(),{},b.output,b.scratch,&b.result).status,f::Status::Ok);
  EXPECT_EQ(Bytes(b.output),bytes);
  c=Mixed();
  for(auto& point:c.points)point={1.e308,1.e308,1.e308};
  EXPECT_EQ(f::Build(c.Input(),{},b.output,b.scratch,&b.result).status,f::Status::NonfiniteResult);
  EXPECT_EQ(Bytes(b.output),bytes);
  c=Mixed();EXPECT_EQ(f::Build(c.Input(),{},b.output,b.scratch,&b.result).status,f::Status::Ok);
}
TEST(Type25InterfaceHost, ForecastInputAndBorrowedOutputAliasesRejectBeforeWrites) {
  auto c=Mixed();Built b(c);ASSERT_EQ(b.report.status,f::Status::Ok);
  const auto bytes=Bytes(b.output);
  auto in=c.Input();
  const auto original_generation=in.source_generation;
  EXPECT_EQ(f::Preflight(in,{},*reinterpret_cast<f::Forecast*>(&in)).status,f::Status::InvalidInput);
  EXPECT_EQ(in.source_generation,original_generation);
  in=c.Input();in.raw_faces=reinterpret_cast<const old::s::Face*>(b.output.data());
  EXPECT_EQ(f::Build(in,{},b.output,b.scratch,&b.result).status,f::Status::InvalidInput);
  EXPECT_EQ(Bytes(b.output),bytes);
  EXPECT_EQ(f::Build(c.Input(),{},b.output,b.scratch,reinterpret_cast<f::Snapshot*>(b.output.data())).status,
      f::Status::InvalidInput);
  EXPECT_EQ(Bytes(b.output),bytes);
  ASSERT_GE(b.scratch.bytes(),b.forecast.output_bytes);
  EXPECT_EQ(f::Build(c.Input(),{},b.scratch,b.scratch,&b.result).status,f::Status::InvalidInput);
  EXPECT_EQ(Bytes(b.output),bytes);
}
TEST(Type25InterfaceHost, OriginDiscriminatorsCountsAndPartnerTotalsAreCheckedAtomically) {
  auto c=Origins();Built b(c);ASSERT_EQ(b.report.status,f::Status::Ok);
  const auto valid=SideInput(c,b.result);Sides sides(valid);
  ASSERT_EQ(sides.report.status,s::Status::Ok);
  const auto bytes=Bytes(sides.output);const auto prior=sides.result;
  std::vector<s::PrimaryFaceIdentity> identities(b.result.identities,b.result.identities+b.result.primary_count);
  auto in=valid;in.primary_identities=identities.data();
  std::size_t group=0;
  while(group<identities.size()&&identities[group].origin!=s::PrimaryOrigin::MultipleOrigins)++group;
  ASSERT_LT(group,identities.size());
  for(unsigned mutation=0;mutation<4;++mutation) {
    SCOPED_TRACE(mutation);
    identities.assign(b.result.identities,b.result.identities+b.result.primary_count);
    in.primary_identities=identities.data();
    if(mutation==0)identities[group].physical_parent_id=100;
    if(mutation==1)identities[group].origin_count=1;
    if(mutation==2)identities[group].origin=s::PrimaryOrigin::SingleSourceFace;
    if(mutation==3)identities[group].local_face=1;
    EXPECT_EQ(s::BuildMixedSides(in,{},sides.output,sides.scratch,&sides.result).status,s::Status::InvalidInput);
    EXPECT_EQ(Bytes(sides.output),bytes);EXPECT_EQ(sides.result.mains,prior.mains);
  }
  auto mixed=Mixed();Built mixed_values(mixed);ASSERT_EQ(mixed_values.report.status,f::Status::Ok);
  auto wrong=SideInput(mixed,mixed_values.result);Sides complete(wrong);ASSERT_EQ(complete.report.status,s::Status::Ok);
  const auto complete_bytes=Bytes(complete.output);
  --wrong.shell_primary_count; // Smaller storage still fits; reaches exact count validation.
  EXPECT_EQ(s::BuildMixedSides(wrong,{},complete.output,complete.scratch,&complete.result).status,s::Status::InvalidInput);
  EXPECT_EQ(Bytes(complete.output),complete_bytes);
  EXPECT_EQ(s::BuildMixedSides(valid,{},sides.output,sides.scratch,&sides.result).status,s::Status::Ok);
}
TEST(Type25InterfaceHost, CompleteRawMappingAndMixedSideAliasesArePreservedOnFailure) {
  auto c=Origins();Built b(c);ASSERT_EQ(b.report.status,f::Status::Ok);
  const auto valid=SideInput(c,b.result);Sides sides(valid);ASSERT_EQ(sides.report.status,s::Status::Ok);
  const auto bytes=Bytes(sides.output);
  std::vector<std::uint32_t> map(valid.raw_origin_to_primary,valid.raw_origin_to_primary+valid.raw_origin_count);
  map.back()=std::uint32_t(valid.primary_count);
  auto in=valid;in.raw_origin_to_primary=map.data();
  EXPECT_EQ(s::BuildMixedSides(in,{},sides.output,sides.scratch,&sides.result).status,s::Status::InvalidInput);
  EXPECT_EQ(Bytes(sides.output),bytes);
  in=valid;in.raw_origins=reinterpret_cast<const s::PrimaryFaceIdentity*>(sides.output.data());
  EXPECT_EQ(s::BuildMixedSides(in,{},sides.output,sides.scratch,&sides.result).status,s::Status::InvalidInput);
  EXPECT_EQ(Bytes(sides.output),bytes);
  s::Limits cap;cap.max_output_bytes=sides.forecast.output_bytes-1;
  EXPECT_EQ(s::BuildMixedSides(valid,cap,sides.output,sides.scratch,&sides.result).status,s::Status::ResourceLimit);
  EXPECT_EQ(Bytes(sides.output),bytes);
}
TEST(Type25InterfaceHost, MissingPostGapSupportNeverProducesMixedNormalsOrAFalseReadyView) {
  auto c=Mixed();Built b(c);ASSERT_EQ(b.report.status,f::Status::Ok);
  const auto in=SideInput(c,b.result);Sides sides(in);ASSERT_EQ(sides.report.status,s::Status::Ok);
  s::Snapshot unavailable;
  EXPECT_EQ(s::Preflight(in).status,s::Status::UnsupportedProfile);
  EXPECT_EQ(s::BuildStarter(in,{},sides.output,sides.scratch,&unavailable).status,s::Status::UnsupportedProfile);
  EXPECT_EQ(unavailable.mains,nullptr);
  EXPECT_EQ(sides.forecast.maximum_references,0u);
  EXPECT_EQ(sides.forecast.maximum_incidence,0u);
}
TEST(Type25InterfaceHost, LegacyPhysicalIdentityAndArenaForecastRemainSeparate) {
  static_assert(sizeof(s::PrimaryFace)==32);
  static_assert(sizeof(s::Main)==80);
  auto legacy=type25_startup_test::Grid(1,1);
  type25_startup_test::Built built(legacy);
  const auto count=s::Preflight(legacy.ids.size(),legacy.primary.size());
  const auto typed=s::Preflight(legacy.Input());
  EXPECT_EQ(count.output_bytes,typed.output_bytes);EXPECT_EQ(count.scratch_bytes,typed.scratch_bytes);
  EXPECT_EQ(count.ready_output_bytes,typed.ready_output_bytes);
  EXPECT_EQ(built.startup.primary_identities,nullptr);EXPECT_EQ(built.startup.raw_origin_count,0u);
  const auto bytes=Bytes(built.output);
  legacy.primary[0].source_id=0;
  EXPECT_EQ(s::BuildStarter(legacy.Input(),{},built.output,built.scratch,&built.startup).status,s::Status::InvalidInput);
  EXPECT_EQ(Bytes(built.output),bytes);
}
TEST(Type25InterfaceHost, HostRoundingIsCheckedAndRetryRemainsValid) {
  auto c=Mixed();Built b(c);ASSERT_EQ(b.report.status,f::Status::Ok);const auto bytes=Bytes(b.output);
  const auto old_rounding=std::fegetround();ASSERT_EQ(std::fesetround(FE_UPWARD),0);
  const auto report=f::Build(c.Input(),{},b.output,b.scratch,&b.result);
  std::fesetround(old_rounding);
  EXPECT_EQ(report.status,f::Status::UnsupportedArithmetic);EXPECT_EQ(Bytes(b.output),bytes);
  EXPECT_EQ(f::Build(c.Input(),{},b.output,b.scratch,&b.result).status,f::Status::Ok);
}
}
