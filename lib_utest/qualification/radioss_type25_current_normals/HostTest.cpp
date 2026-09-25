// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "lib_src/collision/radioss_type25/current_normals/Layout.h"
#include <limits>
#include <cfenv>
namespace type25_current_normals_test {
TEST(CurrentNormals, AllActiveStarterCacheMatchesCompleteOriginalAndFixedReadyStages) {
  for(unsigned mode=0;mode<3;++mode){
    SCOPED_TRACE(mode);auto mesh=Grid(2,2,mode);
    for(std::size_t i=0;i<mesh.ids.size();++i)mesh.positions[3*i+2]=.1*mesh.positions[3*i]*mesh.positions[3*i+1];
    Fixture f(std::move(mesh));const auto expected=Oracle(f.Input(true));const auto actual=EvaluateHostNormals(f.Input());
    Same(actual,expected);SameNormals(actual.normals.data(),f.built.ready.normals.face_normals,actual.normals.size());
    SameReferences(actual.references.data(),f.built.ready.normals.references,actual.references.size());
    EXPECT_TRUE(std::all_of(expected.primary_skip.begin(),expected.primary_skip.end(),[](int x){return x==0;}));
  }
}
TEST(CurrentNormals, IndependentCacheRecurrencesFollowGeneratedSparseMasksAndCurrentGeometry) {
  for(unsigned mode=0;mode<3;++mode){
    SCOPED_TRACE(mode);Fixture f(Grid(4,4,mode));
    for(unsigned step=0;step<6;++step){
      SCOPED_TRACE(step);
      const double angle=.13*step,co=std::cos(angle),si=std::sin(angle);
      for(std::size_t i=0;i<f.mesh.ids.size();++i){const double x=f.mesh.positions[3*i],y=f.mesh.positions[3*i+1],z=.025*step*x*y;
        f.positions[3*i]=co*x-si*z+.2*step;f.positions[3*i+1]=y+.03*step*x;f.positions[3*i+2]=si*x+co*z;}
      if(step==1||step==4)f.GeneratedMasks({6});
      else if(step==2){std::vector<std::uint32_t> removed(f.built.startup.main_count);std::iota(removed.begin(),removed.end(),1);f.GeneratedMasks({},6,removed);}
      else if(step==3)f.GeneratedMasks({},6);
      else f.GeneratedMasks();
      const auto expected=Oracle(f.Input(true));const auto actual=EvaluateHostNormals(f.Input());Same(actual,expected);
      if(step==0)EXPECT_TRUE(std::any_of(expected.primary_skip.begin(),expected.primary_skip.end(),[](int x){return x==1;}));
      // Native and production histories evolve separately: a production error
      // can never seed away its own discrepancy in the next expected update.
      f.native_prior=expected.normals;f.prior=actual.normals;
    }
  }
}
TEST(CurrentNormals, ClosedInactiveInteriorRetainsEveryPriorCacheBitAcrossChangedPositions) {
  Fixture f(Cube());f.GeneratedMasks();ASSERT_TRUE(f.free_ids.empty());
  ASSERT_TRUE(std::all_of(f.main_active.begin(),f.main_active.end(),[](auto x){return x==0;}));
  const auto original=f.prior;
  for(unsigned step=0;step<3;++step){
    for(std::size_t i=0;i<f.mesh.ids.size();++i){f.positions[3*i]+=.31;f.positions[3*i+2]+=.07*double(i);}
    const auto expected=Oracle(f.Input(true));const auto actual=EvaluateHostNormals(f.Input());Same(actual,expected);SameNormals(actual.normals,original);
    for(const auto& ref:actual.references){EXPECT_EQ(ref.boundary,0);SameNormal(ref.bisector[0],{});SameNormal(ref.bisector[1],{});}
    f.native_prior=expected.normals;f.prior=actual.normals;
  }
}
TEST(CurrentNormals, TriangleUnusedSlotsKeepExplicitPriorBitsUntilNativeZeroingDefinesThem) {
  Fixture f(Grid(1,1,1));
  for(std::size_t main=0;main<f.built.startup.main_count;++main){const n::StoredNormal seed{float(main+1),-0.f,.125f};f.prior[4*main+2]=seed;f.native_prior[4*main+2]=seed;}
  auto expected=Oracle(f.Input(true));auto actual=EvaluateHostNormals(f.Input());Same(actual,expected);
  for(std::size_t main=0;main<f.built.startup.main_count;++main)SameNormal(actual.normals[4*main+2],f.prior[4*main+2]);
  std::fill(f.coefficients.begin(),f.coefficients.end(),0);f.RefreshFree();
  expected=Oracle(f.Input(true));actual=EvaluateHostNormals(f.Input());Same(actual,expected);
  for(const auto value:actual.normals)SameNormal(value,{});
}
TEST(CurrentNormals, FreeSlotsCountRawNormalsBeforeZeroDirectionAndRetainZeroSentinel) {
  Fixture f(Grid(1,1));std::fill(f.node_tag.begin(),f.node_tag.end(),0);
  // Supplied cache/masks are an explicit numerical packet. This intentionally
  // isolates NORMP's distinction between raw and transformed zero; it is not a
  // claim that the runtime TAGN stage emits this particular mask combination.
  for(std::size_t m=0;m<f.built.startup.main_count;++m)for(unsigned k=0;k<4;++k){const auto& main=f.built.startup.mains[m];
    const auto a=main.nodes[k],b=main.nodes[(k+1)%4];n::StoredNormal value{float(f.positions[3*b]-f.positions[3*a]),float(f.positions[3*b+1]-f.positions[3*a+1]),0};
    if(m==0&&k==0)value={};f.prior[4*m+k]=f.native_prior[4*m+k]=value;}
  const auto expected=Oracle(f.Input(true));const auto actual=EvaluateHostNormals(f.Input());Same(actual,expected);
  bool zero_slot=false,assigned=false;
  for(const auto& edge:expected.free_edges){zero_slot|=edge[0]==1&&edge[1]==1&&edge[2]==3&&edge[3]==3;assigned|=edge[2]<=2||edge[3]<=2;}
  EXPECT_TRUE(zero_slot);EXPECT_TRUE(assigned);
  for(const auto& ref:actual.references){SameNormal(ref.bisector[0],{});SameNormal(ref.bisector[1],{});}
}
TEST(CurrentNormals, NativeCoordinateBoundaryAndEngine129CohortRetainExactFloatOperations) {
  Fixture f(Grid(11,13));ASSERT_EQ(f.mesh.primary.size(),143u);auto expected=Oracle(f.Input(true));Same(EvaluateHostNormals(f.Input()),expected);
  f.mesh.units=s::Coordinates::Si;
  for(auto& x:f.positions)x*=f.mesh.scale.length_m;
  expected=Oracle(f.Input(true));Same(EvaluateHostNormals(f.Input()),expected);
}
TEST(CurrentNormals, LateArithmeticFailureAndExactCapsNeverPublishPartialCache) {
  Fixture f(Grid(2,1));c::Forecast forecast;ASSERT_EQ(c::Preflight(f.Input(),Fixture::Limits(),forecast).status,c::Status::Ok);
  tl::util::HostArena scratch;ASSERT_TRUE(scratch.Initialize(forecast.scratch_bytes));
  std::vector<n::StoredNormal> normals(f.prior.size(),{3,-0.f,5});std::vector<s::NormalReference> refs(f.built.startup.starter.reference_count);
  for(auto& r:refs){r.boundary=17;r.bisector[0]={9,8,7};r.bisector[1]={6,5,4};}
  const auto old_normals=normals;const auto old_refs=refs;const c::Output output{normals.data(),normals.size(),refs.data(),refs.size()};
  for(auto& x:f.positions)x*=1e25;
  const auto native=Oracle(f.Input(true));EXPECT_FALSE(native.finite);
  EXPECT_EQ(c::Evaluate(f.Input(),Fixture::Limits(),scratch.data(),scratch.bytes(),output).status,c::Status::NonfiniteResult);
  SameNormals(normals,old_normals);SameReferences(refs,old_refs);
  f.positions=f.mesh.positions;
  EXPECT_EQ(c::Evaluate(f.Input(),Fixture::Limits(),scratch.data(),scratch.bytes()-1,output).status,c::Status::ResourceLimit);
  SameNormals(normals,old_normals);SameReferences(refs,old_refs);
  auto limits=Fixture::Limits();limits.scratch_bytes=forecast.scratch_bytes;
  ASSERT_EQ(c::Evaluate(f.Input(),limits,scratch.data(),scratch.bytes(),output).status,c::Status::Ok);
  const auto expected=Oracle(f.Input(true));SameNormals(normals,expected.normals);SameReferences(refs,expected.references);
}
}

namespace type25_current_normals_test {
TEST(CurrentNormals, NativeGeneratedCornerFanRetainsSeparateReferenceGroups) {
  // The selected C++ startup factory deliberately rejects disconnected vertex
  // fans. Original startup splits their reference groups: preserve this actual
  // evidence rather than assert an unsupported LIMIT_CASE premise.
  NativeCornerFan fan;const auto in=fan.Input();
  const auto expected=Oracle(in);Same(EvaluateHostNormals(in),expected);
  bool limited=false;for(const auto& ref:expected.references)if(ref.boundary>2){limited=true;SameNormal(ref.bisector[0],{});SameNormal(ref.bisector[1],{});}
  EXPECT_FALSE(limited);
  const auto forecast=s::Preflight(fan.mesh.ids.size(),fan.mesh.primary.size());tl::util::HostArena output,scratch;
  ASSERT_TRUE(output.Initialize(forecast.output_bytes));
  ASSERT_TRUE(scratch.Initialize(forecast.scratch_bytes));s::Snapshot unused;
  EXPECT_EQ(s::BuildStarter(fan.mesh.Input(),{},output,scratch,&unused).status,s::Status::UnsupportedTopology);
}
TEST(CurrentNormals, MalformedTopologyMaskRosterUnitsAndAliasesPreserveEveryOutput) {
  for(unsigned fault=0;fault<18;++fault){
    SCOPED_TRACE(fault);Fixture f(Grid(2,2));auto in=f.Input();auto limits=Fixture::Limits();c::Forecast plan;
    ASSERT_EQ(c::Preflight(in,limits,plan).status,c::Status::Ok);
    tl::util::HostArena scratch;ASSERT_TRUE(scratch.Initialize(plan.scratch_bytes));
    std::vector<s::Main> mains(in.topology.mains,in.topology.mains+in.topology.main_count);in.topology.mains=mains.data();
    std::vector<std::uint32_t> entries(in.topology.normal_to_main.entries,in.topology.normal_to_main.entries+in.topology.normal_to_main.entry_count);in.topology.normal_to_main.entries=entries.data();
    std::vector<n::StoredNormal> normals(f.prior.size(),{3,-0.f,5});std::vector<s::NormalReference> refs(in.topology.references);
    for(auto& r:refs){r.boundary=17;r.bisector[0]={9,8,7};r.bisector[1]={6,5,4};}
    const auto old_normals=normals;const auto old_refs=refs;const auto old_prior=f.prior;
    c::Output out{normals.data(),normals.size(),refs.data(),refs.size()};void* work=scratch.data();
    if(fault==0)in.profile=c::Profile::Unspecified;
    if(fault==1)--in.tag_count;
    if(fault==2)f.node_tag.back()=2;
    if(fault==3)f.main_active.back()=2;
    if(fault==4)--in.free_count;
    if(fault==5)f.prior.back().x=std::numeric_limits<float>::infinity();
    if(fault==6)f.positions.back()=std::numeric_limits<double>::quiet_NaN();
    if(fault==7){const auto& csr=in.topology.normal_to_main;bool changed=false;
      for(std::size_t r=0;r<in.topology.references&&!changed;++r)if(csr.offsets[r+1]-csr.offsets[r]>1){entries[csr.offsets[r]+1]=entries[csr.offsets[r]];changed=true;}ASSERT_TRUE(changed);}
    if(fault==8)mains[0].normal_reference[1]=mains[0].normal_reference[0];
    if(fault==9)mains[in.topology.primary_count].nodes[0]=mains[0].nodes[0];
    if(fault==10)mains[0].segment_type=1;
    if(fault==11)mains[0].neighbor_edges[0]=5;
    if(fault==12)out.face_normals=f.prior.data();
    if(fault==13)work=f.prior.data();
    if(fault==14)out.references=reinterpret_cast<s::NormalReference*>(normals.data());
    if(fault==15){in.coordinates=s::Coordinates::Si;in.units.length_m=0;}
    if(fault==16)limits.primaries=in.topology.primary_count-1;
    if(fault==17)f.coefficients[f.free_ids.back()-1]=0;
    EXPECT_NE(c::Evaluate(in,limits,work,scratch.bytes(),out).status,c::Status::Ok);
    SameNormals(normals,old_normals);SameReferences(refs,old_refs);
    if(fault!=5)SameNormals(f.prior,old_prior);
  }
}
TEST(CurrentNormals, ForecastFailureCannotOverwriteItsInputDescriptor) {
  Fixture f(Grid(1,1));auto in=f.Input();std::array<unsigned char,sizeof(in)> bytes;std::memcpy(bytes.data(),&in,sizeof in);
  auto* alias=reinterpret_cast<c::Forecast*>(&in);
  EXPECT_EQ(c::Preflight(in,Fixture::Limits(),*alias).status,c::Status::InvalidInput);
  EXPECT_EQ(std::memcmp(&in,bytes.data(),sizeof in),0);
  c::Forecast out{19,23};auto limits=Fixture::Limits();limits.scratch_bytes=1;
  EXPECT_EQ(c::Preflight(in,limits,out).status,c::Status::ResourceLimit);
  EXPECT_EQ(out.scratch_bytes,19u);EXPECT_EQ(out.output_bytes,23u);
}
}

namespace type25_current_normals_test {
TEST(CurrentNormals, HostArithmeticEnvironmentRejectsAndRestoresBeforeRetry) {
  Fixture f(Grid(1,1));c::Forecast forecast;ASSERT_EQ(c::Preflight(f.Input(),Fixture::Limits(),forecast).status,c::Status::Ok);
  tl::util::HostArena scratch;ASSERT_TRUE(scratch.Initialize(forecast.scratch_bytes));
  std::vector<n::StoredNormal> normals(f.prior.size(),{3,-0.f,5});std::vector<s::NormalReference> refs(f.built.startup.starter.reference_count);
  const auto before=normals;const auto old=refs;c::Forecast preserved{19,23};
  const int mode=std::fegetround();ASSERT_EQ(std::fesetround(FE_UPWARD),0);
  const auto preflight=c::Preflight(f.Input(),Fixture::Limits(),preserved);
  const auto report=c::Evaluate(f.Input(),Fixture::Limits(),scratch.data(),scratch.bytes(),{normals.data(),normals.size(),refs.data(),refs.size()});
  const int restored=std::fesetround(mode);
  EXPECT_EQ(restored,0);EXPECT_EQ(preflight.status,c::Status::UnsupportedArithmetic);EXPECT_EQ(report.status,c::Status::UnsupportedArithmetic);
  EXPECT_EQ(preserved.scratch_bytes,19u);EXPECT_EQ(preserved.output_bytes,23u);SameNormals(normals,before);SameReferences(refs,old);
  Same(EvaluateHostNormals(f.Input()),Oracle(f.Input(true)));
}
}

namespace type25_current_normals_test {
TEST(CurrentNormals, ExplicitChangedTopologyPacketExercisesNativeLimitCaseWithoutRefreshClaim) {
  NativeOpenEdges packet;const auto in=packet.Input();const auto expected=Oracle(in);const auto actual=EvaluateHostNormals(in);Same(actual,expected);
  bool limited=false;for(const auto& ref:expected.references)if(ref.boundary>2){limited=true;SameNormal(ref.bisector[0],{});SameNormal(ref.bisector[1],{});}
  ASSERT_TRUE(limited);
  // The original native topology is retained as provenance, not overwritten.
  bool originally_linked=false;for(const auto& main:packet.native_start.mains)for(int other:main.neighbors)originally_linked|=other>0;
  EXPECT_TRUE(originally_linked);
}
}
