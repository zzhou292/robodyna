// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "CudaFixture.h"
#include <limits>
namespace type25_current_normals_test {
namespace {
using CurrentNormalsCuda=device::CurrentNormalsCuda;
void SameStages(const device::Observation& actual,const NativeResult& expected) {
  ASSERT_EQ(actual.report.status,c::Status::Ok);ASSERT_TRUE(expected.finite);ASSERT_TRUE(actual.values.finite);
  SameNormals(actual.values.flag1_normals,expected.flag1_normals);
  SameNormals(actual.values.normals,expected.normals);SameReferences(actual.values.references,expected.references);
  EXPECT_EQ(actual.values.primary_skip,expected.primary_skip);
}
void Deform(Fixture& f,unsigned step) {
  // Qualification geometry only; the oracle receives identical stored doubles.
  // No transformed expected normal is manufactured here.
  const double angle=.13*step,co=std::cos(angle),si=std::sin(angle);
  for(std::size_t i=0;i<f.mesh.ids.size();++i) {
    const double x=f.mesh.positions[3*i],y=f.mesh.positions[3*i+1],z=f.mesh.positions[3*i+2]+.025*step*x*y;
    f.positions[3*i]=co*x-si*z+.2*step;f.positions[3*i+1]=y+.03*step*x;f.positions[3*i+2]=si*x+co*z;
  }
}
void Advance(Fixture& f,const device::Observation& actual,const NativeResult& expected) {
  // Independent recurrences, including the initial independently native cache.
  f.prior=actual.values.normals;f.native_prior=expected.normals;
}
}
TEST_F(CurrentNormalsCuda, FullOriginalStagesMatchAcrossLaunchOrdersAndCurrentUpdates) {
  RecordProperty("explicit_test_device_buffer_bytes",std::to_string(2*Capacity*RowBytes));
  RecordProperty("typed_input_bytes",std::to_string(sizeof(device::Image)));
  RecordProperty("typed_work_and_output_bytes",std::to_string(sizeof(device::State)));
  for(unsigned mode=0;mode<4;++mode)for(unsigned threads:{1u,7u,32u,64u})for(bool reverse:{false,true}) {
    SCOPED_TRACE(mode);
    SCOPED_TRACE(threads);
    SCOPED_TRACE(reverse);
    Fixture f(mode==3?Cube():Grid(4,4,mode));
    for(unsigned step=0;step<4;++step) {
      SCOPED_TRACE(step);Deform(f,step);
      if(step==0)f.AllActive();
      else if(step==1)f.GeneratedMasks({1});
      else if(step==2)f.GeneratedMasks({},1,{2});
      else f.GeneratedMasks();
      const auto expected=Oracle(f.Input(true));
      const auto actual=EvaluateDevice(f.Input(),threads,reverse,Fixture::Limits());
      SameStages(actual,expected);Advance(f,actual,expected);
    }
  }
}
TEST_F(CurrentNormalsCuda, ClosedInactiveCacheRetainsEveryBitAcrossChangedPositions) {
  Fixture f(Cube());f.GeneratedMasks();ASSERT_TRUE(f.free_ids.empty());
  ASSERT_TRUE(std::all_of(f.main_active.begin(),f.main_active.end(),[](auto x){return x==0;}));
  ASSERT_TRUE(std::all_of(f.node_tag.begin(),f.node_tag.end(),[](auto x){return x==0;}));
  const auto before=f.prior;
  for(unsigned step=1;step<=3;++step) {
    SCOPED_TRACE(step);Deform(f,step);
    const auto expected=Oracle(f.Input(true));
    const auto actual=EvaluateDevice(f.Input(),step==1?1:64,step%2!=0,Fixture::Limits());
    SameStages(actual,expected);SameNormals(actual.values.normals,before);
    EXPECT_TRUE(std::all_of(actual.values.primary_skip.begin(),actual.values.primary_skip.end(),[](int x){return x==1;}));
    for(const auto& ref:actual.values.references){EXPECT_EQ(ref.boundary,0);SameNormal(ref.bisector[0],{});SameNormal(ref.bisector[1],{});}
    Advance(f,actual,expected);
  }
}
TEST_F(CurrentNormalsCuda, TriangleUnusedSlotsRetainSeedBitsUntilOriginalZeroingDefinesThem) {
  for(unsigned threads:{1u,32u,64u})for(bool reverse:{false,true}) {
    Fixture f(Grid(2,2,1));
    for(std::size_t main=0;main<f.built.startup.main_count;++main) {
      const n::StoredNormal seed{float(main+1),-0.f,.125f};f.prior[4*main+2]=f.native_prior[4*main+2]=seed;
    }
    const auto original=f.prior;
    for(unsigned step=0;step<3;++step) {
      SCOPED_TRACE(step);Deform(f,step);f.AllActive();
      if(step==2){std::fill(f.coefficients.begin(),f.coefficients.end(),0);f.RefreshFree();}
      const auto expected=Oracle(f.Input(true));const auto actual=EvaluateDevice(f.Input(),threads,reverse,Fixture::Limits());
      SameStages(actual,expected);
      for(std::size_t main=0;main<f.built.startup.main_count;++main)
        SameNormal(actual.values.normals[4*main+2],step==2?n::StoredNormal{}:original[4*main+2]);
      Advance(f,actual,expected);
    }
  }
}
TEST_F(CurrentNormalsCuda, NativeSiAndFloorDominatedPacketsCrossOriginal129RowCohort) {
  for(bool large:{false,true})for(bool si:{false,true})for(unsigned threads:{7u,64u})for(bool reverse:{false,true}) {
    SCOPED_TRACE(large);
    SCOPED_TRACE(si);
    SCOPED_TRACE(threads);
    SCOPED_TRACE(reverse);
    Fixture f(large?Grid(11,13):Grid(2,2));
    if(large)ASSERT_EQ(f.mesh.primary.size(),143u);
    else for(auto& x:f.positions)x*=1e-13; // Nonzero cross products whose squared float norms underflow.
    if(si){f.mesh.units=s::Coordinates::Si;for(auto& x:f.positions)x*=f.mesh.scale.length_m;}
    f.AllActive();const auto expected=Oracle(f.Input(true));
    const auto actual=EvaluateDevice(f.Input(),threads,reverse,Fixture::Limits());SameStages(actual,expected);
    if(!large) {
      // Native FLAG1 still defines a finite result in the literal ReadyFloor
      // branch. The comparison includes internal faces before FLAG2 averaging.
      EXPECT_TRUE(expected.finite);
      EXPECT_TRUE(std::any_of(expected.flag1_normals.begin(),expected.flag1_normals.end(),[](auto v){return std::abs(v.z)>1.f;}));
    }
  }
}
TEST_F(CurrentNormalsCuda, NativeGeneratedLimitCasePreservesCountsAndZeroBisectors) {
  // Source-generated numerical packet only: the bounded C++ startup factory
  // still rejects this disconnected vertex fan, as its owning host test proves.
  NativeCornerFan fan;const auto in=fan.Input();const auto expected=Oracle(in);
  ASSERT_TRUE(std::any_of(expected.references.begin(),expected.references.end(),[](const auto& r){return r.boundary>2;}));
  for(unsigned threads:{1u,7u,32u,64u})for(bool reverse:{false,true}) {
    SCOPED_TRACE(threads);
    SCOPED_TRACE(reverse);
    const auto actual=EvaluateDevice(in,threads,reverse,Fixture::Limits());SameStages(actual,expected);
    for(const auto& r:actual.values.references)if(r.boundary>2) {
      SameNormal(r.bisector[0],{});SameNormal(r.bisector[1],{});
    }
  }
}
TEST_F(CurrentNormalsCuda, AdmissionAndLateArithmeticFailurePreservePriorPublicationAndRetry) {
  Fixture clean(Grid(2,1));const auto expected=Oracle(clean.Input(true));
  const auto valid=EvaluateDevice(clean.Input(),32,false,Fixture::Limits());SameStages(valid,expected);
  for(unsigned fault=0;fault<7;++fault) {
    SCOPED_TRACE(fault);Fixture bad(Grid(2,1));auto cap=Fixture::Limits();auto in=bad.Input();
    std::vector<n::startup::Main> topology(in.topology.mains,in.topology.mains+in.topology.main_count);
    switch(fault) {
      case 0:cap.nodes=1;break;
      case 1:in.profile=c::Profile::Unspecified;break;
      case 2:bad.coefficients.back()=std::numeric_limits<double>::infinity();break;
      case 3:bad.node_tag.back()=2;break;
      case 4:bad.free_ids.pop_back();in.free_count=bad.free_ids.size();break;
      case 5:topology.back().normal_reference[1]=topology.back().normal_reference[0];in.topology.mains=topology.data();break;
      case 6:for(auto& x:bad.positions)x*=1e25;break;
    }
    const auto host=EvaluateHostNormals(in,cap);EXPECT_NE(host.report.status,c::Status::Ok);
    if(fault==6){EXPECT_EQ(host.report.status,c::Status::NonfiniteResult);EXPECT_FALSE(Oracle(bad.Input(true)).finite);}
    const auto result=EvaluateDevice(in,64,true,cap);SameReport(result.report,host.report);
    EXPECT_TRUE(result.publication_unchanged);SameNormals(result.values.normals,valid.values.normals);
    SameReferences(result.values.references,valid.values.references);
    const auto retry=EvaluateDevice(clean.Input(),7,false,Fixture::Limits());SameStages(retry,expected);
  }
}
} // namespace type25_current_normals_test
