// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Access.h"
#include "Fixture.h"
#include "MixedFixture.h"
#include "CapturePairs.h"
#include "../radioss_type25_initial_state/Assertions.h"
#include "../radioss_type25_friction/CudaFixture.h"
#include <cstring>
#include <limits>
namespace initial_source_test {
using Access=n::qualification::InitialSourceAccess;
namespace {
std::uint64_t Bits(double x){std::uint64_t out;std::memcpy(&out,&x,sizeof(out));return out;}
struct Expected {
  type25_search_startup_test::NativeResult geometric;
  InventoryResult inventory;
  initial_state_test::NativeInitialHistory before_tied;
  std::vector<tied::History> rows;
  type25_tied_removal_test::NativeResult final;
  std::vector<std::uint64_t> source_ids;
  std::uint64_t generation=0;
};
Expected Oracle(const Fixture& f,int sharp=1) {
  Expected out;out.geometric=type25_search_startup_test::Oracle(f.Search());
  out.generation=f.Search().mesh.source_generation;
  for(const auto& row:f.secondary)out.source_ids.push_back(f.nodes[row.node].source_id);
  out.inventory=NativeInventory(f.Inventory(out.geometric));
  out.before_tied=FullInitialHistory(f.Search(),f.native,out.inventory.corner_gaps,out.inventory.pairs,sharp);
  out.rows=out.before_tied.rows;
  if(!f.ties.empty()) {
    const auto source=f.Search();const auto& g=out.geometric;
    search::Snapshot view;view.multiplier=g.scalar[0];view.mean_length=g.scalar[1];view.margin=g.scalar[2];view.maximum_extent=g.scalar[3];
    view.primary_extent=g.extent.data();view.primary_count=g.extent.size();view.main_offsets=g.main_offsets.data();
    view.removed_nodes=g.removed_nodes.empty()?nullptr:g.removed_nodes.data();view.secondary_offsets=g.secondary_offsets.data();
    view.removed_mains=g.removed_mains.empty()?nullptr:g.removed_mains.data();view.initial_contact=g.contact.data();
    view.main_count=f.mains.size();view.secondary_count=f.secondary.size();view.removal_count=g.removed_nodes.size();
    view.source_generation=source.mesh.source_generation;view.native_model_nodes=f.nodes.size();
    tied::Input input;input.source=source;input.geometric={view,source.contributors};
    input.finalization=tied::Finalization::CompactedAfterKinChk;input.tied_removal=1;
    input.interfaces=f.ties.data();input.interface_count=f.ties.size();input.history=out.rows.data();input.history_count=out.rows.size();
    input.native_removal_extent=std::max(std::size_t{16}*f.mains.size(),view.removal_count);
    out.final=type25_tied_removal_test::Oracle(input);out.rows=out.final.history;
  }
  std::vector<std::array<int,4>> keys(out.rows.size());
  for(std::size_t row=0;row<keys.size();++row)std::copy_n(out.rows[row].irtlm,4,keys[row].data());
  NativePreparedMain(f.mains.size(),keys);
  for(std::size_t row=0;row<keys.size();++row)std::copy_n(keys[row].data(),4,out.rows[row].irtlm);
  return out;
}
st::Case VoxelPatchMesh(double separation) {
  st::Case mesh;
  const auto node=[&](double x,double y,double z) {
    mesh.ids.push_back(mesh.ids.size()+1);mesh.positions.insert(mesh.positions.end(),{x,y,z});
  };
  // Independent real Q4 patches occupy distinct y/z bands while sharing the
  // same x interval. Dense variant overlaps their search boxes. Each carries
  // near-face secondary points on both sides; no expected pair is prescribed.
  for(unsigned z=0;z<4;++z)for(unsigned y=0;y<4;++y) {
    const auto first=std::uint32_t(mesh.ids.size());const double a=separation*y,b=separation*z;
    node(0,a,b);node(1,a,b);node(1,a+1,b);node(0,a+1,b);
    mesh.Add(n::ShellLayout::Quad4,first,first+1,first+2,first+3);
    for(unsigned j=0;j<3;++j)node(.2+.08*j,a+.4,b+(j%2?.025:-.025));
  }
  // These genuine unused source nodes extend the global box and exercise
  // secondary cells at the inclusive upper padded y/z boundary.
  node(.4,3*separation+5,3*separation+5);node(.4,-5,-5);
  return mesh;
}
std::array<double,5> Penetration(const n::NativeContactRow& row) {
  return {row.history.normal.staged_penetration,row.history.normal.previous_penetration,
      row.history.normal.damping_half_force,row.penetration_auxiliary,row.penetration_offset};
}
void Same(const Access::Result& actual,const Expected& expected) {
  ASSERT_EQ(actual.history.size(),expected.rows.size());ASSERT_EQ(actual.flags.size(),expected.rows.size());
  ASSERT_EQ(actual.history.size(),expected.source_ids.size());
  ASSERT_EQ(actual.gaps.size(),4*expected.inventory.corner_gaps.size());
  for(std::size_t row=0;row<actual.history.size();++row) {
    SCOPED_TRACE(row);
    const auto& a=actual.history[row].row;const auto& b=expected.rows[row];
    EXPECT_EQ(actual.history[row].secondary_source_id,expected.source_ids[row]);
    EXPECT_EQ(actual.history[row].generation,expected.generation);
    for(unsigned k=0;k<4;++k)EXPECT_EQ(a.irtlm[k],b.irtlm[k]);
    const auto values=Penetration(a);
    for(unsigned k=0;k<5;++k)EXPECT_EQ(Bits(values[k]),Bits(b.penetration[k]));
    // Raw Starter TIME_S is not copied: qualified Begin overwrites both metrics
    // before selected first use. Expected rows keep those original raw fields.
    EXPECT_EQ(actual.flags[row],0);
    EXPECT_EQ(Bits(a.history.normal.previous_stiffness),Bits(0.));
    EXPECT_EQ(Bits(a.history.normal.staged_stiffness),Bits(0.));
    for(double force:{a.history.previous_force.x,a.history.previous_force.y,a.history.previous_force.z,
        a.history.staged_force.x,a.history.staged_force.y,a.history.staged_force.z})EXPECT_EQ(Bits(force),Bits(0.));
  }
  for(std::size_t m=0;m<expected.inventory.corner_gaps.size();++m)for(unsigned k=0;k<4;++k)
    EXPECT_EQ(Bits(actual.gaps[4*m+k]),Bits(expected.inventory.corner_gaps[m][k]));
}
class InitialSourceCuda:public type25_friction_test::PacketCuda<> {};
}
TEST_F(InitialSourceCuda, GenuineSourceWholeNativeInventoryAndWarmRowsMatch) {
  std::uint64_t warm=0;
  for(unsigned mode:{0u,1u,2u})for(bool warped:{false,true})for(int sharp:{1,2}) {
    SCOPED_TRACE(mode);
    SCOPED_TRACE(warped);
    SCOPED_TRACE(sharp);
    Fixture f(mode,warped);auto in=f.Input();in.controls.sharp=sharp;
    const auto expected=Oracle(f,sharp);src::PreparedSource host;
    ASSERT_EQ(src::PrepareSource(in,f.Limits(),host).status,src::Status::Ok);
    EXPECT_EQ(Bits(host.removals().engine_margin),Bits(expected.inventory.engine_margin));
    src::DeviceSeed seed;const auto result=src::Prepare(host,stream,seed);
    ASSERT_EQ(result.status,src::Status::Ok);ASSERT_TRUE(result.counts_complete);
    EXPECT_EQ(result.diagnostics.pairs,expected.inventory.pairs.size());warm+=result.diagnostics.warm_after_tied;
    Same(Access::Read(seed,stream),expected);
  }
  EXPECT_GT(warm,0u);RecordProperty("whole_native_candidate_and_consumed_history_bits",true);
}
TEST_F(InitialSourceCuda, GenuineTiedAugmentationPreservesPrefixAndResetsOnlyActualWarmRelations) {
  Fixture f(0,false,true);const auto expected=Oracle(f);src::PreparedSource host;
  ASSERT_EQ(src::PrepareSource(f.Input(),f.Limits(),host).status,src::Status::Ok);
  const auto view=host.removals();ASSERT_EQ(view.used_count,expected.final.nodes.size());
  for(std::size_t m=0;m<=f.mains.size();++m)EXPECT_EQ(view.main_offsets[m],expected.final.main_offsets[m]);
  for(std::size_t k=0;k<view.used_count;++k){EXPECT_EQ(view.removed_nodes[k],expected.final.nodes[k]);EXPECT_EQ(view.by_secondary.entries[k],expected.final.mains[k]);}
  ASSERT_GT(view.added_by_tied,0u);
  src::DeviceSeed seed;const auto result=src::Prepare(host,stream,seed);ASSERT_EQ(result.status,src::Status::Ok);
  EXPECT_EQ(result.diagnostics.pairs,expected.inventory.pairs.size());
  Same(Access::Read(seed,stream),expected);
  EXPECT_GT(result.diagnostics.tied_reset,0u);
}
TEST_F(InitialSourceCuda, CompletePopulationIntervalHasExactNativeMultiplierAndSeedParity) {
  Fixture f(1,true,true);auto in=f.Input();src::PreparedSource exact,interval;
  ASSERT_EQ(src::PrepareSource(in,f.Limits(),exact).status,src::Status::Ok);
  in.native_population={search::NativePopulationPolicy::CompleteModelMultiplierTier,f.nodes.size(),1499999};
  in.contributors.native_auxiliary_nodes=SIZE_MAX;
  ASSERT_EQ(src::PrepareSource(in,f.Limits(),interval).status,src::Status::Ok);
  EXPECT_FALSE(interval.identity().native_model_nodes_exact);EXPECT_EQ(interval.removals().native_model_nodes,0u);
  EXPECT_EQ(Bits(exact.removals().engine_margin),Bits(interval.removals().engine_margin));
  EXPECT_EQ(interval.removals().used_count,exact.removals().used_count);
  src::DeviceSeed a,b;ASSERT_EQ(src::Prepare(exact,stream,a).status,src::Status::Ok);
  ASSERT_EQ(src::Prepare(interval,stream,b).status,src::Status::Ok);
  const auto first=Access::Read(a,stream),second=Access::Read(b,stream);
  ASSERT_EQ(first.history.size(),second.history.size());ASSERT_EQ(first.gaps.size(),second.gaps.size());
  for(std::size_t row=0;row<first.history.size();++row) {
    const auto& x=first.history[row].row;const auto& y=second.history[row].row;
    for(unsigned k=0;k<4;++k)EXPECT_EQ(x.irtlm[k],y.irtlm[k]);
    const auto xp=Penetration(x),yp=Penetration(y);
    for(unsigned k=0;k<5;++k)EXPECT_EQ(Bits(xp[k]),Bits(yp[k]));
  }
  for(std::size_t i=0;i<first.gaps.size();++i)EXPECT_EQ(Bits(first.gaps[i]),Bits(second.gaps[i]));
}
TEST_F(InitialSourceCuda, CapacityFailureNoPublicationAndOwnedSourceSurvivesBorrowedMutation) {
  Fixture f;auto input=f.Input();const auto expected=Oracle(f);auto limits=f.Limits();
  src::PreparedSource good;ASSERT_EQ(src::PrepareSource(input,limits,good).status,src::Status::Ok);
  const auto forecast=good.forecast();ASSERT_GT(forecast.peak_device_bytes,forecast.seed_device_bytes);
  auto short_limit=limits;short_limit.max_device_bytes=forecast.peak_device_bytes-1;
  src::PreparedSource missing;EXPECT_EQ(src::PrepareSource(input,short_limit,missing).status,src::Status::ResourceLimit);
  EXPECT_FALSE(missing.prepared());
  short_limit=limits;short_limit.max_pairs=1;src::PreparedSource capped;
  ASSERT_EQ(src::PrepareSource(input,short_limit,capped).status,src::Status::Ok);
  src::DeviceSeed seed;const auto rejected=src::Prepare(capped,stream,seed);
  EXPECT_EQ(rejected.status,src::Status::ResourceLimit);EXPECT_TRUE(rejected.counts_complete);EXPECT_FALSE(seed.prepared());
  // Every operand used by the GPU is now owned. Mutating the original source
  // after preparation cannot change a prepared seed or require a borrowed X.
  f.mesh.positions.assign(f.mesh.positions.size(),std::numeric_limits<double>::quiet_NaN());
  f.mains.assign(f.mains.size(),{});f.secondary.assign(f.secondary.size(),{});
  ASSERT_EQ(src::Prepare(good,stream,seed).status,src::Status::Ok);Same(Access::Read(seed,stream),expected);
  const auto identity=seed.identity();EXPECT_EQ(src::Prepare(capped,stream,seed).status,src::Status::AlreadyPrepared);
  EXPECT_EQ(seed.identity().source.source,identity.source.source);Same(Access::Read(seed,stream),expected);
}
TEST_F(InitialSourceCuda, GenuineMixedPostGapmSourceUsesTrueExpandedCountAndFinalGapBits) {
  for(unsigned mode:{0u,1u})for(bool reverse:{false,true}) {
    SCOPED_TRACE(mode);
    SCOPED_TRACE(reverse);
    MixedFixture f(mode,reverse);Expected expected;expected.geometric=f.Geometry();
    expected.generation=f.Input().mesh.source_generation;
    for(const auto& row:f.secondary)expected.source_ids.push_back(f.nodes[row.node].source_id);
    expected.inventory=NativeInventory(f.Inventory(expected.geometric));
    expected.before_tied=FullInitialHistory(f.Search(),f.native,expected.inventory.corner_gaps,expected.inventory.pairs);
    expected.rows=expected.before_tied.rows;std::vector<std::array<int,4>> keys(expected.rows.size());
    for(std::size_t row=0;row<keys.size();++row)std::copy_n(expected.rows[row].irtlm,4,keys[row].data());
    NativePreparedMain(f.mains.size(),keys);
    for(std::size_t row=0;row<keys.size();++row)std::copy_n(keys[row].data(),4,expected.rows[row].irtlm);
    src::PreparedSource source;ASSERT_EQ(src::PrepareSource(f.Input(),f.Limits(),source).status,src::Status::Ok);
    ASSERT_NE(f.mains.size(),2*f.topology.startup.primary_count);
    const auto removal=source.removals();ASSERT_EQ(removal.used_count,expected.geometric.removed_nodes.size());
    for(std::size_t m=0;m<=f.mains.size();++m)EXPECT_EQ(removal.main_offsets[m],expected.geometric.main_offsets[m]);
    for(std::size_t k=0;k<removal.used_count;++k)EXPECT_EQ(removal.removed_nodes[k],expected.geometric.removed_nodes[k]);
    src::DeviceSeed seed;const auto result=src::Prepare(source,stream,seed);ASSERT_EQ(result.status,src::Status::Ok);
    EXPECT_EQ(result.diagnostics.pairs,expected.inventory.pairs.size());EXPECT_GT(result.diagnostics.changed_gap_corners,0u);
    Same(Access::Read(seed,stream),expected);
  }
}
TEST_F(InitialSourceCuda, FixedMainSkewOneAndAllFixedSecondaryMatchWholeNativeMaskPacking) {
  for(unsigned mode:{0u,1u,2u}) {
    SCOPED_TRACE(mode);
    Fixture f;
    for(auto node:f.main_nodes){f.nodes[node].constraint=7;f.nodes[node].skew=1;}
    if(mode)for(auto& node:f.nodes){node.constraint=7;node.skew=1;}
    // PEN3A457 restricts fixed-axis screening to solids/coatings. Ordinary
    // shell sides retain candidate eligibility even when fixed and coplanar.
    if(mode==2)for(const auto& row:f.secondary)f.mesh.positions[3*row.node+2]=0.;
    const auto expected=Oracle(f);src::PreparedSource source;
    ASSERT_EQ(src::PrepareSource(f.Input(),f.Limits(),source).status,src::Status::Ok);
    src::DeviceSeed seed;const auto report=src::Prepare(source,stream,seed);ASSERT_EQ(report.status,src::Status::Ok);
    EXPECT_EQ(report.diagnostics.pairs,expected.inventory.pairs.size());Same(Access::Read(seed,stream),expected);
    EXPECT_GT(report.diagnostics.pairs,0u);
  }
}
TEST_F(InitialSourceCuda, GenuineMixedSolidPairExercisesNativeFixedAxisScreen) {
  MixedFixture f(0,false);const auto geometry=f.Geometry();
  const auto before=NativeInventory(f.Inventory(geometry));
  // This genuine source has node2 at(2,0,0), absent from all solid/shell
  // incidence, and a real solid/coating bottom face in z0. The native result
  // must explicitly contain that pair before the shared fixedZ screen.
  const auto found=std::find_if(before.pairs.begin(),before.pairs.end(),[&](const auto& pair) {
    if(f.secondary[std::size_t(pair[0]-1)].node!=2)return false;
    const auto& main=f.native.mains[std::size_t(pair[1]-1)];
    if(main.segment_type!=0&&main.segment_type<=int(f.mains.size()))return false;
    for(auto node:main.nodes)if(f.topology.input.positions.at(node).z!=0)return false;
    return true;
  });
  ASSERT_NE(found,before.pairs.end());const auto affected=*found;
  for(auto& node:f.nodes){node.constraint=7;node.skew=1;}
  Expected expected;expected.geometric=geometry;expected.inventory=NativeInventory(f.Inventory(geometry));
  EXPECT_EQ(std::find(expected.inventory.pairs.begin(),expected.inventory.pairs.end(),affected),expected.inventory.pairs.end());
  EXPECT_LT(expected.inventory.pairs.size(),before.pairs.size());
  expected.generation=f.Input().mesh.source_generation;
  for(const auto& row:f.secondary)expected.source_ids.push_back(f.nodes[row.node].source_id);
  expected.before_tied=FullInitialHistory(f.Search(),f.native,expected.inventory.corner_gaps,expected.inventory.pairs);expected.rows=expected.before_tied.rows;
  std::vector<std::array<int,4>> keys(expected.rows.size());
  for(std::size_t row=0;row<keys.size();++row)std::copy_n(expected.rows[row].irtlm,4,keys[row].data());
  NativePreparedMain(f.mains.size(),keys);
  for(std::size_t row=0;row<keys.size();++row)std::copy_n(keys[row].data(),4,expected.rows[row].irtlm);
  src::PreparedSource source;ASSERT_EQ(src::PrepareSource(f.Input(),f.Limits(),source).status,src::Status::Ok);
  src::DeviceSeed seed;const auto report=src::Prepare(source,stream,seed);ASSERT_EQ(report.status,src::Status::Ok);
  EXPECT_EQ(report.diagnostics.pairs,expected.inventory.pairs.size());Same(Access::Read(seed,stream),expected);
}
TEST_F(InitialSourceCuda, CompleteWarmInitializationCrossesNative128LaneCohorts) {
  auto mesh=Fixture::Mesh(0,false);
  for(unsigned i=0;i<28;++i){mesh.ids.push_back(mesh.ids.size()+1);mesh.positions.insert(mesh.positions.end(),{.25+.08*i,.4,.02});}
  Fixture f(std::move(mesh));f.global_gap=.1+5.;
  std::fill(f.main_gap.begin(),f.main_gap.end(),5.);
  for(auto& main:f.mains){main.maximum_gap=5.;for(auto& gap:main.gap)gap=5.;}
  const auto expected=Oracle(f);ASSERT_GT(expected.inventory.pairs.size(),128u);
  src::PreparedSource source;ASSERT_EQ(src::PrepareSource(f.Input(),f.Limits(),source).status,src::Status::Ok);
  src::DeviceSeed seed;const auto report=src::Prepare(source,stream,seed);ASSERT_EQ(report.status,src::Status::Ok);
  EXPECT_EQ(report.diagnostics.pairs,expected.inventory.pairs.size());EXPECT_GT(report.diagnostics.warm_after_tied,0u);
  Same(Access::Read(seed,stream),expected);
}
TEST_F(InitialSourceCuda, WholeNativeGridMatchesFallbackAndNintBoundaryNeighborhoods) {
  std::vector<double> separations{1.e14};
  // With two unit Q4s: NMN8, exact mean1, bbox=(D+1+2pad,1+2pad,2pad).
  // Solve the continuous raw-x NINT expression only to place test geometry
  // near half-integer thresholds. The whole native BUC supplies every expected
  // count/field; this construction supplies no production result.
  const double pad=double(.20f)+(.1+.05),y=1+2*pad,z=2*pad,c=y+z,d=y*z;
  for(double threshold:{3.5,7.5,31.5}) {
    const double t2=threshold*threshold;
    const double length=(c*t2+std::sqrt(c*c*t2*t2+18*d*t2))/9;
    const double center=length-1-2*pad;double low=center,high=center;
    for(unsigned i=0;i<16;++i){low=std::nextafter(low,0.);high=std::nextafter(high,std::numeric_limits<double>::infinity());}
    separations.push_back(low);separations.push_back(high);
  }
  std::vector<std::uint64_t> native_cells;
  for(double separation:separations) {
    SCOPED_TRACE(separation);
    Fixture f(Fixture::Distant(separation));const auto expected=Oracle(f);src::PreparedSource source;
    ASSERT_EQ(src::PrepareSource(f.Input(),f.Limits(),source).status,src::Status::Ok);
    src::DeviceSeed seed;const auto report=src::Prepare(source,stream,seed);ASSERT_EQ(report.status,src::Status::Ok);
    std::uint64_t cells=1;for(auto axis:report.diagnostics.grid)cells*=std::uint64_t(axis+2);
    EXPECT_EQ(cells,expected.inventory.initialized_voxel_slots);native_cells.push_back(expected.inventory.initialized_voxel_slots);
    EXPECT_EQ(CapturePairs(source,stream),expected.inventory.pairs);
    EXPECT_EQ(report.diagnostics.pairs,expected.inventory.pairs.size());Same(Access::Read(seed,stream),expected);
    if(separation==1.e14){EXPECT_EQ(report.diagnostics.grid[0],100);EXPECT_EQ(cells,918u);}
  }
  for(std::size_t i=1;i<native_cells.size();i+=2)EXPECT_NE(native_cells[i],native_cells[i+1]);
}
TEST_F(InitialSourceCuda, CompleteVoxelRangesMatchEveryNativePairAcrossSeparatedAndOverlappingBoxes) {
  for(double spacing:{4.,.125}) {
    SCOPED_TRACE(spacing);
    Fixture f(VoxelPatchMesh(spacing));const auto expected=Oracle(f);auto limits=f.Limits();
    // Full history wrapper remains bounded to this complete 114-node packet.
    ASSERT_GT(expected.inventory.pairs.size(),0u);
    src::PreparedSource source;ASSERT_EQ(src::PrepareSource(f.Input(),limits,source).status,src::Status::Ok);
    EXPECT_EQ(CapturePairs(source,stream),expected.inventory.pairs);
    src::DeviceSeed seed;const auto result=src::Prepare(source,stream,seed);
    ASSERT_EQ(result.status,src::Status::Ok);EXPECT_TRUE(result.counts_complete);
    EXPECT_EQ(result.diagnostics.pairs,expected.inventory.pairs.size());Same(Access::Read(seed,stream),expected);
    // Every secondary has the same x sweep interval. The separated geometry
    // must discard y/z-ineligible encounters before allocating tasks.
    if(spacing==4.)EXPECT_LT(result.diagnostics.encounters,f.mains.size()*f.secondary.size()/2);
  }
}
TEST_F(InitialSourceCuda, CompleteVoxelTaskCountAdmitsExactCapAndRejectsOneShortBeforePublication) {
  Fixture f(VoxelPatchMesh(4.));const auto expected=Oracle(f);auto limits=f.Limits();
  src::PreparedSource baseline;ASSERT_EQ(src::PrepareSource(f.Input(),limits,baseline).status,src::Status::Ok);
  src::DeviceSeed initial;const auto report=src::Prepare(baseline,stream,initial);
  ASSERT_EQ(report.status,src::Status::Ok);ASSERT_GT(report.diagnostics.tasks,1u);
  limits.max_tasks=report.diagnostics.tasks;src::PreparedSource exact;
  ASSERT_EQ(src::PrepareSource(f.Input(),limits,exact).status,src::Status::Ok);
  src::DeviceSeed accepted;const auto complete=src::Prepare(exact,stream,accepted);
  ASSERT_EQ(complete.status,src::Status::Ok);EXPECT_EQ(complete.diagnostics.tasks,report.diagnostics.tasks);
  EXPECT_EQ(CapturePairs(exact,stream),expected.inventory.pairs);Same(Access::Read(accepted,stream),expected);
  --limits.max_tasks;src::PreparedSource short_source;
  ASSERT_EQ(src::PrepareSource(f.Input(),limits,short_source).status,src::Status::Ok);
  src::DeviceSeed missing;const auto failed=src::Prepare(short_source,stream,missing);
  EXPECT_EQ(failed.status,src::Status::ResourceLimit);EXPECT_FALSE(failed.counts_complete);
  EXPECT_EQ(failed.diagnostics.tasks,report.diagnostics.tasks);EXPECT_EQ(failed.diagnostics.encounters,report.diagnostics.encounters);
  EXPECT_EQ(failed.diagnostics.pairs,0u);EXPECT_FALSE(missing.prepared());
  ASSERT_EQ(src::Prepare(exact,stream,missing).status,src::Status::Ok);Same(Access::Read(missing,stream),expected);
}
}
