#include "VehicleCatalogFixture.h"
#include <limits>

namespace vehicle_catalog_test {
TEST(VehiclePlasticityCatalog, ExplicitCountsBytesAndOverflowRejectBeforeBorrowedReads) {
  Fixture f;fe::ShellBatchBinding b;PrepareGeometry(f,b);Catalog full;
  ASSERT_EQ(full.InitializeCatalog(b,f.input(),f.limits()).status,Status::Success);
  Catalog next;const auto before=Bytes(next);auto poison=f.input();
  poison.curves=reinterpret_cast<const fe::ShellPlasticityCurveInput*>(1);
  poison.materials=reinterpret_cast<const fe::ShellPlasticityMaterialInput*>(1);
  poison.sections=reinterpret_cast<const fe::ShellPlasticitySectionInput*>(1);
  poison.parents=reinterpret_cast<const fe::ShellPlasticityParentInput*>(1);
  EXPECT_EQ(next.Initialize(b,poison).status,Status::ResourceLimit);
  EXPECT_EQ(next.Initialize(b,poison,fe::ShellHostBindingLimits::Vehicle()).status,Status::ResourceLimit);
  for(unsigned fault=0;fault<9;++fault) {
    auto in=poison;auto cap=f.limits();
    if(fault==0) cap.max_owned_bytes=full.host_bytes()-1;
    if(fault==1) cap.max_startup_scratch_bytes=full.startup_scratch_bytes()-1;
    if(fault==2) cap.max_parents=f.parents.size()-1;
    if(fault==3) cap.max_nodes=f.geometry.count-1;
    if(fault==4) cap.max_definitions=f.materials.size()-1;
    if(fault==5) cap.max_parents=fe::MaxVehiclePlasticityCatalogParents+1;
    if(fault==6) cap.max_nodes=fe::MaxVehiclePlasticityCatalogNodes+1;
    if(fault==7) in.parent_count=std::numeric_limits<std::size_t>::max();
    if(fault==8) in.material_count=std::numeric_limits<std::size_t>::max();
    EXPECT_EQ(next.InitializeCatalog(b,in,cap).status,Status::ResourceLimit)<<fault;
    EXPECT_EQ(Bytes(next),before);EXPECT_EQ(next.startup_scratch_bytes(),0u);
  }
  auto exact=f.limits();exact.max_owned_bytes=full.host_bytes();
  exact.max_startup_scratch_bytes=full.startup_scratch_bytes();
  ASSERT_EQ(next.InitializeCatalog(b,f.input(),exact).status,Status::Success);
  EXPECT_TRUE(next.SameScope(full));EXPECT_EQ(next.host_bytes(),full.host_bytes());
  // Preserve older host limits semantics: byte admission was a caller budget,
  // and the binding-only scratch field did not govern catalog preparation.
  Fixture small(40,40,17,8);fe::ShellBatchBinding legacy_binding;PrepareGeometry(small,legacy_binding);
  fe::ShellHostBindingLimits legacy;legacy.max_owned_bytes=std::numeric_limits<std::size_t>::max();
  legacy.max_startup_scratch_bytes=0;Catalog legacy_catalog;
  EXPECT_EQ(legacy_catalog.Initialize(legacy_binding,small.input(),legacy).status,Status::Success);
}
TEST(VehiclePlasticityCatalog, IndexedParentChecksKeepFirstSourceFailureAndNativeBits) {
  Fixture f;fe::ShellBatchBinding b;PrepareGeometry(f,b);Catalog c;const auto before=Bytes(c);
  const auto original=f.parents;
  // Unsorted PID IDs: two inconsistent assignments. Earliest source row wins,
  // and ID/family/native-source checks still precede the PID consistency check.
  f.parents[100].source_part_id=f.parents[0].source_part_id;
  f.parents[200].source_part_id=f.parents[1].source_part_id;
  auto r=c.InitializeCatalog(b,f.input(),f.limits());
  EXPECT_EQ(r.status,Status::IdentityMismatch);EXPECT_EQ(r.entry,100u);EXPECT_EQ(Bytes(c),before);
  f.parents[99].source_parent_id=0;
  r=c.InitializeCatalog(b,f.input(),f.limits());EXPECT_EQ(r.status,Status::InvalidParent);EXPECT_EQ(r.entry,99u);
  f.parents=original;
  const auto mid=f.parents.back().material_id;
  const auto m=(std::uint64_t{1}<<59)+f.materials.size()-mid;
  f.materials[m].density_kg_m3=std::nextafter(f.materials[m].density_kg_m3,INFINITY);
  auto first=std::find_if(f.parents.begin(),f.parents.end(),[&](const auto& p){return p.material_id==mid;})-f.parents.begin();
  r=c.InitializeCatalog(b,f.input(),f.limits());EXPECT_EQ(r.status,Status::IdentityMismatch);
  EXPECT_EQ(r.entry,static_cast<std::size_t>(first));EXPECT_EQ(Bytes(c),before);
  f.materials[m].density_kg_m3=1024.;
  ASSERT_EQ(c.InitializeCatalog(b,f.input(),f.limits()).status,Status::Success);
}
TEST(VehiclePlasticityCatalog, LateAssignmentsPoolAndDeclarationsFailAtomicallyThenRetry) {
  Fixture f;fe::ShellBatchBinding b;PrepareGeometry(f,b);
  for(unsigned fault=0;fault<7;++fault) {
    auto in=f.input();auto parents=f.parents;auto materials=f.materials;auto sections=f.sections;
    auto curves=f.curves;std::array<double,2> bady=f.y;
    in.parents=parents.data();in.materials=materials.data();in.sections=sections.data();in.curves=curves.data();
    if(fault==0) parents.back().family_index=parents[parents.size()-2].family_index;
    if(fault==1) parents.back().source_parent_id=1;
    if(fault==2) parents.back().source_part_id=parents[parents.size()-2].source_part_id;
    if(fault==3) {parents.back().source_part_id=9;parents.back().material_id=9;}
    if(fault==4) {bady.back()=-1;curves[0].curve.yield_stress_pa=bady.data();}
    if(fault==5) curves[0].curve.count=std::numeric_limits<std::uint32_t>::max();
    if(fault==6) materials.back().linear.tangent_modulus_pa=materials.back().young_pa;
    Catalog c;const auto before=Bytes(c);const auto report=c.InitializeCatalog(b,in,f.limits());
    EXPECT_NE(report.status,Status::Success)<<fault;EXPECT_EQ(Bytes(c),before);
    if(fault<4) EXPECT_EQ(report.entry,parents.size()-1);
    EXPECT_EQ(c.InitializeCatalog(b,f.input(),f.limits()).status,Status::Success)<<fault;
  }
}
TEST(VehiclePlasticityCatalog, EveryRetainedAndTransientAllocationRejectsAtomically) {
  Fixture f;fe::ShellBatchBinding b;PrepareGeometry(f,b);
  // Five retained arrays and three transient arrays, each with a separate
  // shared-control allocation. Legacy inline extents remain allocation-free.
  for(std::ptrdiff_t fail=0;fail<16;++fail) {
    Catalog c;const auto before=Bytes(c);fe::ShellPlasticityBindingReport r;
    {host_shell_test::AllocationFailure failure(fail);r=c.InitializeCatalog(b,f.input(),f.limits());}
    EXPECT_EQ(r.status,Status::ResourceLimit)<<fail;EXPECT_EQ(Bytes(c),before);
    EXPECT_EQ(c.InitializeCatalog(b,f.input(),f.limits()).status,Status::Success);
  }
  Catalog c;fe::ShellPlasticityBindingReport r;
  {host_shell_test::AllocationFailure failure(16);r=c.InitializeCatalog(b,f.input(),f.limits());}
  EXPECT_EQ(r.status,Status::Success);
}
TEST(VehiclePlasticityCatalog, FullDefinitionBoundaryAndAllAnalyticZeroPoolStayOwned) {
  Fixture f(2049,1049,4097,1024);fe::ShellBatchBinding b;PrepareGeometry(f,b);Catalog mixed;
  ASSERT_EQ(mixed.InitializeCatalog(b,f.input(),f.limits()).status,Status::Success);
  EXPECT_EQ(mixed.material_count(),1024u);EXPECT_EQ(mixed.section_count(),1024u);
  for(std::size_t i=0;i<f.parents.size();++i) CheckParent(f,mixed,i);
  auto in=f.input();in.material_count=1025;Catalog invalid;const auto before=Bytes(invalid);
  EXPECT_EQ(invalid.InitializeCatalog(b,in,f.limits()).status,Status::ResourceLimit);
  const auto section=f.sections.back();f.sections.back().section_id=f.sections.front().section_id;
  auto report=invalid.InitializeCatalog(b,f.input(),f.limits());
  EXPECT_EQ(report.status,Status::InvalidSection);EXPECT_EQ(report.entry,1023u);EXPECT_EQ(Bytes(invalid),before);
  f.sections.back()=section;
  for(auto& m:f.materials) {m.hardening=Kind::LinearLaw44;m.curve_id=0;m.linear={20000.,20000.};}
  in=f.input();in.curve_count=0;in.curves=nullptr;
  ASSERT_EQ(invalid.InitializeCatalog(b,in,f.limits()).status,Status::Success);
  EXPECT_EQ(invalid.curve_count(),0u);EXPECT_EQ(invalid.curve_point_count(),0u);
  for(std::size_t i=0;i<f.parents.size();++i) CheckParent(f,invalid,i);
  EXPECT_FALSE(invalid.SameScope(mixed));
}
TEST(VehiclePlasticityCatalog, OwnedMixedParametersSurviveSourcesAndCopiesWithoutAllocation) {
  std::unique_ptr<Catalog> copy,moved;fe::sections::PointParameters table,analytic;
  fe::ShellBindingFamily table_family{},analytic_family{};std::size_t ti=0,ai=0;
  {
    Fixture f;fe::ShellBatchBinding b;PrepareGeometry(f,b);Catalog c;
    ASSERT_EQ(c.InitializeCatalog(b,f.input(),f.limits()).status,Status::Success);
    for(std::size_t i=0;i<f.parents.size();++i) CheckParent(f,c,i);
    // Q family0 uses material0/table; family1 uses material1/analytic.
    table_family=analytic_family=fe::ShellBindingFamily::Qeph;ti=0;ai=1;
    copy=std::make_unique<Catalog>(c);moved=std::make_unique<Catalog>(std::move(*copy));
    bool same=false;
    {host_shell_test::AllocationFailure fail;Catalog inline_copy(*moved);same=inline_copy.SameScope(*copy);}
    EXPECT_TRUE(same);ASSERT_TRUE(moved->Parameters(table_family,ti,&table));
    ASSERT_TRUE(moved->Parameters(analytic_family,ai,&analytic));
    fe::sections::PointParameters old_table;ASSERT_TRUE(c.Parameters(table_family,ti,&old_table));
    EXPECT_NE(table.curve.yield_stress_pa,old_table.curve.yield_stress_pa);
    f.y[0]=-1;f.materials[1].linear.initial_yield_pa=-1;
  }
  EXPECT_DOUBLE_EQ(table.curve.yield_stress_pa[0],20000.);
  EXPECT_DOUBLE_EQ(analytic.linear.initial_yield_pa,20001.);
  EXPECT_EQ(analytic.curve.plastic_strain,nullptr);
  EXPECT_TRUE(copy->SameScope(*moved));
}
} // namespace vehicle_catalog_test
