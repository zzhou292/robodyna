#include "lib_utest/q4_planar_geometry_fixture.h"

#include <gtest/gtest.h>
#include <cfloat>
#include <cmath>
#include <limits>

namespace {
namespace sc=tlfea::contact;
namespace fixture=q4_planar_test;
using PStatus=sc::PlanarContactStatus;
using Single=fixture::Single;

void Same(sc::Vec3 a,sc::Vec3 b) { EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.y,b.y); EXPECT_EQ(a.z,b.z); }
void Same(sc::VectorView a,sc::VectorView b) {
  EXPECT_EQ(a.data,b.data); EXPECT_EQ(a.node_count,b.node_count);
  EXPECT_EQ(a.node_stride,b.node_stride); EXPECT_EQ(a.component_stride,b.component_stride);
}
void Same(const sc::Q4PreparedIntegration& a,const sc::Q4PreparedIntegration& b) {
  EXPECT_EQ(a.covered,b.covered);
  Same(a.input.surface.positions,b.input.surface.positions); Same(a.input.surface.velocities,b.input.surface.velocities);
  EXPECT_EQ(a.input.surface.parents,b.input.surface.parents); EXPECT_EQ(a.input.surface.parent_count,b.input.surface.parent_count);
  EXPECT_EQ(a.input.mass.inverse_mass,b.input.mass.inverse_mass); EXPECT_EQ(a.input.mass.translation_fixed_bits,b.input.mass.translation_fixed_bits);
  EXPECT_EQ(a.input.mass.node_count,b.input.mass.node_count); EXPECT_EQ(a.input.mass.base_epoch,b.input.mass.base_epoch);
  EXPECT_EQ(a.input.parent_index,b.input.parent_index); EXPECT_EQ(a.input.attempt,b.input.attempt);
  EXPECT_EQ(a.input.wall_x,b.input.wall_x); EXPECT_EQ(a.input.projected_area,b.input.projected_area);
  EXPECT_EQ(a.input.stiffness_per_area,b.input.stiffness_per_area); EXPECT_EQ(a.input.max_penetration,b.input.max_penetration);
}
void Same(sc::Q4PlanarReferenceView a,sc::Q4PlanarReferenceView b) {
  ASSERT_EQ(a.parent_count,b.parent_count); EXPECT_EQ(a.global_node_count,b.global_node_count);
  EXPECT_EQ(a.wall_x,b.wall_x); EXPECT_EQ(a.wall_tolerance,b.wall_tolerance);
  for (unsigned p=0;p<a.parent_count;++p) {
    const auto& x=a.parents[p]; const auto& y=b.parents[p];
    for (unsigned n=0;n<4;++n) { EXPECT_EQ(x.parent.nodes[n],y.parent.nodes[n]); Same(x.reference_projection[n],y.reference_projection[n]); }
    EXPECT_EQ(x.parent.feature_id,y.parent.feature_id); EXPECT_EQ(x.parent.parent_element_id,y.parent.parent_element_id);
    EXPECT_EQ(x.parent.parent_face_id,y.parent.parent_face_id); EXPECT_EQ(x.parent.half_thickness,y.parent.half_thickness);
    EXPECT_EQ(x.projected_area,y.projected_area); EXPECT_EQ(x.area_enclosure.lower,y.area_enclosure.lower);
    EXPECT_EQ(x.area_enclosure.upper,y.area_enclosure.upper); EXPECT_EQ(x.covered,y.covered);
  }
}

TEST(Q4PlanarGeometry, CopiesNaturalReferenceAndFeedsActualAreaWithoutOwningState) {
  const auto input_wall=fixture::Square(); sc::PlanarWallGeometry wall;
  ASSERT_EQ(wall.Initialize(input_wall.view()).status,PStatus::Ok);
  Single surface; surface.Gaps(.01,.02,.03,.04); const auto initial=surface.Input();
  sc::Q4PlanarGeometry geometry;
  ASSERT_EQ(geometry.Initialize(wall,initial.surface,initial.mass,fixture::Clearance).status,PStatus::Ok);
  const auto reference=geometry.view(); ASSERT_EQ(reference.parent_count,1u);
  const auto& parent=reference.parents[0]; EXPECT_TRUE(parent.covered);
  EXPECT_EQ(parent.projected_area,1); EXPECT_EQ(parent.area_enclosure.lower,1); EXPECT_EQ(parent.area_enclosure.upper,1);
  EXPECT_EQ(parent.parent.feature_id,73u); EXPECT_EQ(parent.parent.parent_element_id,42u);
  EXPECT_GT(parent.reference_projection[0].y,parent.reference_projection[1].y);
  EXPECT_GT(parent.reference_projection[0].z,parent.reference_projection[3].z);
  surface.Gaps(.04,-.02,.01,-.03);  // Bilinear normal warping is supported.
  auto current=surface.Input();
  ASSERT_EQ(sc::ValidateQ4PlanarMotion(reference,current.surface,current.mass),PStatus::Ok);
  sc::Q4PreparedIntegration selection;
  ASSERT_EQ(sc::PrepareQ4PlanarIntegration(reference,current.surface,current.mass,0,1200,.1,17,&selection),PStatus::Ok);
  EXPECT_TRUE(selection.covered); EXPECT_EQ(selection.input.wall_x,0); EXPECT_EQ(selection.input.projected_area,1);
  EXPECT_EQ(selection.input.surface.positions.data,surface.position); EXPECT_EQ(selection.input.attempt,17u);
  const auto before=geometry;
  surface.position[surface.parent.nodes[0]+4]+=1;
  EXPECT_EQ(geometry.Initialize(wall,current.surface,current.mass,fixture::Clearance).status,PStatus::InvalidInput);
  Same(geometry.view(),before.view());
  EXPECT_EQ(parent.reference_projection[0].y,.5);  // Copied reference survived source mutation.
}

TEST(Q4PlanarGeometry, FiniteWallAndDisplayDiagonalsHaveOneCoverageAndStableSeamOwner) {
  for (unsigned variant=0;variant<4;++variant) {
    const auto input_wall=fixture::Square(variant); sc::PlanarWallGeometry wall;
    ASSERT_EQ(wall.Initialize(input_wall.view()).status,PStatus::Ok) << variant;
    Single surface; const auto input=surface.Input(); sc::Q4PlanarGeometry geometry;
    ASSERT_EQ(geometry.Initialize(wall,input.surface,input.mass,fixture::Clearance).status,PStatus::Ok);
    const auto& parent=geometry.view().parents[0]; EXPECT_TRUE(parent.covered);
    const unsigned triangles[4][3]={{0,1,2},{0,2,3},{0,1,3},{1,2,3}};
    for (const auto& indices:triangles) {
      sc::TriangleGeometry triangle; triangle.face_id=73;
      for (unsigned n=0;n<3;++n) {
        triangle.vertices[n]=parent.reference_projection[indices[n]];
        triangle.vertex_ids[n]=parent.parent.nodes[indices[n]];
      }
      bool covered=false;
      ASSERT_EQ(wall.ClassifyTriangle(triangle,fixture::Clearance,&covered).status,PStatus::Ok); EXPECT_TRUE(covered);
    }
    std::uint32_t owner=UINT32_MAX; sc::TrianglePointGeometry point;
    ASSERT_EQ(sc::planar_detail::FindOwner({0,0,0},wall.faces().data(),static_cast<unsigned>(wall.faces().size()),
                                         wall.tolerance(),&owner,&point),sc::Status::kOk);
    ASSERT_LT(owner,wall.faces().size()); EXPECT_EQ(wall.faces()[owner].geometry.face_id,fixture::LargeId+2);
    EXPECT_EQ(point.feature.kind,variant == 2 ? sc::FeatureKind::kVertex : sc::FeatureKind::kEdge);
  }
}

TEST(Q4PlanarGeometry, SharedParentsPreserveAnalyticResultantMomentAndWorkAcrossWallMeshes) {
  constexpr double stiffness=16,scale=stiffness*fixture::Depth;
  const long double coefficient[6]={5.L/288,13.L/288,43.L/288,11.L/288,30.L/288,6.L/288};
  std::array<double,6> baseline{},baseline_error{};
  for (unsigned variant=0;variant<4;++variant) {
    const auto input_wall=fixture::Square(variant); sc::PlanarWallGeometry wall;
    ASSERT_EQ(wall.Initialize(input_wall.view()).status,PStatus::Ok);
    fixture::Pair pair; sc::Q4PlanarGeometry geometry; fixture::Scratch scratch;
    ASSERT_EQ(geometry.Initialize(wall,pair.surface(),pair.mass(),fixture::Clearance).status,PStatus::Ok);
    std::array<double,6> force{},error{}; double energy=0,energy_error=0;
    for (unsigned p=0;p<2;++p) {
      sc::Q4PreparedIntegration selected;
      ASSERT_EQ(sc::PrepareQ4PlanarIntegration(geometry.view(),pair.surface(),pair.mass(),p,stiffness,fixture::Depth,7,&selected),PStatus::Ok);
      ASSERT_TRUE(selected.covered);
      auto limits=q4_contact_test::Limits(fixture::Depth); limits.force_error*=stiffness; limits.energy_error*=stiffness;
      sc::Q4IntegrationResult result;
      ASSERT_EQ(sc::IntegrateQ4NormalContact(selected.input,limits,scratch.view(),&result).status,sc::Q4IntegrationStatus::Ok);
      for (unsigned n=0;n<4;++n) {
        const auto global=result.nodal.nodes[n];
        force[global]+=result.nodal.forces[n].x; error[global]+=result.force[n].error;
        Same(result.nodal.couples[n],{});
      }
      energy+=result.potential.value; energy_error+=result.potential.error;
    }
    sc::Vec3 moment; double resultant=0,power=0,force_error=0,moment_error=0,power_error=0;
    for (unsigned n=0;n<6;++n) {
      const double rounding=64*DBL_EPSILON;
      EXPECT_LE(std::abs(static_cast<long double>(force[n])+scale*coefficient[n]),error[n]+rounding);
      if (variant == 0) { baseline[n]=force[n]; baseline_error[n]=error[n]; }
      else EXPECT_NEAR(force[n],baseline[n],error[n]+baseline_error[n]+rounding);
      const auto x=pair.surface().positions.at(n),v=pair.surface().velocities.at(n);
      resultant+=force[n]; force_error+=error[n];
      moment=sc::Add(moment,sc::geometry_detail::Cross(x,{force[n],0,0}));
      power+=force[n]*v.x; moment_error+=(std::abs(x.y)+std::abs(x.z))*error[n]; power_error+=std::abs(v.x)*error[n];
    }
    const double roundoff=128*DBL_EPSILON;
    EXPECT_NEAR(resultant,-scale*3/8,force_error+roundoff);
    EXPECT_EQ(moment.x,0); EXPECT_NEAR(moment.y,-scale/12,moment_error+roundoff);
    EXPECT_NEAR(moment.z,scale/16,moment_error+roundoff);
    EXPECT_NEAR(power,-scale*5/288,power_error+roundoff);
    // The same arbitrary nodal virtual direction scaled by 1/64 gives work.
    EXPECT_NEAR(power/64,-scale*5/(288*64),power_error/64+roundoff);
    EXPECT_NEAR(energy,stiffness*fixture::Depth*fixture::Depth/9,energy_error+roundoff);
  }
}

TEST(Q4PlanarGeometry, OutsideAndHoleInteriorsAreExplicitlySkippedWithoutValidForceInput) {
  for (bool inside_hole:{false,true}) {
    const auto input_wall=inside_hole ? fixture::Ring() : fixture::Square(); sc::PlanarWallGeometry wall;
    ASSERT_EQ(wall.Initialize(input_wall.view()).status,PStatus::Ok);
    Single surface; surface.Gaps(1,1,1,1);
    for (unsigned n=0;n<4;++n) {
      if (inside_hole) { surface.position[n+4]*=.25; surface.position[n+8]*=.25; }
      else surface.position[n+4]+=3;
    }
    const auto input=surface.Input(); sc::Q4PlanarGeometry geometry;
    ASSERT_EQ(geometry.Initialize(wall,input.surface,input.mass,fixture::Clearance).status,PStatus::Ok);
    EXPECT_FALSE(geometry.view().parents[0].covered);
    sc::Q4PreparedIntegration result; result.covered=true; result.input=input;
    ASSERT_EQ(sc::PrepareQ4PlanarIntegration(geometry.view(),input.surface,input.mass,0,1,1,7,&result),PStatus::Ok);
    EXPECT_FALSE(result.covered); EXPECT_EQ(result.input.projected_area,0);
    EXPECT_EQ(result.input.surface.parents,nullptr); EXPECT_EQ(result.input.attempt,0u);
  }
}

TEST(Q4PlanarGeometry, ExposedEdgesAndEnclosedHoleRejectBeforeReferencePublication) {
  for (unsigned variant=0;variant<4;++variant) {
    const auto input_wall=variant >= 2 ? fixture::Ring() : fixture::Square(); sc::PlanarWallGeometry wall;
    ASSERT_EQ(wall.Initialize(input_wall.view()).status,PStatus::Ok);
    Single surface;
    for (unsigned n=0;n<4;++n) {
      if (variant == 0) surface.position[n+4]+=2;
      if (variant == 1) surface.position[n+4]+=1.5-.5*fixture::Clearance;
      if (variant == 3) { surface.position[n+4]*=.5; surface.position[n+8]*=.5; surface.position[n+4]+=.4; }
    }
    auto input=surface.Input(); sc::Q4PlanarGeometry geometry;
    EXPECT_EQ(geometry.Initialize(wall,input.surface,input.mass,fixture::Clearance).status,PStatus::AmbiguousBoundary) << variant;
    EXPECT_FALSE(geometry.initialized()); EXPECT_EQ(geometry.view().parent_count,0u);
    Single clean; for (unsigned n=0;n<4;++n) clean.position[n+4]-=1;
    input=clean.Input();
    ASSERT_EQ(geometry.Initialize(wall,input.surface,input.mass,fixture::Clearance).status,PStatus::Ok);
  }
}

TEST(Q4PlanarGeometry, ProjectedWarpageOrderingAspectAndUnresolvedSidesReject) {
  const auto input_wall=fixture::Square(); sc::PlanarWallGeometry wall;
  ASSERT_EQ(wall.Initialize(input_wall.view()).status,PStatus::Ok);
  for (unsigned variant=0;variant<7;++variant) {
    Single surface;
    if (variant == 0) surface.position[surface.parent.nodes[0]+4]+=.001;
    if (variant == 1) surface.position[surface.parent.nodes[0]+8]+=.001;
    if (variant == 2) std::swap(surface.parent.nodes[0],surface.parent.nodes[1]);
    if (variant == 3) for (unsigned n=0;n<4;++n) surface.position[n+8]/=2*sc::MaxQ4ProjectedAspectRatio;
    if (variant == 4) for (unsigned n=0;n<4;++n) { surface.position[n+4]*=1e-15; surface.position[n+8]*=1e-15; }
    if (variant == 5) surface.parent.half_thickness=.001;
    if (variant == 6) for (unsigned n=0;n<4;++n) {
      const double y=surface.position[n+4],z=surface.position[n+8];
      surface.position[n+4]=.8*y-.6*z; surface.position[n+8]=.6*y+.8*z;
    }
    const auto input=surface.Input(); sc::Q4PlanarGeometry geometry;
    EXPECT_EQ(geometry.Initialize(wall,input.surface,input.mass,fixture::Clearance).status,PStatus::UnsupportedGeometry) << variant;
    EXPECT_FALSE(geometry.initialized());
  }
  Single boundary;
  for (unsigned n=0;n<4;++n) boundary.position[n+8]/=sc::MaxQ4ProjectedAspectRatio;
  const auto input=boundary.Input(); sc::Q4PlanarGeometry geometry;
  EXPECT_EQ(geometry.Initialize(wall,input.surface,input.mass,fixture::Clearance).status,PStatus::Ok);
}

TEST(Q4PlanarGeometry, CoincidentIndependentNodesDoNotPermitInteriorOverlapButEdgeAdjacencyDoes) {
  const auto input_wall=fixture::Square(); sc::PlanarWallGeometry wall;
  ASSERT_EQ(wall.Initialize(input_wall.view()).status,PStatus::Ok);
  for (double shift:{0.,.5,1.}) {
    std::array<double,24> x{},v{}; std::array<double,8> inverse; inverse.fill(1);
    std::array<std::uint8_t,8> fixed; fixed.fill(6);
    const sc::SurfaceQ4 parents[2]={{{0,1,2,3},73,42,0,0},{{4,5,6,7},74,43,0,0}};
    const double y[4]={.5,-.5,-.5,.5},z[4]={.5,.5,-.5,-.5};
    for (unsigned p=0;p<2;++p) for (unsigned n=0;n<4;++n) { x[4*p+n+8]=y[n]+p*shift; x[4*p+n+16]=z[n]; }
    const sc::Q4SurfaceView surface{{x.data(),8,1,8},{v.data(),8,3,1},parents,2};
    const sc::Q4FixedYZMassView mass{inverse.data(),fixed.data(),8,0}; sc::Q4PlanarGeometry geometry;
    EXPECT_EQ(geometry.Initialize(wall,surface,mass,fixture::Clearance).status,
              shift == 1 ? PStatus::Ok : PStatus::UnsupportedGeometry);
  }
}

TEST(Q4PlanarGeometry, LateCurrentParentFailurePreservesSelectedInputAndCanRetry) {
  const auto input_wall=fixture::Square(); sc::PlanarWallGeometry wall;
  ASSERT_EQ(wall.Initialize(input_wall.view()).status,PStatus::Ok);
  fixture::Pair pair; sc::Q4PlanarGeometry geometry;
  ASSERT_EQ(geometry.Initialize(wall,pair.surface(),pair.mass(),fixture::Clearance).status,PStatus::Ok);
  sc::Q4PreparedIntegration before;
  ASSERT_EQ(sc::PrepareQ4PlanarIntegration(geometry.view(),pair.surface(),pair.mass(),0,16,.1,7,&before),PStatus::Ok);
  for (unsigned variant=0;variant<9;++variant) {
    auto bad=pair;
    if (variant == 0) bad.x[5+6]=std::nextafter(bad.x[5+6],2.);
    if (variant == 1) bad.v[3*5+2]=std::numeric_limits<double>::denorm_min();
    if (variant == 2) bad.v[3*5]=std::numeric_limits<double>::quiet_NaN();
    if (variant == 3) bad.parents[1].feature_id+=1;
    if (variant == 4) bad.parents[1].nodes[3]=4;
    if (variant == 5) bad.fixed[5]=0;
    if (variant == 6) bad.inverse[5]=0;
    if (variant == 7) bad.parents[1].half_thickness=.1;
    if (variant == 8) { bad.fixed[5]=7; bad.inverse[5]=0; bad.v[3*5]=.01; }
    auto output=before;
    EXPECT_NE(sc::PrepareQ4PlanarIntegration(geometry.view(),bad.surface(),bad.mass(),0,16,.1,8,&output),PStatus::Ok) << variant;
    Same(output,before);
    ASSERT_EQ(sc::PrepareQ4PlanarIntegration(geometry.view(),pair.surface(),pair.mass(),0,16,.1,7,&output),PStatus::Ok);
    Same(output,before);
  }
  auto stationary=pair;
  stationary.fixed[5]=7; stationary.inverse[5]=0; stationary.v[3*5]=0;
  EXPECT_EQ(sc::ValidateQ4PlanarMotion(geometry.view(),stationary.surface(),stationary.mass()),PStatus::Ok);
  pair.x[5]+=.01; pair.v[3*5]-=.02; pair.inverse[5]*=2;
  EXPECT_EQ(sc::ValidateQ4PlanarMotion(geometry.view(),pair.surface(),pair.mass()),PStatus::Ok);
}

TEST(Q4PlanarGeometry, DuplicateIdentitiesAndInvalidStartupCanRetryWithoutPartialReference) {
  const auto input_wall=fixture::Square(); sc::PlanarWallGeometry wall;
  ASSERT_EQ(wall.Initialize(input_wall.view()).status,PStatus::Ok);
  for (unsigned variant=0;variant<5;++variant) {
    fixture::Pair pair; sc::Q4PlanarGeometry geometry;
    if (variant == 0) pair.parents[1].feature_id=pair.parents[0].feature_id;
    if (variant == 1) pair.parents[1].parent_element_id=pair.parents[0].parent_element_id;
    if (variant == 2) pair.x[5+12]=std::nextafter(pair.x[5+12],1.);
    if (variant == 3) pair.v[3*5+1]=.01;
    if (variant == 4) pair.inverse[5]=0;
    EXPECT_NE(geometry.Initialize(wall,pair.surface(),pair.mass(),fixture::Clearance).status,PStatus::Ok);
    EXPECT_FALSE(geometry.initialized()); EXPECT_EQ(geometry.view().global_node_count,0u);
    const fixture::Pair clean;
    ASSERT_EQ(geometry.Initialize(wall,clean.surface(),clean.mass(),fixture::Clearance).status,PStatus::Ok);
  }
  fixture::Pair pair; sc::Q4PlanarGeometry geometry; const sc::PlanarWallGeometry missing;
  EXPECT_EQ(geometry.Initialize(missing,pair.surface(),pair.mass(),fixture::Clearance).status,PStatus::NotInitialized);
  const sc::SurfaceQ4 too_many[3]={pair.parents[0],pair.parents[1],pair.parents[0]};
  auto excessive=pair.surface(); excessive.parents=too_many; excessive.parent_count=3;
  EXPECT_EQ(geometry.Initialize(wall,excessive,pair.mass(),fixture::Clearance).status,PStatus::ResourceLimit);
  EXPECT_EQ(geometry.Initialize(wall,pair.surface(),pair.mass(),8*wall.tolerance()).status,PStatus::InvalidInput);
  sc::Q4PreparedIntegration output; output.covered=true; const auto before=output;
  EXPECT_EQ(sc::PrepareQ4PlanarIntegration(geometry.view(),pair.surface(),pair.mass(),0,16,.1,7,&output),PStatus::NotInitialized);
  Same(output,before);
}
}  // namespace
