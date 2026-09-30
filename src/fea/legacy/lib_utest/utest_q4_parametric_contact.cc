#include "lib_src/collision/Q4ParametricContact.h"
#include "lib_src/collision/Q4RectangularIntegration.h"
#include "lib_src/collision/SurfaceContactGeometry.h"
#include "lib_utest/q4_prescribed_contact_fixture.h"

#include <gtest/gtest.h>
#include <array>
#include <limits>

namespace {
namespace sc=tlfea::contact;
namespace fixture=q4_prescribed_test;
namespace geometry=q4_planar_test;
using Code=sc::Q4ParametricStatus;
using Result=sc::Q4ParametricResult;
using Single=geometry::Single;
sc::Q4ParametricConfig Config() {
  const auto old=fixture::Config();
  return {sc::Q4ReferenceContactMeasure::CenterAreaUniformNatural,old.stiffness_per_area,
          old.maximum_penetration,old.sweep.exposed_clearance,old.integration};
}
struct Prepared {
  sc::PlanarWallGeometry wall;
  sc::Q4ParametricReference reference;
  sc::Q4ParametricConfig config=Config();
  fixture::Scratch scratch;
  bool Initialize(const sc::Q4SurfaceView& input,unsigned variant=0) {
    const auto mesh=geometry::Square(variant);
    return wall.Initialize(mesh.view()).status == sc::PlanarContactStatus::Ok &&
           reference.Initialize(input.positions,input.parents,input.parent_count).status == Code::Ok;
  }
  sc::Q4ParametricReport Evaluate(const sc::Q4SurfaceView& base,const sc::Q4SurfaceView& endpoint,
                                 sc::LumpedTranslationMassView mass,Result* output) {
    return sc::IntegrateQ4ParametricContact(wall,reference,base,endpoint,mass,config,7,scratch.view(),output);
  }
};
void SetPoint(Single* target,unsigned natural,sc::Vec3 p) {
  const auto n=target->parent.nodes[natural];
  target->position[n]=p.x; target->position[n+4]=p.y; target->position[n+8]=p.z;
}
sc::Vec3 Point(const Single& target,unsigned natural) {
  return target.Input().surface.positions.at(target.parent.nodes[natural]);
}
void Same(sc::Vec3 a,sc::Vec3 b) { EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.y,b.y); EXPECT_EQ(a.z,b.z); }
void SameValues(const sc::Q4IntegrationResult& a,const sc::Q4IntegrationResult& b) {
  EXPECT_EQ(a.resultant.value,b.resultant.value); EXPECT_EQ(a.potential.value,b.potential.value);
  for (unsigned n=0;n<4;++n) {
    EXPECT_EQ(a.nodal.nodes[n],b.nodal.nodes[n]); Same(a.nodal.forces[n],b.nodal.forces[n]);
    Same(a.nodal.couples[n],b.nodal.couples[n]);
  }
}
void Encloses(sc::Q4CertifiedIntegral cert,long double value) {
  EXPECT_LE(static_cast<long double>(cert.lower),value); EXPECT_GE(static_cast<long double>(cert.upper),value);
  EXPECT_GE(static_cast<long double>(cert.error),std::abs(static_cast<long double>(cert.value)-value));
}
long double CenterArea(const Single& x) {
  long double du[3]{},dv[3]{};
  constexpr int su[4]={1,-1,-1,1},sv[4]={1,1,-1,-1};
  for (unsigned n=0;n<4;++n) {
    const auto p=Point(x,n); const double c[3]={p.x,p.y,p.z};
    for (unsigned i=0;i<3;++i) { du[i]+=su[n]*static_cast<long double>(c[i])/4; dv[i]+=sv[n]*static_cast<long double>(c[i])/4; }
  }
  const long double g[3]={du[1]*dv[2]-du[2]*dv[1],du[2]*dv[0]-du[0]*dv[2],du[0]*dv[1]-du[1]*dv[0]};
  return 4*::sqrtl(g[0]*g[0]+g[1]*g[1]+g[2]*g[2]);
}

TEST(Q4ParametricContact, NamedImmutableCenterAreaIsDistinctFromWarpedSurfaceIntegral) {
  Single source;
  const double u[4]={1,-1,-1,1},v[4]={1,1,-1,-1};
  for (unsigned n=0;n<4;++n) SetPoint(&source,n,{.125*u[n]*v[n],.5*u[n],.25*v[n]});
  sc::Q4ParametricReference reference;
  const auto view=source.Input().surface;
  ASSERT_EQ(reference.Initialize(view.positions,view.parents,1).status,Code::Ok);
  EXPECT_STREQ(sc::Q4CenterAreaContactModel,"center-area-uniform-natural-v1");
  Encloses(reference.parent(0).area,.5);
  EXPECT_EQ(reference.parent(0).area.value,.5);
  sc::Q4CertifiedIntegral center,corner;
  ASSERT_EQ(sc::EvaluateQ4MaterialDensity(reference.parent(0).intrinsic,0,0,&center),sc::SurfaceMeasureStatus::Ok);
  ASSERT_EQ(sc::EvaluateQ4MaterialDensity(reference.parent(0).intrinsic,1,1,&corner),sc::SurfaceMeasureStatus::Ok);
  EXPECT_GT(corner.lower,center.upper); // Uniform natural measure is a named model, not this varying density.
  const auto before=fixture::Bytes(reference);
  source.position[0]+=1;
  EXPECT_EQ(fixture::Bytes(reference),before);
  EXPECT_EQ(reference.Initialize(view.positions,view.parents,1).status,Code::InvalidInput);
  EXPECT_EQ(fixture::Bytes(reference),before);
}

TEST(Q4ParametricContact, FlatOverlapRetainsRawC2AndLegacyPrescribedForceValues) {
  Single source; fixture::PhysicalMass<4> mass; Prepared prepared; fixture::Prepared legacy;
  ASSERT_TRUE(prepared.Initialize(source.Input().surface));
  ASSERT_TRUE(legacy.Initialize(geometry::Square(),source.Input().surface));
  const double gaps[3][4]={{.03125,.03125,.03125,.03125},{.02,.03,.04,.01},{.03125,-.03125,-.03125,.03125}};
  for (const auto& gap:gaps) {
    Single endpoint=source; endpoint.Gaps(gap[0],gap[1],gap[2],gap[3]);
    Result result; sc::Q4PrescribedPlanarResult old;
    ASSERT_EQ(prepared.Evaluate(source.Input().surface,endpoint.Input().surface,mass.view(),&result).status,Code::Ok);
    ASSERT_EQ(legacy.Evaluate(source.Input().surface,endpoint.Input().surface,mass.view(),&old).status,
              sc::Q4PrescribedPlanarStatus::Ok);
    SameValues(result.parents[0].integration,old.parents[0].integration);
    const sc::Q4PrescribedNormalIntegrationInput input{endpoint.Input().surface,mass.view(),0,7,0,1,16,.125};
    sc::Q4RectangularResult raw;
    ASSERT_EQ(sc::IntegrateQ4NormalContactRectangular(input,prepared.config.integration,prepared.scratch.view(),&raw).status,
              sc::Q4IntegrationStatus::Ok);
    SameValues(result.parents[0].integration,raw.integration);
    EXPECT_LE(result.parents[0].integration.resultant.error,prepared.config.integration.force_error);
    EXPECT_LE(result.parents[0].integration.potential.error,prepared.config.integration.energy_error);
  }
}

TEST(Q4ParametricContact, ActualMixedProjectionSourceQuadUsesItsUnmodifiedCenterArea) {
  Single source;
  const sc::Vec3 points[4]={{-.36290677,.42733408,.41496219},{-.36980676,.42903571,.42858191},
    {-.38900333000000004,.42824298,.42051611000000005},{-.37343292000000006,.42528113,.40896570000000004}};
  for (unsigned n=0;n<4;++n) SetPoint(&source,n,points[n]);
  source.parent.parent_element_id=2126280;
  Prepared prepared; fixture::PhysicalMass<4> mass;
  ASSERT_TRUE(prepared.Initialize(source.Input().surface));
  const auto& saved=prepared.reference.parent(0); Encloses(saved.area,CenterArea(source));
  bool positive=false,negative=false;
  for (unsigned n=0;n<4;++n) {
    positive=positive || saved.intrinsic.nominal_corner(n).x > 0;
    negative=negative || saved.intrinsic.nominal_corner(n).x < 0;
  }
  EXPECT_TRUE(positive && negative);
  Single endpoint=source; endpoint.Gaps(.03125,.03125,.03125,.03125);
  Result result;
  ASSERT_EQ(prepared.Evaluate(source.Input().surface,endpoint.Input().surface,mass.view(),&result).status,Code::Ok);
  const auto& force=result.parents[0].integration;
  Encloses(force.resultant,16*CenterArea(source)*.03125L);
  Encloses(force.potential,8*CenterArea(source)*.03125L*.03125L);
  EXPECT_EQ(force.parent_element_id,2126280u);
  for (unsigned n=0;n<4;++n) {
    EXPECT_EQ(force.nodal.nodes[n],source.parent.nodes[n]); Same(force.nodal.couples[n],{});
    EXPECT_EQ(saved.intrinsic.position(n).x,points[n].x);
  }
}

TEST(Q4ParametricContact, ReferenceRigidRotationCyclicOrderAndReversalPreservePhysicalNodalForces) {
  Single source,endpoint; endpoint.Gaps(.01,.02,.03,.04);
  fixture::PhysicalMass<4> mass; Prepared baseline;
  ASSERT_TRUE(baseline.Initialize(source.Input().surface));
  Result expected;
  ASSERT_EQ(baseline.Evaluate(endpoint.Input().surface,endpoint.Input().surface,mass.view(),&expected).status,Code::Ok);
  std::array<double,4> by_node{};
  for (unsigned n=0;n<4;++n) by_node[expected.parents[0].integration.nodal.nodes[n]]=expected.parents[0].integration.nodal.forces[n].x;
  for (unsigned shift=0;shift<8;++shift) {
    SCOPED_TRACE(shift);
    Single transformed=source,current=endpoint;
    for (unsigned n=0;n<4;++n) {
      const auto p=Point(source,n); SetPoint(&transformed,n,{p.z+2,p.x-3,p.y+.5});
    }
    for (unsigned n=0;n<4;++n) {
      const auto index=shift < 4 ? (n+shift)%4 : (4+shift-n)%4;
      transformed.parent.nodes[n]=source.parent.nodes[index]; current.parent.nodes[n]=source.parent.nodes[index];
    }
    Prepared prepared; ASSERT_TRUE(prepared.Initialize(transformed.Input().surface));
    Encloses(prepared.reference.parent(0).area,1);
    Result actual;
    ASSERT_EQ(prepared.Evaluate(current.Input().surface,current.Input().surface,mass.view(),&actual).status,Code::Ok);
    EXPECT_NEAR(actual.parents[0].integration.potential.value,expected.parents[0].integration.potential.value,1e-16);
    for (unsigned n=0;n<4;++n) {
      const auto& integral=actual.parents[0].integration;
      EXPECT_NEAR(integral.nodal.forces[n].x,by_node[integral.nodal.nodes[n]],2e-15);
    }
  }
}

TEST(Q4ParametricContact, CurrentProjectedAreaAndEdgeOnOrientationDoNotChangeTheFrozenMeasure) {
  Single source,endpoint; endpoint.Gaps(.0390625,.0234375,.0234375,.0390625);
  fixture::PhysicalMass<4> mass; Prepared prepared;
  ASSERT_TRUE(prepared.Initialize(source.Input().surface));
  Result expected;
  ASSERT_EQ(prepared.Evaluate(endpoint.Input().surface,endpoint.Input().surface,mass.view(),&expected).status,Code::Ok);
  auto edge_on=endpoint;
  for (unsigned n=0;n<4;++n) {
    const auto p=Point(endpoint,n); SetPoint(&edge_on,n,{p.x,p.z,0});
    edge_on.velocity[3*n+1]=n+.5; edge_on.velocity[3*n+2]=-static_cast<double>(n)-.25;
  }
  Result result;
  ASSERT_EQ(prepared.Evaluate(endpoint.Input().surface,edge_on.Input().surface,mass.view(),&result).status,Code::Ok);
  SameValues(result.parents[0].integration,expected.parents[0].integration);
  EXPECT_EQ(prepared.reference.parent(0).area.value,1);
  ASSERT_EQ(prepared.Evaluate(edge_on.Input().surface,edge_on.Input().surface,mass.view(),&result).status,Code::Ok);
  EXPECT_EQ(result.coverage[0].physical.minimum.z,0); EXPECT_EQ(result.coverage[0].physical.maximum.z,0);
  EXPECT_GT(result.coverage[0].upper_expansion_upper.z,0);
  SameValues(result.parents[0].integration,expected.parents[0].integration);
}

TEST(Q4ParametricContact, AllCoordinateDerivativesCurrentMomentsAndPowerMatchTheDiscretePotential) {
  Single source,endpoint; fixture::TransformProjection(&source,.5,0,0,.25);
  endpoint=source; endpoint.Gaps(.02,.03,.04,.01);
  fixture::TransformProjection(&endpoint,.75,.125,-.25,1.125,.25,-.125);
  fixture::PhysicalMass<4> mass; Prepared prepared;
  ASSERT_TRUE(prepared.Initialize(source.Input().surface));
  Result result;
  ASSERT_EQ(prepared.Evaluate(endpoint.Input().surface,endpoint.Input().surface,mass.view(),&result).status,Code::Ok);
  const auto& integral=result.parents[0].integration;
  for (double h:{1e-5,5e-6}) for (unsigned axis=0;axis<3;++axis) for (unsigned n=0;n<4;++n) {
    auto plus=endpoint,minus=endpoint; const auto node=endpoint.parent.nodes[n];
    plus.position[node+4*axis]+=h; minus.position[node+4*axis]-=h;
    Result a,b;
    ASSERT_EQ(prepared.Evaluate(plus.Input().surface,plus.Input().surface,mass.view(),&a).status,Code::Ok);
    ASSERT_EQ(prepared.Evaluate(minus.Input().surface,minus.Input().surface,mass.view(),&b).status,Code::Ok);
    const auto& f=integral.nodal.forces[n]; const double expected=axis == 0 ? -f.x : 0;
    const double derivative=(a.parents[0].integration.potential.value-b.parents[0].integration.potential.value)/(2*h);
    EXPECT_NEAR(derivative,expected,2e-12);
    // Keep the nominal check above, and independently propagate BOTH energy
    // truth intervals through the divided difference using outward arithmetic.
    // Multiplication by2 is exact for these finite normal binary64 steps.
    const auto& ua=a.parents[0].integration.potential;
    const auto& ub=b.parents[0].integration.potential;
    sc::Q4IntegralInterval lower_difference,upper_difference;
    ASSERT_TRUE(sc::q4_bounds::Difference(ua.lower,ub.upper,&lower_difference));
    ASSERT_TRUE(sc::q4_bounds::Difference(ua.upper,ub.lower,&upper_difference));
    double derivative_lower=0,derivative_upper=0;
    ASSERT_TRUE(sc::q4_bounds::Round(lower_difference.lower/(2*h),false,&derivative_lower));
    ASSERT_TRUE(sc::q4_bounds::Round(upper_difference.upper/(2*h),true,&derivative_upper));
    sc::Q4IntegralInterval energy_error_sum;
    ASSERT_TRUE(sc::q4_bounds::Add({ua.error,ua.error},{ub.error,ub.error},&energy_error_sum));
    double propagated_error=0;
    ASSERT_TRUE(sc::q4_bounds::Round(energy_error_sum.upper/(2*h),true,&propagated_error));
    EXPECT_LE(propagated_error,2e-12); // Predeclared N budget, not a relaxed integral tolerance.
    // Tensor product of the exact linear-shape Gram matrices gives int Ni*Nj.
    // For a quadratic energy, the exact represented-coordinate divided
    // difference equals its derivative at the endpoint midpoint, times the
    // represented displacement/(2h). This accounts for x +/- h rounding.
    long double analytic=0;
    if (axis==0) {
      constexpr int us[4]={1,-1,-1,1},vs[4]={1,1,-1,-1};
      for (unsigned j=0;j<4;++j) {
        const auto other=endpoint.parent.nodes[j];
        const long double g=(static_cast<long double>(plus.position[other])+minus.position[other])*.5L;
        const int coefficient=(us[n]==us[j] ? 2 : 1)*(vs[n]==vs[j] ? 2 : 1);
        analytic+=coefficient*g;
      }
      analytic*=16*CenterArea(source)/36;
      analytic*=(static_cast<long double>(plus.position[node])-minus.position[node])/(2*h);
    }
    EXPECT_LE(static_cast<long double>(derivative_lower),analytic);
    EXPECT_GE(static_cast<long double>(derivative_upper),analytic);
    EXPECT_LE(std::abs(analytic-expected),2e-12L);
  }
  sc::Vec3 nodal_moment,quadrature_moment; double nodal_power=0,quadrature_power=0;
  for (unsigned n=0;n<4;++n) {
    const auto node=integral.nodal.nodes[n];
    nodal_moment=sc::Add(nodal_moment,sc::geometry_detail::Cross(endpoint.Input().surface.positions.at(node),integral.nodal.forces[n]));
    nodal_power+=sc::Dot(endpoint.Input().surface.velocities.at(node),integral.nodal.forces[n]);
  }
  // Independent 2x2 Gauss oracle: x, velocity and each moment coordinate are
  // bilinear, so the fully active force/moment/work products are integrated exactly.
  const double q=1/std::sqrt(3.);
  for (double u:{-q,q}) for (double v:{-q,q}) {
    sc::Q4PointKinematics point;
    ASSERT_EQ(sc::EvaluateQ4Point(endpoint.Input().surface,{0,u,v},&point),sc::Status::kOk);
    const sc::Vec3 force{-16*.125*.25*point.position.x,0,0};
    quadrature_moment=sc::Add(quadrature_moment,sc::geometry_detail::Cross(point.position,force));
    quadrature_power+=sc::Dot(point.velocity,force);
  }
  EXPECT_NEAR(nodal_moment.y,quadrature_moment.y,1e-15); EXPECT_NEAR(nodal_moment.z,quadrature_moment.z,1e-15);
  EXPECT_NEAR(nodal_power,quadrature_power,1e-15);
}

TEST(Q4ParametricContact, SharedPhysicalNodesAndDistinctActivationIgnoreWallTessellation) {
  geometry::Pair source,endpoint; fixture::PhysicalMass<6> mass;
  // Nonuniform normal field with different active sets in the two parents.
  const double gaps[6]={-.03125,.03125,.03125,-.03125,.0625,-.0625};
  for (unsigned n=0;n<6;++n) { source.x[n]=0; endpoint.x[n]=gaps[n]; }
  std::array<double,6> expected{};
  for (unsigned variant=0;variant<4;++variant) {
    Prepared prepared; ASSERT_TRUE(prepared.Initialize(source.surface(),variant));
    Result result;
    ASSERT_EQ(prepared.Evaluate(source.surface(),endpoint.surface(),mass.view(),&result).status,Code::Ok);
    ASSERT_EQ(result.parent_count,2u);
    std::array<double,6> assembled{};
    for (const auto& parent:result.parents) for (unsigned n=0;n<4;++n)
      assembled[parent.integration.nodal.nodes[n]]+=parent.integration.nodal.forces[n].x;
    EXPECT_NE(result.parents[0].integration.resultant.value,result.parents[1].integration.resultant.value);
    if (variant == 0) expected=assembled;
    else EXPECT_EQ(assembled,expected);
    // Shared nodes2/3 occur in both true parent stencils, summed only once per contributor.
    EXPECT_LT(assembled[2],0); EXPECT_LT(assembled[3],0);
  }
}

TEST(Q4ParametricContact, AllParentPreflightAndLateSampleMassFailurePreserveOutputsAndRetry) {
  geometry::Pair source; for (unsigned n=0;n<6;++n) source.x[n]=.03125;
  fixture::PhysicalMass<6> mass; Prepared prepared;
  ASSERT_TRUE(prepared.Initialize(source.surface()));
  Result clean;
  ASSERT_EQ(prepared.Evaluate(source.surface(),source.surface(),mass.view(),&clean).status,Code::Ok);
  auto result=clean; const auto before=fixture::Bytes(result);
  prepared.scratch.leaves[0].bounds.integrals[0]={101,102};
  prepared.scratch.heap[0]=UINT32_MAX;
  const auto leaves_before=prepared.scratch.leaves; const auto heap_before=prepared.scratch.heap;
  for (unsigned end=0;end<2;++end) {
    auto base=source,candidate=source;
    (end == 0 ? base : candidate).x[4]=.25;
    const auto rejected=prepared.Evaluate(base.surface(),candidate.surface(),mass.view(),&result);
    EXPECT_EQ(rejected.status,Code::IntegrationFailure); EXPECT_EQ(rejected.parent,1u);
    EXPECT_EQ(rejected.node.endpoint,end); EXPECT_EQ(fixture::Bytes(result),before);
    EXPECT_EQ(std::memcmp(prepared.scratch.leaves.data(),leaves_before.data(),
                          leaves_before.size()*sizeof(sc::Q4RectangularCell)),0);
    EXPECT_EQ(prepared.scratch.heap,heap_before);
  }
  mass.inverse[4]=mass.inverse[5]=16*std::numeric_limits<double>::denorm_min();
  sc::NormalJacobian center;
  ASSERT_EQ(sc::BuildQ4NormalXJacobian(mass.view(),source.parents[1],0,0,7,&center),sc::Status::kOk);
  const auto report=prepared.Evaluate(source.surface(),source.surface(),mass.view(),&result);
  EXPECT_EQ(report.status,Code::IntegrationFailure); EXPECT_EQ(report.parent,1u);
  EXPECT_EQ(report.integration.status,sc::Q4IntegrationStatus::NonFiniteArithmetic);
  EXPECT_EQ(fixture::Bytes(result),before);
  mass.inverse[4]=mass.inverse[5]=1;
  ASSERT_EQ(prepared.Evaluate(source.surface(),source.surface(),mass.view(),&result).status,Code::Ok);
  for (unsigned p=0;p<2;++p) SameValues(result.parents[p].integration,clean.parents[p].integration);
}

TEST(Q4ParametricContact, FixedNodesForeignBindingsInvalidModelAndFailedReferenceRemainStaged) {
  Single source; source.Gaps(.03125,.03125,.03125,.03125);
  fixture::PhysicalMass<4> mass; Prepared prepared;
  ASSERT_TRUE(prepared.Initialize(source.Input().surface));
  Result result;
  ASSERT_EQ(prepared.Evaluate(source.Input().surface,source.Input().surface,mass.view(),&result).status,Code::Ok);
  const auto before=fixture::Bytes(result);
  for (unsigned fault=0;fault<6;++fault) {
    auto endpoint=source; auto altered=mass;
    switch (fault) {
      case 0: endpoint.parent.parent_element_id+=1; break;
      case 1: altered.fixed[0]=1; altered.inverse[0]=0; break; // Existing normal velocity is nonzero.
      case 2: altered.fixed[0]=6; break;
      case 3: endpoint.velocity[11]=HUGE_VAL; break;
      case 4: endpoint.position[4]=3; break; // Exposed finite-wall crossing.
      case 5: prepared.config.measure=static_cast<sc::Q4ReferenceContactMeasure>(9); break;
    }
    EXPECT_NE(prepared.Evaluate(source.Input().surface,endpoint.Input().surface,altered.view(),&result).status,Code::Ok);
    EXPECT_EQ(fixture::Bytes(result),before); prepared.config=Config();
  }
  auto fixed=mass; fixed.fixed[2]=1; fixed.inverse[2]=0;
  ASSERT_EQ(prepared.Evaluate(source.Input().surface,source.Input().surface,fixed.view(),&result).status,Code::Ok);
  EXPECT_LT(result.parents[0].integration.nodal.forces[2].x,0); // Physical support reaction retained.
  sc::PrescribedParentInterval interval;
  const auto view=source.Input().surface;
  ASSERT_EQ(sc::CheckPrescribedSurfaceInterval(view.positions,view.velocities,view.positions,view.velocities,
      mass.view(),source.parent.nodes,4,0,.125,&interval).status,sc::PrescribedNodeStatus::Ok);
  const auto interval_before=fixture::Bytes(interval);
  auto bad=source; bad.velocity[3*source.parent.nodes[3]+2]=HUGE_VAL;
  EXPECT_EQ(sc::CheckPrescribedSurfaceInterval(view.positions,view.velocities,bad.Input().surface.positions,
      bad.Input().surface.velocities,mass.view(),source.parent.nodes,4,0,.125,&interval).status,
      sc::PrescribedNodeStatus::InvalidInput);
  EXPECT_EQ(fixture::Bytes(interval),interval_before);
  sc::Q4ParametricReference rejected;
  const auto empty=fixture::Bytes(rejected); auto parent=source.parent; parent.nodes[3]=parent.nodes[0];
  EXPECT_NE(rejected.Initialize(source.Input().surface.positions,&parent,1).status,Code::Ok);
  EXPECT_EQ(fixture::Bytes(rejected),empty);
  std::array<sc::SurfaceQ4,2> duplicate{{source.parent,source.parent}};
  EXPECT_NE(rejected.Initialize(source.Input().surface.positions,duplicate.data(),2).status,Code::Ok);
  EXPECT_EQ(fixture::Bytes(rejected),empty);
  ASSERT_EQ(rejected.Initialize(source.Input().surface.positions,&source.parent,1).status,Code::Ok);
}
} // namespace
