#include "lib_src/collision/Q4IntegralMeasure.h"
#include "lib_src/collision/Q4RectangularIntegration.h"
#include "lib_utest/q4_prescribed_contact_fixture.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>

namespace {
namespace sc=tlfea::contact;
namespace fixture=q4_prescribed_test;
namespace geometry=q4_planar_test;
using Code=sc::Q4PrescribedPlanarStatus;
using Result=sc::Q4PrescribedPlanarResult;
using Single=geometry::Single;

void Same(sc::Vec3 a,sc::Vec3 b) { EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.y,b.y); EXPECT_EQ(a.z,b.z); }
void Same(sc::Q4IntegralInterval a,sc::Q4IntegralInterval b) { EXPECT_EQ(a.lower,b.lower); EXPECT_EQ(a.upper,b.upper); }
void Same(sc::Q4CertifiedIntegral a,sc::Q4CertifiedIntegral b) {
  EXPECT_EQ(a.value,b.value); EXPECT_EQ(a.lower,b.lower); EXPECT_EQ(a.upper,b.upper); EXPECT_EQ(a.error,b.error);
}
void Same(const Result& a,const Result& b) {
  ASSERT_EQ(a.geometry.parent_count,b.geometry.parent_count); EXPECT_EQ(a.valid,b.valid);
  EXPECT_EQ(a.geometry.valid,b.geometry.valid);
  for (unsigned p=0;p<a.geometry.parent_count;++p) {
    const auto& x=a.parents[p]; const auto& y=b.parents[p];
    EXPECT_EQ(x.deepest_u,y.deepest_u); EXPECT_EQ(x.deepest_v,y.deepest_v);
    const auto& i=x.integration; const auto& j=y.integration;
    EXPECT_EQ(i.valid,j.valid); EXPECT_EQ(i.feature_id,j.feature_id); EXPECT_EQ(i.parent_element_id,j.parent_element_id);
    EXPECT_EQ(i.base_epoch,j.base_epoch); EXPECT_EQ(i.attempt,j.attempt);
    EXPECT_EQ(i.leaf_count,j.leaf_count); EXPECT_EQ(i.visited,j.visited); EXPECT_EQ(i.deepest_leaf,j.deepest_leaf);
    Same(i.resultant,j.resultant); Same(i.potential,j.potential); Same(i.active_area,j.active_area);
    for (unsigned n=0;n<4;++n) {
      EXPECT_EQ(i.nodal.nodes[n],j.nodal.nodes[n]); Same(i.nodal.forces[n],j.nodal.forces[n]);
      Same(i.nodal.couples[n],j.nodal.couples[n]); Same(i.force[n],j.force[n]);
    }
    const auto& g=a.geometry.parents[p]; const auto& h=b.geometry.parents[p];
    Same(g.projected_minimum,h.projected_minimum); Same(g.projected_maximum,h.projected_maximum);
    Same(g.projected_jacobian,h.projected_jacobian); Same(g.projected_jacobian_ratio,h.projected_jacobian_ratio);
    EXPECT_EQ(g.required_jacobian_lower,h.required_jacobian_lower);
    EXPECT_EQ(g.reference_material_area,h.reference_material_area);
    Same(g.reference_material_area_enclosure,h.reference_material_area_enclosure);
  }
}
void Encloses(sc::Q4CertifiedIntegral result,long double expected) {
  EXPECT_LE(static_cast<long double>(result.lower),expected);
  EXPECT_GE(static_cast<long double>(result.upper),expected);
  EXPECT_LE(std::abs(static_cast<long double>(result.value)-expected),static_cast<long double>(result.error));
}
long double ReferenceArea(const sc::PreparedQ4PlanarParent& parent) {
  const auto* p=parent.reference_projection;
  return (static_cast<long double>(p[0].y)-p[1].y)*(static_cast<long double>(p[0].z)-p[3].z);
}
struct ActiveOracle { std::array<long double,4> force{}; long double potential=0; };
ActiveOracle Active(const Single& surface,long double area,long double stiffness) {
  // Closed-form integrals of the four Q4 shape products over the material
  // rectangle. Independent of adaptive cells, Gauss samples and mass values.
  constexpr unsigned coefficient[4][4]={{4,2,1,2},{2,4,2,1},{1,2,4,2},{2,1,2,4}};
  ActiveOracle result;
  for (unsigned i=0;i<4;++i) for (unsigned j=0;j<4;++j) {
    const long double gi=surface.position[surface.parent.nodes[i]],gj=surface.position[surface.parent.nodes[j]];
    result.force[i]-=stiffness*area*coefficient[i][j]*gj/36;
    result.potential+=stiffness*area*coefficient[i][j]*gi*gj/72;
  }
  return result;
}

TEST(Q4PrescribedPlanarContact, GeometryOnlyReferenceDoesNotInventInitialConstraintsOrMass) {
  Single reference;
  for (unsigned n=0;n<4;++n) { reference.velocity[3*n+1]=.25; reference.velocity[3*n+2]=-.5; }
  fixture::Prepared prepared;
  ASSERT_TRUE(prepared.Initialize(geometry::Square(),reference.Input().surface));
  sc::Q4PlanarGeometry legacy;
  EXPECT_EQ(legacy.Initialize(prepared.wall,reference.Input().surface,reference.Input().mass,geometry::Clearance).status,
            sc::PlanarContactStatus::UnsupportedMotion);
  EXPECT_FALSE(legacy.initialized());
  EXPECT_EQ(prepared.geometry.view().parents[0].projected_area,1);
  auto malformed=reference; malformed.velocity[3*2+2]=std::numeric_limits<double>::quiet_NaN();
  sc::Q4PlanarGeometry fresh;
  EXPECT_EQ(fresh.InitializeReference(prepared.wall,malformed.Input().surface,geometry::Clearance).status,
            sc::PlanarContactStatus::InvalidInput);
  EXPECT_FALSE(fresh.initialized());
  EXPECT_EQ(fresh.InitializeReference(prepared.wall,reference.Input().surface,geometry::Clearance).status,sc::PlanarContactStatus::Ok);
  const auto saved=fresh.view().parents[0];
  reference.position[reference.parent.nodes[0]+4]+=1;
  EXPECT_EQ(fresh.InitializeReference(prepared.wall,reference.Input().surface,geometry::Clearance).status,
            sc::PlanarContactStatus::InvalidInput);
  EXPECT_EQ(fresh.view().parents[0].reference_projection[0].y,saved.reference_projection[0].y);
}

TEST(Q4PrescribedPlanarContact, NonunitMovingFootprintKeepsMaterialMeasureAndExactSameNormalIntegral) {
  Single reference; fixture::TransformProjection(&reference,.5,0,0,.25);
  fixture::Prepared prepared; fixture::PhysicalMass<4> mass;
  ASSERT_TRUE(prepared.Initialize(geometry::Square(),reference.Input().surface));
  Single unmoved=reference; unmoved.Gaps(.03125,.03125,.03125,.03125);
  Single moved=unmoved; fixture::TransformProjection(&moved,.875,-.25,.25,.875,.125,-.0625);
  for (unsigned n=0;n<4;++n) { moved.velocity[3*n+1]=n+.125; moved.velocity[3*n+2]=-.25-n; }
  Result baseline,result;
  ASSERT_EQ(prepared.Evaluate(reference.Input().surface,unmoved.Input().surface,mass.view(),&baseline).status,Code::Ok);
  ASSERT_EQ(prepared.Evaluate(reference.Input().surface,moved.Input().surface,mass.view(),&result).status,Code::Ok);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.geometry.parent_count,1u);
  EXPECT_EQ(result.geometry.parents[0].reference_material_area,.125);
  EXPECT_NE(result.geometry.parents[0].projected_jacobian.lower,.125/4);
  const auto& integral=result.parents[0].integration;
  Same(integral.resultant,baseline.parents[0].integration.resultant);
  Same(integral.potential,baseline.parents[0].integration.potential);
  for (unsigned n=0;n<4;++n) {
    Same(integral.force[n],baseline.parents[0].integration.force[n]);
    Encloses(integral.force[n],16.L*.125L*.03125L/4);
    Same(integral.nodal.couples[n],{});
  }
  Encloses(integral.potential,16.L*.125L*.03125L*.03125L/2);
  EXPECT_EQ(integral.base_epoch,mass.view().base_epoch); EXPECT_EQ(integral.attempt,7u);
}

TEST(Q4PrescribedPlanarContact, AllTranslationDerivativesAndCurrentMomentsMatchReferenceMeasurePotential) {
  Single reference; fixture::TransformProjection(&reference,.5,0,0,.25);
  fixture::Prepared prepared; fixture::PhysicalMass<4> mass;
  ASSERT_TRUE(prepared.Initialize(geometry::Square(),reference.Input().surface));
  Single current=reference;
  fixture::TransformProjection(&current,.875,-.25,.25,.875,.125,-.0625);
  current.position[current.parent.nodes[0]+4]+=.03125;  // Nonaffine current projected map.
  current.Gaps(.03125,.046875,.015625,.0625);
  for (unsigned n=0;n<4;++n) {
    current.velocity[3*n]=.25+n; current.velocity[3*n+1]=-2.+n; current.velocity[3*n+2]=3.-n;
  }
  Result result;
  ASSERT_EQ(prepared.Evaluate(reference.Input().surface,current.Input().surface,mass.view(),&result).status,Code::Ok);
  const auto& integral=result.parents[0].integration;
  const auto oracle=Active(current,ReferenceArea(prepared.geometry.view().parents[0]),16);
  Encloses(integral.potential,oracle.potential);
  sc::Vec3 moment; long double expected_y=0,expected_z=0,power=0,expected_power=0;
  long double moment_error_y=0,moment_error_z=0,power_error=0;
  for (unsigned local=0;local<4;++local) {
    Encloses(integral.force[local],-oracle.force[local]);
    const auto node=current.parent.nodes[local];
    const auto x=current.Input().surface.positions.at(node);
    const auto v=current.Input().surface.velocities.at(node);
    moment=sc::Add(moment,sc::geometry_detail::Cross(x,integral.nodal.forces[local]));
    expected_y+=x.z*oracle.force[local]; expected_z-=x.y*oracle.force[local];
    moment_error_y+=std::abs(x.z)*integral.force[local].error;
    moment_error_z+=std::abs(x.y)*integral.force[local].error;
    power+=sc::Dot(integral.nodal.forces[local],v); expected_power+=v.x*oracle.force[local];
    power_error+=std::abs(v.x)*integral.force[local].error;
    EXPECT_EQ(integral.nodal.forces[local].y,0); EXPECT_EQ(integral.nodal.forces[local].z,0);
    Same(integral.nodal.couples[local],{});
  }
  EXPECT_EQ(moment.x,0);
  EXPECT_LE(std::abs(moment.y-expected_y),moment_error_y+128*DBL_EPSILON*std::abs(expected_y));
  EXPECT_LE(std::abs(moment.z-expected_z),moment_error_z+128*DBL_EPSILON*std::abs(expected_z));
  EXPECT_LE(std::abs(power-expected_power),power_error+128*DBL_EPSILON*std::abs(expected_power));
  for (unsigned refinement=0;refinement<2;++refinement) for (unsigned local=0;local<4;++local) for (unsigned axis=0;axis<3;++axis) {
    SCOPED_TRACE(refinement);
    SCOPED_TRACE(local);
    SCOPED_TRACE(axis);
    const double delta=std::ldexp(1.,-18-static_cast<int>(refinement));
    const auto coordinate=current.parent.nodes[local]+4*axis;
    Single plus=current,minus=current; plus.position[coordinate]+=delta; minus.position[coordinate]-=delta;
    Result positive,negative;
    ASSERT_EQ(prepared.Evaluate(current.Input().surface,plus.Input().surface,mass.view(),&positive).status,Code::Ok);
    ASSERT_EQ(prepared.Evaluate(current.Input().surface,minus.Input().surface,mass.view(),&negative).status,Code::Ok);
    const auto a=positive.parents[0].integration.potential,b=negative.parents[0].integration.potential;
    if (axis) {
      EXPECT_EQ(a.value,integral.potential.value); EXPECT_EQ(b.value,integral.potential.value);
    } else {
      const long double derivative=(static_cast<long double>(a.value)-b.value)/(2*delta);
      const long double arithmetic=128*LDBL_EPSILON*(std::abs(a.value)+std::abs(b.value))/(2*delta);
      const long double budget=(static_cast<long double>(a.error)+b.error)/(2*delta)+integral.force[local].error+arithmetic;
      EXPECT_LE(std::abs(derivative+integral.nodal.forces[local].x),budget);
    }
  }
}

TEST(Q4PrescribedPlanarContact, ActualFullyFixedNodesCannotMoveInAnyCoordinateOrVelocity) {
  Single reference; fixture::Prepared prepared; fixture::PhysicalMass<4> mass;
  mass.fixed[2]=1; mass.inverse[2]=0;
  ASSERT_TRUE(prepared.Initialize(geometry::Square(),reference.Input().surface));
  Single endpoint=reference; endpoint.Gaps(.03125,.03125,0,.03125);
  fixture::TransformProjection(&endpoint,.875,-.125,.125,.875,-.125,0);  // Pivot about physical node2.
  Result clean;
  ASSERT_EQ(prepared.Evaluate(reference.Input().surface,endpoint.Input().surface,mass.view(),&clean).status,Code::Ok);
  for (unsigned fault=0;fault<7;++fault) {
    SCOPED_TRACE(fault);
    auto base=reference,current=endpoint;
    if (fault < 3) current.position[2+4*fault]+=.0001;
    else if (fault < 6) current.velocity[3*2+fault-3]=.0001;
    else base.velocity[3*2+1]=.0001;
    auto result=clean; const auto before=fixture::Bytes(result);
    EXPECT_EQ(prepared.Evaluate(base.Input().surface,current.Input().surface,mass.view(),&result).status,Code::FixedMotion);
    EXPECT_EQ(fixture::Bytes(result),before);
  }
  auto unsupported=mass.view(); unsupported.model=sc::TranslationMassModel::kUnspecified;
  auto result=clean; const auto before=fixture::Bytes(result);
  const auto report=prepared.Evaluate(reference.Input().surface,endpoint.Input().surface,unsupported,&result);
  EXPECT_EQ(report.status,Code::MassFailure); EXPECT_EQ(report.mass_status,sc::Status::kUnsupportedInterpolation);
  EXPECT_EQ(fixture::Bytes(result),before);
}

TEST(Q4PrescribedPlanarContact, SharedParentsAndFiniteWallTessellationPreservePartialContactAndCurrentMoments) {
  const long double coefficient[6]={5.L/288,13.L/288,43.L/288,11.L/288,30.L/288,6.L/288};
  Result baseline;
  for (unsigned variant=0;variant<4;++variant) {
    SCOPED_TRACE(variant);
    geometry::Pair base,current; fixture::PhysicalMass<6> mass; fixture::Prepared prepared;
    ASSERT_TRUE(prepared.Initialize(geometry::Square(variant),base.surface()));
    for (unsigned node=0;node<6;++node) {
      current.x[node+6]+=.125; current.x[node+12]-=.125;
      current.v[3*node+1]=.25; current.v[3*node+2]=-.125;
    }
    Result result; ASSERT_EQ(prepared.Evaluate(base.surface(),current.surface(),mass.view(),&result).status,Code::Ok);
    ASSERT_EQ(result.geometry.parent_count,2u);
    std::array<long double,6> force{},error{};
    for (unsigned p=0;p<2;++p) {
      const auto& integral=result.parents[p].integration;
      EXPECT_EQ(integral.parent_element_id,base.parents[p].parent_element_id);
      EXPECT_GT(integral.active_area.lower,0); EXPECT_LT(integral.active_area.upper,1);
      for (unsigned n=0;n<4;++n) {
        force[integral.nodal.nodes[n]]+=integral.nodal.forces[n].x;
        error[integral.nodal.nodes[n]]+=integral.force[n].error;
      }
    }
    long double moment_y=0,moment_z=0,expected_y=0,expected_z=0,error_y=0,error_z=0;
    for (unsigned node=0;node<6;++node) {
      const long double expected=-16.L*geometry::Depth*coefficient[node];
      EXPECT_LE(std::abs(force[node]-expected),error[node]);
      const auto x=current.surface().positions.at(node);
      moment_y+=x.z*force[node]; moment_z-=x.y*force[node];
      expected_y+=x.z*expected; expected_z-=x.y*expected;
      error_y+=std::abs(x.z)*error[node]; error_z+=std::abs(x.y)*error[node];
    }
    EXPECT_LE(std::abs(moment_y-expected_y),error_y+128*LDBL_EPSILON*std::abs(expected_y));
    EXPECT_LE(std::abs(moment_z-expected_z),error_z+128*LDBL_EPSILON*std::abs(expected_z));
    if (variant == 0) baseline=result;
    else Same(result,baseline);
  }
}

TEST(Q4PrescribedPlanarContact, ExactCoordinateReferenceAreaExpandsCertificatesAndRechecksOriginalBudgets) {
  Single reference; fixture::TransformProjection(&reference,.2,0,0,.3);
  fixture::Prepared prepared; fixture::PhysicalMass<4> mass;
  ASSERT_TRUE(prepared.Initialize(geometry::Square(),reference.Input().surface));
  Single endpoint=reference; endpoint.Gaps(.03125,.03125,.03125,.03125);
  fixture::TransformProjection(&endpoint,1.25,0,0,.75,.125,0);
  Result result; ASSERT_EQ(prepared.Evaluate(reference.Input().surface,endpoint.Input().surface,mass.view(),&result).status,Code::Ok);
  const auto& saved=prepared.geometry.view().parents[0]; const long double area=ReferenceArea(saved);
  EXPECT_LT(saved.area_enclosure.lower,saved.area_enclosure.upper);
  const auto& integral=result.parents[0].integration;
  for (const auto& force:integral.force) Encloses(force,16.L*area*.03125L/4);
  Encloses(integral.potential,16.L*area*.03125L*.03125L/2);
  EXPECT_EQ(result.geometry.parents[0].reference_material_area,saved.projected_area);
  sc::Q4PrescribedNormalIntegrationInput input{endpoint.Input().surface,mass.view(),0,7,0,saved.projected_area,16,.125};
  sc::Q4RectangularResult raw;
  ASSERT_EQ(sc::IntegrateQ4NormalContactRectangular(input,prepared.config.integration,prepared.scratch.view(),&raw).status,
            sc::Q4IntegrationStatus::Ok);
  double raw_error=raw.integration.resultant.error;
  for (const auto& force:raw.integration.force) raw_error=std::max(raw_error,force.error);
  ASSERT_GT(raw_error,0);
  ASSERT_GT(integral.resultant.error,raw_error);
  for (unsigned n=0;n<4;++n) EXPECT_EQ(integral.force[n].value,raw.integration.force[n].value);
  prepared.config.integration.force_error=raw_error;
  const auto before=fixture::Bytes(result);
  const auto rejected=prepared.Evaluate(reference.Input().surface,endpoint.Input().surface,mass.view(),&result);
  EXPECT_EQ(rejected.status,Code::IntegrationFailure);
  EXPECT_EQ(rejected.integration.status,sc::Q4IntegrationStatus::UnattainableAccuracy);
  EXPECT_EQ(fixture::Bytes(result),before);
}

TEST(Q4PrescribedPlanarContact, BothEndpointCapsAndGeometryArePreflightedBeforeScratchIntegration) {
  geometry::Pair base; fixture::Prepared prepared; fixture::PhysicalMass<6> mass;
  ASSERT_TRUE(prepared.Initialize(geometry::Square(),base.surface()));
  Result result;
  ASSERT_EQ(prepared.Evaluate(base.surface(),base.surface(),mass.view(),&result).status,Code::Ok);
  const auto before=fixture::Bytes(result);
  const auto leaf_before=prepared.scratch.leaves;
  const auto heap_before=prepared.scratch.heap;
  for (unsigned end=0;end<2;++end) {
    auto first=base,second=base;
    (end == 0 ? first : second).x[4]=.25;
    const auto report=prepared.Evaluate(first.surface(),second.surface(),mass.view(),&result);
    EXPECT_EQ(report.status,Code::IntegrationFailure); EXPECT_EQ(report.parent,1u);
    EXPECT_EQ(report.integration.status,sc::Q4IntegrationStatus::PenetrationLimit);
    EXPECT_EQ(fixture::Bytes(result),before);
    EXPECT_EQ(std::memcmp(prepared.scratch.leaves.data(),leaf_before.data(),leaf_before.size()*sizeof(sc::Q4RectangularCell)),0);
    EXPECT_EQ(prepared.scratch.heap,heap_before);
  }
  auto outside=base; for (unsigned n=0;n<6;++n) outside.x[n+6]+=4;
  EXPECT_EQ(prepared.Evaluate(base.surface(),outside.surface(),mass.view(),&result).status,Code::GeometryFailure);
  EXPECT_EQ(fixture::Bytes(result),before);
  EXPECT_EQ(std::memcmp(prepared.scratch.leaves.data(),leaf_before.data(),leaf_before.size()*sizeof(sc::Q4RectangularCell)),0);
  EXPECT_EQ(prepared.scratch.heap,heap_before);
}

TEST(Q4PrescribedPlanarContact, LateSecondParentGaussMassFailurePreservesResultAndCleanRetry) {
  geometry::Pair base; for (unsigned n=0;n<6;++n) base.x[n]=.03125;
  fixture::Prepared prepared; fixture::PhysicalMass<6> mass;
  ASSERT_TRUE(prepared.Initialize(geometry::Square(),base.surface()));
  Result clean; ASSERT_EQ(prepared.Evaluate(base.surface(),base.surface(),mass.view(),&clean).status,Code::Ok);
  auto result=clean; const auto before=fixture::Bytes(result);
  mass.inverse[4]=mass.inverse[5]=16*std::numeric_limits<double>::denorm_min();
  sc::NormalJacobian center;
  ASSERT_EQ(sc::BuildQ4NormalXJacobian(mass.view(),base.parents[1],0,0,7,&center),sc::Status::kOk);
  const auto report=prepared.Evaluate(base.surface(),base.surface(),mass.view(),&result);
  EXPECT_EQ(report.status,Code::IntegrationFailure); EXPECT_EQ(report.parent,1u);
  EXPECT_EQ(report.integration.status,sc::Q4IntegrationStatus::NonFiniteArithmetic);
  EXPECT_EQ(fixture::Bytes(result),before);
  mass.inverse[4]=mass.inverse[5]=1;
  ASSERT_EQ(prepared.Evaluate(base.surface(),base.surface(),mass.view(),&result).status,Code::Ok);
  Same(result,clean);
}
}  // namespace
