#include "lib_src/collision/Q4SurfaceMapping.h"
#include "lib_src/collision/Q4SurfaceMass.h"
#include "lib_src/collision/SurfaceContactGeometry.h"

#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <limits>
#include <type_traits>

namespace {
namespace sc = tlfea::contact;
constexpr double kTolerance = 2e-13;
constexpr double sign_u[4] = {1, -1, -1, 1};
constexpr double sign_v[4] = {1, 1, -1, -1};
static_assert(std::is_trivially_copyable<sc::Q4SurfaceView>::value);
static_assert(std::is_standard_layout<sc::Q4SurfaceView>::value);
static_assert(std::is_trivially_copyable<sc::Q4FixedYZMassView>::value);

sc::Vec3 Position(double u, double v) { return {2+3*u-4*v+.5*u*v, -1+2*u+.25*v, .7-u+5*v+2*u*v}; }
sc::Vec3 Velocity(double u, double v) { return {-3+2*u+4*v-5*u*v, 7-u+2*u*v, 1+.5*v+.25*u*v}; }
struct Fixture {
  std::array<double, 18> position{}, velocity{};
  std::array<double, 6> inverse{{1, .25, .125, 1, .5, 0}};
  std::array<std::uint8_t, 6> fixed{{6, 6, 6, 6, 6, 7}};
  sc::SurfaceQ4 parent{{4, 1, 5, 2}, 73, 42, 2, 0};
  sc::Q4Point point{0, .2, -.4};
  Fixture() {
    for (unsigned local = 0; local < 4; ++local) {
      const auto n = parent.nodes[local];
      const auto x = Position(sign_u[local], sign_v[local]);
      const auto v = Velocity(sign_u[local], sign_v[local]);
      position[n] = x.x; position[n+6] = x.y; position[n+12] = x.z;
      velocity[3*n] = v.x; velocity[3*n+1] = v.y; velocity[3*n+2] = v.z;
    }
  }
  sc::Q4SurfaceView view() const { return {{position.data(),6,1,6}, {velocity.data(),6,3,1}, &parent,1}; }
  sc::Q4FixedYZMassView mass() const { return {inverse.data(), fixed.data(),6,9}; }
};
void Near(sc::Vec3 actual, sc::Vec3 expected, double tolerance = kTolerance) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}
void Same(const sc::Q4PointKinematics& a, const sc::Q4PointKinematics& b) {
  Near(a.position,b.position,0); Near(a.velocity,b.velocity,0);
  for (unsigned n=0;n<4;++n) EXPECT_EQ(a.shape[n],b.shape[n]);
}
void Same(const sc::Q4NodalForces& a, const sc::Q4NodalForces& b) {
  for (unsigned n=0;n<4;++n) {
    EXPECT_EQ(a.nodes[n],b.nodes[n]); Near(a.forces[n],b.forces[n],0); Near(a.couples[n],b.couples[n],0);
  }
}
void Same(const sc::NormalJacobian& a, const sc::NormalJacobian& b) {
  EXPECT_EQ(a.count,b.count); EXPECT_EQ(a.valid,b.valid); EXPECT_EQ(a.base_epoch,b.base_epoch); EXPECT_EQ(a.attempt,b.attempt);
  EXPECT_EQ(a.inverse_effective_mass,b.inverse_effective_mass);
  for (unsigned i=0;i<sc::kMaxNormalNodes;++i) {
    EXPECT_EQ(a.nodes[i],b.nodes[i]); Near(a.values[i],b.values[i],0); EXPECT_EQ(a.normalized_norm[i],b.normalized_norm[i]);
  }
}

TEST(Q4SurfaceMapping, NaturalCornersPartitionAndBilinearReproduction) {
  for (unsigned corner=0;corner<4;++corner) {
    double shape[4];
    ASSERT_EQ(sc::EvaluateQ4Shape(sign_u[corner],sign_v[corner],shape),sc::Status::kOk);
    for (unsigned n=0;n<4;++n) EXPECT_EQ(shape[n],n==corner?1:0);
  }
  for (double u:{-1.,-.75,-.2,0.,.6,1.}) for (double v:{-1.,-.25,.3,1.}) {
    double shape[4]; ASSERT_EQ(sc::EvaluateQ4Shape(u,v,shape),sc::Status::kOk);
    double sum=0,linear_u=0,linear_v=0,mixed=0;
    for (unsigned n=0;n<4;++n) {
      EXPECT_GE(shape[n],0); EXPECT_LE(shape[n],1);
      sum+=shape[n]; linear_u+=sign_u[n]*shape[n]; linear_v+=sign_v[n]*shape[n];
      mixed+=sign_u[n]*sign_v[n]*shape[n];
    }
    EXPECT_NEAR(sum,1,1e-15); EXPECT_NEAR(linear_u,u,1e-15); EXPECT_NEAR(linear_v,v,1e-15); EXPECT_NEAR(mixed,u*v,1e-15);
  }
}

TEST(Q4SurfaceMapping, NonuniformFieldsUsePhysicalIdsAndIndependentStrides) {
  Fixture f;
  // Unrelated owner nodes are intentionally invalid: this map reads only its
  // four declared physical nodes, not a contiguous local corner proxy.
  f.position[0]=std::numeric_limits<double>::quiet_NaN();
  for (double u:{-.73,0.,.2,.81}) for (double v:{-.41,.17,.6}) {
    sc::Q4PointKinematics value;
    ASSERT_EQ(sc::EvaluateQ4Point(f.view(),{0,u,v},&value),sc::Status::kOk);
    Near(value.position,Position(u,v)); Near(value.velocity,Velocity(u,v));
  }
  sc::Q4PointKinematics value;
  ASSERT_EQ(sc::EvaluateQ4Point(f.view(),f.point,&value),sc::Status::kOk);
  const double expected[4]={.18,.12,.28,.42};
  for(unsigned n=0;n<4;++n) EXPECT_NEAR(value.shape[n],expected[n],1e-15);
  EXPECT_EQ(f.parent.feature_id,73u); EXPECT_EQ(f.parent.parent_element_id,42u); EXPECT_EQ(f.parent.parent_face_id,2u);
}

TEST(Q4SurfaceMapping, TransposeScatterPreservesResultantMomentAndVirtualWork) {
  const Fixture f; const auto view=f.view(); const sc::Vec3 force{3,-4,9};
  sc::Q4PointKinematics point; sc::Q4NodalForces nodal;
  ASSERT_EQ(sc::EvaluateQ4Point(view,f.point,&point),sc::Status::kOk);
  ASSERT_EQ(sc::ProjectQ4PointForce(view,f.point,force,&nodal),sc::Status::kOk);
  sc::Vec3 resultant,moment; double power=0;
  for(unsigned n=0;n<4;++n) {
    EXPECT_EQ(nodal.nodes[n],f.parent.nodes[n]); Near(nodal.couples[n],{},0);
    resultant=sc::Add(resultant,nodal.forces[n]);
    moment=sc::Add(moment,sc::geometry_detail::Cross(view.positions.at(nodal.nodes[n]),nodal.forces[n]));
    power+=sc::Dot(view.velocities.at(nodal.nodes[n]),nodal.forces[n]);
  }
  Near(resultant,force); Near(moment,sc::geometry_detail::Cross(point.position,force));
  EXPECT_NEAR(power,sc::Dot(point.velocity,force),kTolerance);
  EXPECT_NE(nodal.forces[0].x,nodal.forces[1].x);  // No equal-corner/triangle scatter.
  auto moved=f;
  const sc::Vec3 dx[4]={{.003,-.001,.005},{0,.006,-.002},{-.004,.002,.001},{.001,-.007,.004}};
  double work=0;
  for(unsigned n=0;n<4;++n) {
    const auto global=f.parent.nodes[n];
    moved.position[global]+=dx[n].x; moved.position[global+6]+=dx[n].y; moved.position[global+12]+=dx[n].z;
    work+=sc::Dot(nodal.forces[n],dx[n]);
  }
  sc::Q4PointKinematics after;
  ASSERT_EQ(sc::EvaluateQ4Point(moved.view(),moved.point,&after),sc::Status::kOk);
  EXPECT_NEAR(work,sc::Dot(force,sc::Subtract(after.position,point.position)),kTolerance);
}

TEST(Q4SurfaceMapping, InvalidDomainTopologyOffsetAndViewsPreserveOutputs) {
  Fixture f; sc::Q4PointKinematics accepted; sc::Q4NodalForces forces;
  ASSERT_EQ(sc::EvaluateQ4Point(f.view(),f.point,&accepted),sc::Status::kOk);
  ASSERT_EQ(sc::ProjectQ4PointForce(f.view(),f.point,{3,4,5},&forces),sc::Status::kOk);
  for(unsigned variant=0;variant<13;++variant) {
    auto bad=f; auto point=f.point;
    if(variant==0) bad.parent.nodes[3]=bad.parent.nodes[0];
    if(variant==1) bad.parent.nodes[3]=6;
    if(variant==2) bad.parent.feature_id=0;
    if(variant==3) bad.parent.parent_element_id=0;
    if(variant==4) bad.parent.half_thickness=.001;
    if(variant==5) bad.parent.half_thickness=std::numeric_limits<double>::quiet_NaN();
    if(variant==6) point.u=std::nextafter(1.,2.);
    if(variant==7) point.v=std::numeric_limits<double>::infinity();
    if(variant==8) point.parent_index=1;
    auto view=bad.view();
    if(variant==9) view.velocities.node_count=5;
    if(variant==10) view.positions.data=nullptr;
    if(variant==11) view.parent_count=0;
    if(variant==12) view.parents=nullptr;
    auto out=accepted; auto projected=forces;
    EXPECT_NE(sc::EvaluateQ4Point(view,point,&out),sc::Status::kOk) << variant;
    EXPECT_NE(sc::ProjectQ4PointForce(view,point,{3,4,5},&projected),sc::Status::kOk) << variant;
    Same(out,accepted); Same(projected,forces);
  }
  double shape[4]={3,4,5,6};
  EXPECT_EQ(sc::EvaluateQ4Shape(0,std::nextafter(-1.,-2.),shape),sc::Status::kOutOfRange);
  for(unsigned i=0;i<4;++i) EXPECT_EQ(shape[i],3+i);
  EXPECT_EQ(sc::EvaluateQ4Point(f.view(),f.point,nullptr),sc::Status::kInvalidArgument);
  auto point_output=accepted;
  EXPECT_EQ(sc::EvaluateQ4Point({},f.point,&point_output),sc::Status::kInvalidArgument);
  Same(point_output,accepted);
  auto out=forces;
  EXPECT_EQ(sc::ProjectQ4PointForce(f.view(),f.point,{0,std::numeric_limits<double>::quiet_NaN(),0},&out),sc::Status::kInvalidArgument);
  Same(out,forces);
}

TEST(Q4SurfaceMapping, LateNonfiniteAndZeroWeightNodesStillRejectWithoutPublication) {
  Fixture f; sc::Q4PointKinematics before;
  ASSERT_EQ(sc::EvaluateQ4Point(f.view(),f.point,&before),sc::Status::kOk);
  for(const bool corner:{false,true}) {
    auto bad=f; bad.velocity[3*f.parent.nodes[3]+2]=std::numeric_limits<double>::quiet_NaN();
    auto result=before;
    EXPECT_EQ(sc::EvaluateQ4Point(bad.view(),corner?sc::Q4Point{0,1,1}:f.point,&result),sc::Status::kInvalidArgument);
    Same(result,before);
    ASSERT_EQ(sc::EvaluateQ4Point(f.view(),f.point,&result),sc::Status::kOk); Same(result,before);
  }
  auto invalid=f.view(); invalid.positions.component_stride=UINT64_MAX;
  auto result=before;
  EXPECT_EQ(sc::EvaluateQ4Point(invalid,f.point,&result),sc::Status::kInvalidArgument); Same(result,before);
}

TEST(Q4SurfaceMass, ProjectedNormalMassMatchesIndependentImpulseAndUpwardNorms) {
  Fixture f; f.velocity.fill(0);
  sc::NormalJacobian jacobian;
  ASSERT_EQ(sc::BuildQ4NormalXJacobian(f.mass(),f.parent,f.point.u,f.point.v,7,&jacobian),sc::Status::kOk);
  EXPECT_TRUE(jacobian.valid); EXPECT_EQ(jacobian.base_epoch,9u); EXPECT_EQ(jacobian.attempt,7u); EXPECT_EQ(jacobian.count,4u);
  EXPECT_NEAR(jacobian.inverse_effective_mass,.04185,1e-15);
  for(unsigned entry=0;entry<jacobian.count;++entry) {
    if(entry) EXPECT_LT(jacobian.nodes[entry-1],jacobian.nodes[entry]);
    EXPECT_EQ(jacobian.values[entry].y,0); EXPECT_EQ(jacobian.values[entry].z,0);
    const long double exact=std::abs(static_cast<long double>(jacobian.values[entry].x))*
                            std::sqrt(static_cast<long double>(f.inverse[jacobian.nodes[entry]]));
    EXPECT_GE(static_cast<long double>(jacobian.normalized_norm[entry]),exact);
  }
  sc::Q4PointKinematics before,after; sc::Q4NodalForces impulse;
  ASSERT_EQ(sc::EvaluateQ4Point(f.view(),f.point,&before),sc::Status::kOk);
  ASSERT_EQ(sc::ProjectQ4PointForce(f.view(),f.point,{-.75,0,0},&impulse),sc::Status::kOk);
  for(unsigned local=0;local<4;++local) f.velocity[3*impulse.nodes[local]]+=f.inverse[impulse.nodes[local]]*impulse.forces[local].x;
  ASSERT_EQ(sc::EvaluateQ4Point(f.view(),f.point,&after),sc::Status::kOk);
  EXPECT_NEAR(-(after.velocity.x-before.velocity.x),.75*jacobian.inverse_effective_mass,1e-15);
  EXPECT_EQ(f.velocity[3*f.parent.nodes[2]],0);  // Fixed X receives no velocity change.
}

TEST(Q4SurfaceMass, RejectsUnsupportedMasksInvalidMassAndMissingDynamicPoint) {
  Fixture f; sc::NormalJacobian before;
  ASSERT_EQ(sc::BuildQ4NormalXJacobian(f.mass(),f.parent,.2,-.4,7,&before),sc::Status::kOk);
  for(const auto bits:{0,1,2,3,4,5,8,255}) {
    auto bad=f; bad.fixed[f.parent.nodes[3]]=bits; auto output=before;
    EXPECT_EQ(sc::BuildQ4NormalXJacobian(bad.mass(),bad.parent,.2,-.4,7,&output),sc::Status::kUnsupportedInterpolation);
    Same(output,before);
  }
  for(const double inverse:{0.,-1.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
    auto bad=f; bad.inverse[f.parent.nodes[3]]=inverse; auto output=before;
    EXPECT_EQ(sc::BuildQ4NormalXJacobian(bad.mass(),bad.parent,.2,-.4,7,&output),sc::Status::kInvalidArgument);
    Same(output,before);
  }
  auto fixed=f; fixed.fixed.fill(7); fixed.inverse.fill(0); auto output=before;
  EXPECT_EQ(sc::BuildQ4NormalXJacobian(fixed.mass(),fixed.parent,.2,-.4,7,&output),sc::Status::kNoDynamicDofs); Same(output,before);
  // The only nonzero corner weight belongs to fixed physical node 5.
  EXPECT_EQ(sc::BuildQ4NormalXJacobian(f.mass(),f.parent,-1,-1,7,&output),sc::Status::kNoDynamicDofs); Same(output,before);
  auto bad=f; bad.inverse[f.parent.nodes[2]]=1;
  EXPECT_EQ(sc::BuildQ4NormalXJacobian(bad.mass(),bad.parent,.2,-.4,7,&output),sc::Status::kInvalidArgument); Same(output,before);
  EXPECT_EQ(sc::BuildQ4NormalXJacobian(f.mass(),f.parent,.2,-.4,0,&output),sc::Status::kInvalidArgument); Same(output,before);
  EXPECT_EQ(sc::BuildQ4NormalXJacobian(sc::Q4FixedYZMassView{},f.parent,.2,-.4,7,&output),sc::Status::kInvalidArgument); Same(output,before);
  EXPECT_EQ(sc::BuildQ4NormalXJacobian(f.mass(),{},.2,-.4,7,&output),sc::Status::kInvalidArgument); Same(output,before);
  EXPECT_EQ(sc::BuildQ4NormalXJacobian(f.mass(),f.parent,2,-.4,7,&output),sc::Status::kOutOfRange); Same(output,before);
  EXPECT_EQ(sc::BuildQ4NormalXJacobian(f.mass(),f.parent,.2,-.4,7,nullptr),sc::Status::kInvalidArgument);
}

TEST(Q4SurfaceMass, PositiveUnderflowAndMalformedLateNodesPreserveCompleteJacobian) {
  Fixture f; sc::NormalJacobian before;
  ASSERT_EQ(sc::BuildQ4NormalXJacobian(f.mass(),f.parent,0,0,7,&before),sc::Status::kOk);
  auto bad=f; bad.inverse[f.parent.nodes[3]]=std::numeric_limits<double>::denorm_min(); auto output=before;
  EXPECT_EQ(sc::BuildQ4NormalXJacobian(bad.mass(),bad.parent,0,0,7,&output),sc::Status::kNonFiniteResult); Same(output,before);
  bad=f; bad.parent.nodes[3]=bad.parent.nodes[0];
  EXPECT_EQ(sc::BuildQ4NormalXJacobian(bad.mass(),bad.parent,0,0,7,&output),sc::Status::kInvalidArgument); Same(output,before);
  bad=f; bad.fixed[f.parent.nodes[3]]=0;  // Even a zero-weight node must be admitted.
  EXPECT_EQ(sc::BuildQ4NormalXJacobian(bad.mass(),bad.parent,1,1,7,&output),sc::Status::kUnsupportedInterpolation); Same(output,before);
  ASSERT_EQ(sc::BuildQ4NormalXJacobian(f.mass(),f.parent,0,0,7,&output),sc::Status::kOk); Same(output,before);
}
}  // namespace
