#include "lib_src/collision/Q4PlanarSweepGeometry.h"
#include "lib_utest/q4_planar_geometry_fixture.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

namespace {
namespace sc=tlfea::contact;
namespace fixture=q4_planar_test;
using PStatus=sc::PlanarContactStatus;
using Single=fixture::Single;
using Result=sc::Q4PlanarSweepGeometry;

sc::Q4PlanarSweepLimits Limits(double ratio=1e-6) { return {fixture::Clearance,ratio}; }
struct Prepared {
  sc::PlanarWallGeometry wall;
  sc::Q4PlanarGeometry reference;
  bool Initialize(const fixture::Wall& input,const sc::Q4SurfaceView& surface,const sc::Q4FixedYZMassView& mass) {
    return wall.Initialize(input.view()).status == PStatus::Ok &&
           reference.Initialize(wall,surface,mass,fixture::Clearance).status == PStatus::Ok;
  }
  bool Initialize(const fixture::Wall& input,const Single& surface) {
    const auto view=surface.Input(); return Initialize(input,view.surface,view.mass);
  }
  sc::PlanarContactReport Check(const sc::Q4SurfaceView& base,const sc::Q4SurfaceView& candidate,
                              Result* output,sc::Q4PlanarSweepLimits limits=Limits()) const {
    return sc::CheckQ4PlanarSweep(wall,reference.view(),base,candidate,limits,output);
  }
  sc::PlanarContactReport Check(const Single& base,const Single& candidate,Result* output,
                              sc::Q4PlanarSweepLimits limits=Limits()) const {
    return Check(base.Input().surface,candidate.Input().surface,output,limits);
  }
};
using ResultBytes=std::array<unsigned char,sizeof(Result)>;
ResultBytes Bytes(const Result& result) {
  ResultBytes bytes; std::memcpy(bytes.data(),&result,sizeof(result)); return bytes;
}
Result Seed() {
  Result result; result.valid=true; result.parent_count=2;
  for (unsigned p=0;p<2;++p) {
    auto& r=result.parents[p]; r.parent.feature_id=fixture::LargeId+p+9;
    r.reference_material_area=17+p; r.projected_jacobian={3,4};
    r.projected_minimum={6,7,8}; r.projected_maximum={9,10,11};
  }
  return result;
}
void Same(sc::Vec3 a,sc::Vec3 b) { EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.y,b.y); EXPECT_EQ(a.z,b.z); }
void Same(sc::Q4IntegralInterval a,sc::Q4IntegralInterval b) { EXPECT_EQ(a.lower,b.lower); EXPECT_EQ(a.upper,b.upper); }
void Same(const Result& a,const Result& b) {
  ASSERT_EQ(a.parent_count,b.parent_count); EXPECT_EQ(a.valid,b.valid);
  for (unsigned p=0;p<a.parent_count;++p) {
    const auto& x=a.parents[p]; const auto& y=b.parents[p];
    for (unsigned n=0;n<4;++n) EXPECT_EQ(x.parent.nodes[n],y.parent.nodes[n]);
    EXPECT_EQ(x.parent.feature_id,y.parent.feature_id); EXPECT_EQ(x.parent.parent_element_id,y.parent.parent_element_id);
    EXPECT_EQ(x.parent.parent_face_id,y.parent.parent_face_id); EXPECT_EQ(x.parent.half_thickness,y.parent.half_thickness);
    Same(x.projected_minimum,y.projected_minimum); Same(x.projected_maximum,y.projected_maximum);
    Same(x.projected_jacobian,y.projected_jacobian); Same(x.projected_jacobian_ratio,y.projected_jacobian_ratio);
    EXPECT_EQ(x.required_jacobian_lower,y.required_jacobian_lower);
    EXPECT_EQ(x.reference_material_area,y.reference_material_area);
    Same(x.reference_material_area_enclosure,y.reference_material_area_enclosure);
  }
}
void TransformProjection(Single* surface,double yy,double yz,double zy,double zz,double y0=0,double z0=0) {
  for (unsigned n=0;n<4;++n) {
    const double y=surface->position[n+4],z=surface->position[n+8];
    surface->position[n+4]=yy*y+yz*z+y0; surface->position[n+8]=zy*y+zz*z+z0;
  }
}

// Independent direct Q4 shape-derivative oracle using long-double arithmetic.
// It evaluates the represented endpoint coordinates; it neither reconstructs
// Bernstein coefficients nor treats sampling as a positivity certificate.
long double Jacobian(const sc::Q4SurfaceView& base,const sc::Q4SurfaceView& candidate,
                     unsigned parent,long double u,long double v,long double t) {
  constexpr long double su[4]={1,-1,-1,1},sv[4]={1,1,-1,-1};
  long double dydu=0,dydv=0,dzdu=0,dzdv=0;
  for (unsigned n=0;n<4;++n) {
    const auto index=base.parents[parent].nodes[n];
    const auto a=base.positions.at(index),b=candidate.positions.at(index);
    const long double y=(1-t)*a.y+t*b.y,z=(1-t)*a.z+t*b.z;
    const long double du=su[n]*(1+sv[n]*v)/4,dv=sv[n]*(1+su[n]*u)/4;
    dydu+=du*y; dydv+=dv*y; dzdu+=du*z; dzdv+=dv*z;
  }
  return dydu*dzdv-dydv*dzdu;
}

TEST(Q4PlanarSweepGeometry, MovingProjectionKeepsReferenceMeasureAndOldC3Restriction) {
  Single base,candidate; Prepared prepared;
  ASSERT_TRUE(prepared.Initialize(fixture::Square(),base));
  TransformProjection(&candidate,1,0,0,1,.25,-.125);
  for (unsigned n=0;n<4;++n) { candidate.velocity[3*n+1]=n+.5; candidate.velocity[3*n+2]=-.25; }
  // These are prescribed geometry views, not fixed-component owner admission.
  Result result;
  ASSERT_EQ(prepared.Check(base,candidate,&result).status,PStatus::Ok);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.parent_count,1u);
  const auto& parent=result.parents[0];
  Same(parent.projected_minimum,{0,-.5,-.625}); Same(parent.projected_maximum,{0,.75,.5});
  Same(parent.projected_jacobian,{.25,.25});
  EXPECT_LE(parent.projected_jacobian_ratio.lower,1); EXPECT_GE(parent.projected_jacobian_ratio.upper,1);
  EXPECT_EQ(parent.reference_material_area,1); Same(parent.reference_material_area_enclosure,{1,1});
  const auto current=candidate.Input();
  EXPECT_EQ(sc::ValidateQ4PlanarMotion(prepared.reference.view(),current.surface,current.mass),PStatus::UnsupportedMotion);
  EXPECT_EQ(prepared.reference.view().parents[0].reference_projection[0].y,.5);
  auto shifted_wall=fixture::Square();
  for (auto& vertex:shifted_wall.vertices) vertex.position.x=3.25;
  Prepared shifted; ASSERT_TRUE(shifted.Initialize(shifted_wall,base));
  ASSERT_EQ(shifted.Check(base,candidate,&result).status,PStatus::Ok);
  EXPECT_EQ(result.parents[0].projected_minimum.x,3.25);
  EXPECT_EQ(result.parents[0].projected_maximum.x,3.25);
  EXPECT_EQ(result.parents[0].reference_material_area,1);
}

TEST(Q4PlanarSweepGeometry, SharedParentsAndSweptBoxesIgnoreWallDiagonalsSubdivisionAndFaceOrder) {
  Result baseline;
  for (unsigned variant=0;variant<4;++variant) {
    SCOPED_TRACE(variant);
    fixture::Pair base,candidate; Prepared prepared;
    ASSERT_TRUE(prepared.Initialize(fixture::Square(variant),base.surface(),base.mass()));
    for (unsigned n=0;n<6;++n) {
      candidate.x[n+6]+=.125; candidate.x[n+12]-=.25;
      candidate.v[3*n+1]=.125; candidate.v[3*n+2]=-.25;
    }
    Result result;
    ASSERT_EQ(prepared.Check(base.surface(),candidate.surface(),&result).status,PStatus::Ok);
    ASSERT_EQ(result.parent_count,2u);
    for (unsigned p=0;p<2;++p) {
      EXPECT_EQ(result.parents[p].parent.parent_element_id,base.parents[p].parent_element_id);
      EXPECT_EQ(result.parents[p].reference_material_area,1);
      Same(result.parents[p].projected_jacobian,{.25,.25});
    }
    Same(result.parents[0].projected_minimum,{0,-1,-.75});
    Same(result.parents[0].projected_maximum,{0,.125,.5});
    Same(result.parents[1].projected_minimum,{0,0,-.75});
    Same(result.parents[1].projected_maximum,{0,1.125,.5});
    if (variant == 0) baseline=result;
    else Same(result,baseline);
  }
}

TEST(Q4PlanarSweepGeometry, RotatedBilinearFootprintAndNormalWarpHaveBoundedPositiveCurrentMap) {
  Single base,candidate; Prepared prepared;
  ASSERT_TRUE(prepared.Initialize(fixture::Square(2),base));
  const double cosine=std::sqrt(.75);
  TransformProjection(&candidate,cosine,-.5,.5,cosine,.125,-.125);
  candidate.position[candidate.parent.nodes[0]+4]+=.0625;
  candidate.position[candidate.parent.nodes[0]+8]-=.03125;
  candidate.Gaps(.125,-.25,.375,-.5);
  Result result;
  ASSERT_EQ(prepared.Check(base,candidate,&result).status,PStatus::Ok);
  const auto& r=result.parents[0]; EXPECT_GT(r.projected_jacobian.lower,r.required_jacobian_lower);
  EXPECT_EQ(r.reference_material_area,1);  // Current nonaffine projection is not a new material measure.
  for (unsigned step=0;step<=8;++step) for (unsigned iu=0;iu<=8;++iu) for (unsigned iv=0;iv<=8;++iv) {
    const auto actual=Jacobian(base.Input().surface,candidate.Input().surface,0,-1+iu/4.L,-1+iv/4.L,step/8.L);
    EXPECT_GE(actual,static_cast<long double>(r.projected_jacobian.lower));
    EXPECT_LE(actual,static_cast<long double>(r.projected_jacobian.upper));
  }
  for (unsigned n=0;n<4;++n) {
    const auto x=candidate.Input().surface.positions.at(n);
    EXPECT_GE(x.y,r.projected_minimum.y); EXPECT_LE(x.y,r.projected_maximum.y);
    EXPECT_GE(x.z,r.projected_minimum.z); EXPECT_LE(x.z,r.projected_maximum.z);
  }
}

TEST(Q4PlanarSweepGeometry, HalfTurnRejectsIntermediateCollapseDespiteBothValidEndpoints) {
  Single base,candidate; Prepared prepared;
  ASSERT_TRUE(prepared.Initialize(fixture::Square(),base));
  TransformProjection(&candidate,-1,0,0,-1);
  Result endpoint;
  ASSERT_EQ(prepared.Check(base,base,&endpoint).status,PStatus::Ok);
  ASSERT_EQ(prepared.Check(candidate,candidate,&endpoint).status,PStatus::Ok);
  EXPECT_EQ(Jacobian(base.Input().surface,candidate.Input().surface,0,.25L,-.75L,.5L),0);
  auto result=Seed(); const auto before=Bytes(result);
  const auto report=prepared.Check(base,candidate,&result);
  EXPECT_EQ(report.status,PStatus::UnsupportedMotion); EXPECT_EQ(report.sample,0u);
  EXPECT_NE(std::string(report.message).find("Unresolved Bernstein"),std::string::npos);
  EXPECT_EQ(Bytes(result),before);
}

TEST(Q4PlanarSweepGeometry, PositiveSweepsWithNonpositiveControlsAreUnresolvedWithoutEndpointFallback) {
  Single base; Prepared prepared;
  ASSERT_TRUE(prepared.Initialize(fixture::Square(),base));
  for (const double real:{0.,-.5}) {
    SCOPED_TRACE(real);
    Single candidate=base; TransformProjection(&candidate,real,-1,1,real);
    // A quarter turn has middle coefficient zero. The second scaled rotation
    // has coefficient -A0/8, despite true minimum J=1/13. Both curves stay
    // positive, but this unsplit Bernstein hull cannot certify the floor.
    for (unsigned step=0;step<=16;++step) {
      const long double t=step/16.L,a=1+(static_cast<long double>(real)-1)*t;
      const long double expected=.25L*(a*a+t*t);
      EXPECT_EQ(Jacobian(base.Input().surface,candidate.Input().surface,0,-.5L,.25L,t),expected);
      EXPECT_GE(expected,real == 0 ? .125L : .0625L);
    }
    Result endpoint;
    ASSERT_EQ(prepared.Check(candidate,candidate,&endpoint).status,PStatus::Ok);
    auto result=Seed(); const auto before=Bytes(result);
    const auto report=prepared.Check(base,candidate,&result);
    EXPECT_EQ(report.status,PStatus::UnsupportedMotion);
    EXPECT_NE(std::string(report.message).find("Unresolved Bernstein"),std::string::npos);
    EXPECT_EQ(Bytes(result),before);
  }
}

TEST(Q4PlanarSweepGeometry, SweptBoxRejectsHoleAndExposedBoundaryEvenWhenEndpointMapsArePositive) {
  Single base; TransformProjection(&base,.25,0,0,.25,-1,0);
  Single candidate=base; TransformProjection(&candidate,1,0,0,1,2,0);
  Prepared ring; ASSERT_TRUE(ring.Initialize(fixture::Ring(),base));
  Result endpoint;
  ASSERT_EQ(ring.Check(base,base,&endpoint).status,PStatus::Ok);
  ASSERT_EQ(ring.Check(candidate,candidate,&endpoint).status,PStatus::Ok);
  auto result=Seed(); const auto before=Bytes(result);
  EXPECT_EQ(ring.Check(base,candidate,&result).status,PStatus::AmbiguousBoundary);
  EXPECT_EQ(Bytes(result),before);
  Single center; Prepared square; ASSERT_TRUE(square.Initialize(fixture::Square(),center));
  for (const double shift:{1.5,1.75,3.5}) {
    SCOPED_TRACE(shift);
    Single outside=center; TransformProjection(&outside,1,0,0,1,shift,0);
    EXPECT_NE(square.Check(center,outside,&result).status,PStatus::Ok);
    EXPECT_EQ(Bytes(result),before);
  }
}

TEST(Q4PlanarSweepGeometry, ExplicitRatioIncludesThresholdAndRejectsNeighborBelowWithUnchangedOutput) {
  Single reference; Prepared prepared; ASSERT_TRUE(prepared.Initialize(fixture::Square(),reference));
  for (const double scale:{std::nextafter(.5,0),.5,std::nextafter(.5,1)}) {
    SCOPED_TRACE(scale);
    Single current=reference; TransformProjection(&current,scale,0,0,1);
    auto result=Seed(); const auto before=Bytes(result);
    const auto report=prepared.Check(current,current,&result,Limits(.5));
    if (scale < .5) {
      EXPECT_EQ(report.status,PStatus::UnsupportedMotion); EXPECT_EQ(Bytes(result),before);
    } else {
      ASSERT_EQ(report.status,PStatus::Ok);
      EXPECT_EQ(result.parents[0].required_jacobian_lower,.125);
      EXPECT_GE(result.parents[0].projected_jacobian.lower,.125);
    }
  }
  for (const double ratio:{0.,-1.,std::nextafter(1.,2.),std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    auto result=Seed(); const auto before=Bytes(result);
    EXPECT_EQ(prepared.Check(reference,reference,&result,Limits(ratio)).status,PStatus::InvalidInput);
    EXPECT_EQ(Bytes(result),before);
  }
  Result result;
  EXPECT_EQ(prepared.Check(reference,reference,&result,Limits(1)).status,PStatus::Ok);
  Single smaller; TransformProjection(&smaller,.5,0,0,.25);
  Prepared nonunit; ASSERT_TRUE(nonunit.Initialize(fixture::Square(),smaller));
  ASSERT_EQ(nonunit.reference.view().parents[0].projected_area,.125);
  for (const double scale:{std::nextafter(.5,0),.5,std::nextafter(.5,1)}) {
    SCOPED_TRACE(scale);
    Single current=smaller; TransformProjection(&current,scale,0,0,1);
    result=Seed(); const auto before=Bytes(result);
    const auto report=nonunit.Check(current,current,&result,Limits(.5));
    if (scale < .5) {
      EXPECT_EQ(report.status,PStatus::UnsupportedMotion); EXPECT_EQ(Bytes(result),before);
    } else {
      ASSERT_EQ(report.status,PStatus::Ok);
      const auto& parent=result.parents[0];
      EXPECT_EQ(parent.reference_material_area,.125); Same(parent.reference_material_area_enclosure,{.125,.125});
      EXPECT_EQ(parent.required_jacobian_lower,.015625);  // .5 * A0/4.
      Same(parent.projected_jacobian,{.03125*scale,.03125*scale});
      EXPECT_LE(parent.projected_jacobian_ratio.lower,scale);
      EXPECT_GE(parent.projected_jacobian_ratio.upper,scale);
    }
  }
  for (const double clearance:{0.,8*prepared.wall.tolerance(),std::numeric_limits<double>::quiet_NaN()}) {
    auto limits=Limits(); limits.exposed_clearance=clearance;
    result=Seed(); const auto before=Bytes(result);
    EXPECT_EQ(prepared.Check(reference,reference,&result,limits).status,PStatus::InvalidInput);
    EXPECT_EQ(Bytes(result),before);
  }
}

TEST(Q4PlanarSweepGeometry, LateSecondParentFailuresPreserveAllOutputAndPermitCleanRetry) {
  fixture::Pair base; Prepared prepared;
  ASSERT_TRUE(prepared.Initialize(fixture::Square(),base.surface(),base.mass()));
  Result clean; ASSERT_EQ(prepared.Check(base.surface(),base.surface(),&clean).status,PStatus::Ok);
  for (unsigned fault=0;fault<6;++fault) {
    SCOPED_TRACE(fault);
    fixture::Pair candidate=base;
    if (fault == 0) candidate.x[4+6]=std::numeric_limits<double>::quiet_NaN();
    if (fault == 1) candidate.v[3*5+2]=std::numeric_limits<double>::infinity();
    if (fault == 2) ++candidate.parents[1].parent_element_id;
    if (fault == 3) candidate.parents[1].nodes[3]=6;
    if (fault == 4) candidate.parents[1].half_thickness=.01;
    if (fault == 5) { candidate.x[4+6]=std::numeric_limits<double>::max(); candidate.x[2+6]=-std::numeric_limits<double>::max(); }
    auto result=Seed(); const auto before=Bytes(result);
    const auto report=prepared.Check(base.surface(),candidate.surface(),&result);
    EXPECT_NE(report.status,PStatus::Ok);
    if (fault != 5) EXPECT_EQ(report.sample,1u);
    EXPECT_EQ(Bytes(result),before);
    ASSERT_EQ(prepared.Check(base.surface(),base.surface(),&result).status,PStatus::Ok);
    Same(result,clean);
  }
  auto result=Seed(); const auto before=Bytes(result);
  auto reference=prepared.reference.view();
  std::array<sc::PreparedQ4PlanarParent,2> parents{{reference.parents[0],reference.parents[1]}};
  reference.parents=parents.data();
  for (unsigned fault=0;fault<5;++fault) {
    SCOPED_TRACE(fault);
    parents[1]=prepared.reference.view().parents[1];
    if (fault == 0) parents[1].projected_area=2;
    if (fault == 1) parents[1].area_enclosure.upper=2;
    if (fault == 2) parents[1].parent.feature_id=parents[0].parent.feature_id;
    if (fault == 3) parents[1].reference_projection[0].y+=.125;
    if (fault == 4) parents[1].covered=false;
    const auto report=sc::CheckQ4PlanarSweep(prepared.wall,reference,base.surface(),base.surface(),Limits(),&result);
    EXPECT_NE(report.status,PStatus::Ok); EXPECT_EQ(report.sample,1u); EXPECT_EQ(Bytes(result),before);
  }
  EXPECT_EQ(sc::CheckQ4PlanarSweep(prepared.wall,{},base.surface(),base.surface(),Limits(),&result).status,PStatus::NotInitialized);
  EXPECT_EQ(Bytes(result),before);
  auto foreign=prepared.reference.view(); foreign.wall_x=1;
  EXPECT_EQ(sc::CheckQ4PlanarSweep(prepared.wall,foreign,base.surface(),base.surface(),Limits(),&result).status,PStatus::NotInitialized);
  EXPECT_EQ(Bytes(result),before);
  EXPECT_EQ(prepared.Check(base.surface(),base.surface(),nullptr).status,PStatus::InvalidOutput);
  auto invalid=base.surface(); invalid.positions.node_count=5;
  EXPECT_EQ(prepared.Check(invalid,base.surface(),&result).status,PStatus::InvalidInput); EXPECT_EQ(Bytes(result),before);
}
}  // namespace
