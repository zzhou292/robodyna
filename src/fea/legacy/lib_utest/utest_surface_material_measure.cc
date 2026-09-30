#include "lib_src/collision/SurfaceMaterialMeasure.h"

#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <limits>

namespace {
namespace sc=tlfea::contact;
using Code=sc::SurfaceMeasureStatus;
using Points=std::array<sc::Vec3,4>;
constexpr std::uint64_t SourceId=(std::uint64_t{1}<<55)+17;
sc::SurfaceQ4 Parent() { return {{0,1,2,3},SourceId,SourceId+1,2,0}; }
sc::SurfaceTriangle Triangle() {
  return {{0,1,2},SourceId,SourceId+1,2,0,sc::SurfaceInterpolation::kLinearTriangle};
}
sc::VectorView View(const Points& x) { return {&x[0].x,4,3,1}; }
Points Saddle(double a=1) { return {{{1,1,a},{-1,1,-a},{-1,-1,a},{1,-1,-a}}}; }
template<class T> auto Bytes(const T& object) {
  std::array<unsigned char,sizeof(T)> bytes; std::memcpy(bytes.data(),&object,sizeof(T)); return bytes;
}
void Encloses(sc::Q4IntegralInterval interval,long double truth) {
  EXPECT_LE(static_cast<long double>(interval.lower),truth);
  EXPECT_GE(static_cast<long double>(interval.upper),truth);
}
void Encloses(sc::Q4CertifiedIntegral interval,long double truth) {
  Encloses(sc::Q4IntegralInterval{interval.lower,interval.upper},truth);
  EXPECT_GE(static_cast<long double>(interval.error),::fabsl(static_cast<long double>(interval.value)-truth));
}
// Independent direct shape-derivative oracle in extended precision. This is
// sampling for regression, never the production regularity proof.
long double Density(const Points& x,long double u,long double v) {
  constexpr int su[4]={1,-1,-1,1},sv[4]={1,1,-1,-1};
  long double a[3]{},b[3]{};
  for (unsigned n=0;n<4;++n) {
    const double p[3]={x[n].x,x[n].y,x[n].z};
    for (unsigned c=0;c<3;++c) {
      a[c]+=su[n]*(1+sv[n]*v)*p[c]/4;
      b[c]+=sv[n]*(1+su[n]*u)*p[c]/4;
    }
  }
  const long double g[3]={a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
  return ::sqrtl(g[0]*g[0]+g[1]*g[1]+g[2]*g[2]);
}

TEST(SurfaceMaterialMeasure, NativeTriangleHasSimplexDensityAndPhysicalArea) {
  Points x{{{0,0,0},{2,0,0},{0,3,4},{9,9,9}}};
  sc::T3MaterialMeasure reference;
  ASSERT_EQ(sc::PrepareT3MaterialMeasure(View(x),Triangle(),&reference),Code::Ok);
  ASSERT_TRUE(reference.prepared());
  Encloses(reference.density(),10); Encloses(reference.area_enclosure(),5);
  EXPECT_EQ(reference.parent().parent_element_id,SourceId+1);
  EXPECT_EQ(reference.position(2).z,4);
  const auto before=Bytes(reference);
  x[1].x=100;  // Preparation copied the original reference, not a borrowed owner.
  EXPECT_EQ(Bytes(reference),before); Encloses(reference.area_enclosure(),5);
}

TEST(SurfaceMaterialMeasure, SkewPlanarQuadUsesIntrinsicNonunitMeasure) {
  Points x;
  constexpr double u[4]={1,-1,-1,1},v[4]={1,1,-1,-1};
  for (unsigned n=0;n<4;++n) x[n]={u[n],.25*u[n]+2*v[n],.5*v[n]};
  sc::Q4MaterialMeasure reference;
  ASSERT_EQ(sc::PrepareQ4MaterialMeasure(View(x),Parent(),&reference),Code::Ok);
  const long double density=::sqrtl(.125L*.125L+.5L*.5L+4);
  Encloses(reference.area_enclosure(),4*density);
  for (double a:{-1.,-.3,0.,.625,1.}) for (double b:{-1.,-.7,0.,.25,1.}) {
    sc::Q4CertifiedIntegral result;
    ASSERT_EQ(sc::EvaluateQ4MaterialDensity(reference,a,b,&result),Code::Ok);
    Encloses(result,density);
  }
  EXPECT_GT(reference.direction_norm_upper(),1); // Its fixed direction is not assumed unit.
}

TEST(SurfaceMaterialMeasure, WarpedSaddleCellBoundsEncloseAnalyticDensityAndArea) {
  const auto x=Saddle(); sc::Q4MaterialMeasure reference;
  ASSERT_EQ(sc::PrepareQ4MaterialMeasure(View(x),Parent(),&reference),Code::Ok);
  // Integral on [-1,1]^2 of sqrt(1+u^2+v^2), from the rectangular
  // antiderivative. Unlike an area-from-display-triangles oracle, it is warped.
  const long double root3=::sqrtl(3.L);
  const long double area=4.L/3*(root3+4*::asinhl(1/::sqrtl(2.L))-::atanl(1/root3));
  Encloses(reference.area_enclosure(),area);
  long double summed_lower=0,summed_upper=0;
  constexpr unsigned cells=16;
  for (unsigned row=0;row<cells;++row) for (unsigned col=0;col<cells;++col) {
    const sc::Q4NaturalBox box{-1+2.*col/cells,-1+2.*(col+1)/cells,
                               -1+2.*row/cells,-1+2.*(row+1)/cells};
    sc::Q4IntegralInterval bounds;
    ASSERT_EQ(sc::BoundQ4MaterialDensity(reference,box,&bounds),Code::Ok);
    const long double u=(box.u0+box.u1)/2,v=(box.v0+box.v1)/2;
    Encloses(bounds,::sqrtl(1+u*u+v*v));
    summed_lower+=bounds.lower*4.L/(cells*cells); summed_upper+=bounds.upper*4.L/(cells*cells);
  }
  EXPECT_LE(summed_lower,area); EXPECT_GE(summed_upper,area);
  EXPECT_LT(summed_upper-summed_lower,.2L*(reference.area_enclosure().upper-reference.area_enclosure().lower));
  sc::Q4CertifiedIntegral center,corner;
  ASSERT_EQ(sc::EvaluateQ4MaterialDensity(reference,0,0,&center),Code::Ok);
  ASSERT_EQ(sc::EvaluateQ4MaterialDensity(reference,1,1,&corner),Code::Ok);
  Encloses(center,1); Encloses(corner,root3);
  EXPECT_GT(corner.value,1.7*center.value); // A scalar reference area cannot replace this density.
}

TEST(SurfaceMaterialMeasure, CyclicReversalProperRotationAndScalingKeepPhysicalMeasure) {
  const auto original=Saddle(.25);
  for (double scale:{.25,2.}) for (unsigned permutation=0;permutation<8;++permutation) {
    SCOPED_TRACE(scale);
    SCOPED_TRACE(permutation);
    Points x;
    for (unsigned n=0;n<4;++n) {
      const auto index=permutation < 4 ? (n+permutation)%4 : (4+permutation-n)%4;
      const auto p=original[index];
      x[n]={4+scale*p.z,-2+scale*p.x,.5+scale*p.y}; // Exact cyclic proper rotation + dyadic translation.
    }
    sc::Q4MaterialMeasure reference;
    ASSERT_EQ(sc::PrepareQ4MaterialMeasure(View(x),Parent(),&reference),Code::Ok);
    for (double u:{-1.,-.25,.5,1.}) for (double v:{-1.,0.,.75,1.}) {
      sc::Q4CertifiedIntegral value;
      ASSERT_EQ(sc::EvaluateQ4MaterialDensity(reference,u,v,&value),Code::Ok);
      Encloses(value,Density(x,u,v));
      Encloses(value,scale*scale*::sqrtl(1+.0625L*(u*u+v*v)));
    }
  }
}

TEST(SurfaceMaterialMeasure, EdgeOnAndMixedWallProjectionAreRegularIntrinsicSurfaces) {
  Points edge_on{{{1,1,0},{-1,1,0},{-1,-1,0},{1,-1,0}}};
  sc::Q4MaterialMeasure reference;
  ASSERT_EQ(sc::PrepareQ4MaterialMeasure(View(edge_on),Parent(),&reference),Code::Ok);
  Encloses(reference.area_enclosure(),4);
  // Actual source Q4 2126280, retained source order, no invented planar fit.
  const Points mixed{{{-.36290677,.42733408,.41496219},{-.36980676,.42903571,.42858191},
                       {-.38900333000000004,.42824298,.42051611000000005},
                       {-.37343292000000006,.42528113,.40896570000000004}}};
  auto parent=Parent(); parent.parent_element_id=2126280;
  ASSERT_EQ(sc::PrepareQ4MaterialMeasure(View(mixed),parent,&reference),Code::Ok);
  bool positive=false,negative=false;
  for (unsigned n=0;n<4;++n) {
    positive=positive || reference.nominal_corner(n).x > 0;
    negative=negative || reference.nominal_corner(n).x < 0;
  }
  EXPECT_TRUE(positive && negative);
  for (double u:{-1.,0.,1.}) for (double v:{-1.,0.,1.}) {
    sc::Q4CertifiedIntegral result;
    ASSERT_EQ(sc::EvaluateQ4MaterialDensity(reference,u,v,&result),Code::Ok);
    Encloses(result,Density(mixed,u,v));
  }
}

TEST(SurfaceMaterialMeasure, Q4MalformedAndUnresolvedPreparationPreservesCompleteOutput) {
  const auto x=Saddle(.25); sc::Q4MaterialMeasure reference;
  ASSERT_EQ(sc::PrepareQ4MaterialMeasure(View(x),Parent(),&reference),Code::Ok);
  const auto before=Bytes(reference);
  for (unsigned fault=0;fault<8;++fault) {
    SCOPED_TRACE(fault);
    auto bad=x; auto parent=Parent();
    switch (fault) {
      case 0: bad[3].z=std::numeric_limits<double>::quiet_NaN(); break;
      case 1: parent.nodes[3]=0; break;
      case 2: parent.nodes[3]=4; break;
      case 3: parent.half_thickness=.001; break;
      case 4: parent.feature_id=0; break;
      case 5: bad={{{1,0,0},{-1,0,0},{-2,0,0},{2,0,0}}}; break;
      case 6: bad={{{1,1,0},{-1,-1,0},{-1,1,0},{1,-1,0}}}; break;
      case 7: for (auto& p:bad) p=sc::Scale(p,DBL_MIN); break;
    }
    EXPECT_NE(sc::PrepareQ4MaterialMeasure(View(bad),parent,&reference),Code::Ok);
    EXPECT_EQ(Bytes(reference),before);
  }
  ASSERT_EQ(sc::PrepareQ4MaterialMeasure(View(x),Parent(),&reference),Code::Ok);
  sc::Q4MaterialMeasure retry;
  ASSERT_EQ(sc::PrepareQ4MaterialMeasure(View(x),Parent(),&retry),Code::Ok);
  EXPECT_EQ(reference.area_enclosure().lower,retry.area_enclosure().lower);
  EXPECT_EQ(reference.area_enclosure().upper,retry.area_enclosure().upper);
}

TEST(SurfaceMaterialMeasure, NativeTriangleFailuresAndReversalAreStaged) {
  const Points x{{{0,0,0},{2,0,0},{0,3,4},{0,0,0}}}; sc::T3MaterialMeasure reference;
  ASSERT_EQ(sc::PrepareT3MaterialMeasure(View(x),Triangle(),&reference),Code::Ok);
  for (unsigned fault=0;fault<7;++fault) {
    SCOPED_TRACE(fault);
    const auto before=Bytes(reference); auto bad=x; auto parent=Triangle();
    switch (fault) {
      case 0: parent.nodes[2]=1; break;
      case 1: parent.nodes[2]=4; break;
      case 2: parent.half_thickness=1; break;
      case 3: parent.interpolation=sc::SurfaceInterpolation::kUnspecified; break;
      case 4: parent.parent_element_id=0; break;
      case 5: bad[2]={4,0,0}; break;
      case 6: bad[2].z=HUGE_VAL; break;
    }
    EXPECT_NE(sc::PrepareT3MaterialMeasure(View(bad),parent,&reference),Code::Ok);
    EXPECT_EQ(Bytes(reference),before);
  }
  auto reversed=Triangle(); reversed.nodes[1]=2; reversed.nodes[2]=1;
  ASSERT_EQ(sc::PrepareT3MaterialMeasure(View(x),reversed,&reference),Code::Ok);
  Encloses(reference.density(),10); Encloses(reference.area_enclosure(),5);
}

TEST(SurfaceMaterialMeasure, DensityQueriesRejectDefaultsInvalidBoxesAndPreserveOutputs) {
  sc::Q4MaterialMeasure empty,reference; const auto x=Saddle();
  ASSERT_EQ(sc::PrepareQ4MaterialMeasure(View(x),Parent(),&reference),Code::Ok);
  sc::Q4IntegralInterval bounds{17,19}; sc::Q4CertifiedIntegral point{23,21,24,3};
  const auto bounds_before=Bytes(bounds);
  const auto point_before=Bytes(point);
  EXPECT_EQ(sc::BoundQ4MaterialDensity(empty,{},&bounds),Code::NotPrepared);
  EXPECT_EQ(Bytes(bounds),bounds_before);
  const sc::Q4NaturalBox boxes[4]={{0,-.1,-1,1},{-1,1,-1,1.01},{-HUGE_VAL,1,-1,1},
                                 {-1,1,0,std::numeric_limits<double>::quiet_NaN()}};
  for (auto box:boxes) {
    EXPECT_EQ(sc::BoundQ4MaterialDensity(reference,box,&bounds),Code::InvalidInput);
    EXPECT_EQ(Bytes(bounds),bounds_before);
  }
  EXPECT_EQ(sc::EvaluateQ4MaterialDensity(reference,1.001,0,&point),Code::InvalidInput);
  EXPECT_EQ(Bytes(point),point_before);
  EXPECT_EQ(sc::BoundQ4MaterialDensity(reference,{},nullptr),Code::InvalidOutput);
  EXPECT_EQ(sc::PrepareQ4MaterialMeasure(View(x),Parent(),nullptr),Code::InvalidOutput);
  EXPECT_EQ(sc::PrepareT3MaterialMeasure(View(x),Triangle(),nullptr),Code::InvalidOutput);
  ASSERT_EQ(sc::EvaluateQ4MaterialDensity(reference,.2,-.3,&point),Code::Ok);
  Encloses(point,Density(x,.2,-.3));
}
} // namespace
