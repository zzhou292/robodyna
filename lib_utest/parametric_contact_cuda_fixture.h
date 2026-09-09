#pragma once

#include "lib_src/collision/Q4ParametricContact.h"
#include "lib_src/collision/Q4IntegralMeasure.h"
#include "lib_src/collision/Q4RectangularIntegration.h"
#include "lib_src/collision/T3ContactIntegration.h"

#include <algorithm>

namespace parametric_cuda_test {
namespace sc=tlfea::contact;
inline constexpr sc::T3IntegrationLimits T3Budget{1e-10,1e-11}; // Frozen native host-test budgets.
inline sc::Q4IntegrationLimits Q4Budget() {
  sc::Q4IntegrationLimits value; value.force_error=1e-7; value.energy_error=1e-9; return value;
}
struct NodalData {
  double reference[24]{},position[24]{},velocity[24]{},inverse[8]{};
  std::uint8_t fixed[8]{};
  TL_SURFACE_HD sc::VectorView View(const double* values) const { return {values,8,3,1}; }
  TL_SURFACE_HD sc::LumpedTranslationMassView Mass() const {
    return {inverse,fixed,8,17,sc::TranslationMassModel::kIsotropicLumped};
  }
  void ResetMass() { for (unsigned n=0;n<8;++n) { inverse[n]=1+.25*n; fixed[n]=0; } }
};
struct Q4Fixture : NodalData {
  sc::SurfaceQ4 parents[2]{{{0,2,4,6},101,201,0,0},{{5,1,7,3},103,203,9,0}};
  sc::Q4MaterialMeasure intrinsic;
  sc::Q4CertifiedIntegral area;
  sc::Q4IntegrationLimits limits=Q4Budget();
  sc::TranslationMassModel mass_model=sc::TranslationMassModel::kIsotropicLumped;
  TL_SURFACE_HD sc::Q4PrescribedNormalIntegrationInput Input() const {
    auto mass=Mass(); mass.model=mass_model;
    return {{View(position),View(velocity),parents,2},mass,1,31,0,area.value,16,.125};
  }
  bool Prepare(unsigned kind) {
    ResetMass();
    constexpr double u[4]={1,-1,-1,1},v[4]={1,1,-1,-1};
    const sc::Vec3 source[4]={{-.36290677,.42733408,.41496219},{-.36980676,.42903571,.42858191},
      {-.38900333000000004,.42824298,.42051611000000005},{-.37343292000000006,.42528113,.40896570000000004}};
    for (unsigned i=0;i<4;++i) {
      const unsigned node=parents[1].nodes[i];
      sc::Vec3 x{0,.25*u[i],.125*v[i]};
      if (kind==1) x=source[i];
      if (kind==2) x={.25*u[i],0,.125*v[i]}; // Intrinsically regular and exactly edge-on.
      reference[3*node]=x.x; reference[3*node+1]=x.y; reference[3*node+2]=x.z;
      double gap=.03125;
      if (kind==1) gap*=u[i];
      if (kind==2) gap+=.0078125*u[i];
      if (kind==3) gap=-gap;
      position[3*node]=gap; position[3*node+1]=x.y; position[3*node+2]=x.z;
      velocity[3*node]=.25+i; velocity[3*node+1]=-.5*i; velocity[3*node+2]=.125;
    }
    if (kind==1) parents[1].parent_element_id=2126280;
    sc::Q4ParametricReference prepared;
    if (prepared.Initialize(View(reference),&parents[1],1).status!=sc::Q4ParametricStatus::Ok) return false;
    intrinsic=prepared.parent(0).intrinsic; area=prepared.parent(0).area;
    if (kind==0) {
      fixed[7]=1; inverse[7]=0;
      velocity[21]=velocity[22]=velocity[23]=0;
    }
    return true;
  }
};
struct T3Fixture : NodalData {
  sc::SurfaceTriangle parents[2]{{{0,2,4},101,201,0,0,sc::SurfaceInterpolation::kLinearTriangle},
                                 {{5,1,7},103,203,9,0,sc::SurfaceInterpolation::kLinearTriangle}};
  sc::T3MaterialMeasure intrinsic;
  sc::T3IntegrationLimits limits=T3Budget;
  sc::TranslationMassModel mass_model=sc::TranslationMassModel::kIsotropicLumped;
  TL_SURFACE_HD sc::T3NormalIntegrationInput Input() const {
    auto mass=Mass(); mass.model=mass_model;
    return {&intrinsic,{View(position),View(velocity),inverse,parents,2},mass,1,31,0,1,4};
  }
  bool Prepare(unsigned kind) {
    ResetMass(); reference[3*1+1]=2; reference[3*7+2]=1;
    if (kind==5) {
      reference[3*1+1]=0; reference[3*1+2]=2; reference[3*7]=1; reference[3*7+2]=0;
    }
    const double gaps[6][3]={{1,1,1},{1,-1,-1},{1,1,-1},{-1,-1,-1},{0,0,0},{1,-1,-1}};
    for (unsigned i=0;i<3;++i) {
      const unsigned node=parents[1].nodes[i];
      position[3*node]=gaps[kind][i]; position[3*node+1]=reference[3*node+1]; position[3*node+2]=reference[3*node+2];
      velocity[3*node]=.25+i; velocity[3*node+1]=-.5*i; velocity[3*node+2]=.125;
    }
    if (kind==0) { fixed[1]=1; inverse[1]=0; velocity[3]=velocity[4]=velocity[5]=0; }
    return sc::PrepareT3MaterialMeasure(View(reference),parents[1],&intrinsic)==sc::SurfaceMeasureStatus::Ok;
  }
};

// Independent original-edge derivative oracle in extended precision. No stored
// interval corner arithmetic or CPU integrator result enters this value.
inline long double Q4Density(const Q4Fixture& fixture,long double u,long double v) {
  constexpr int su[4]={1,-1,-1,1},sv[4]={1,1,-1,-1};
  long double a[3]{},b[3]{};
  for (unsigned n=0;n<4;++n) for (unsigned c=0;c<3;++c) {
    const long double x=fixture.reference[3*fixture.parents[1].nodes[n]+c];
    a[c]+=su[n]*(1+sv[n]*v)*x/4; b[c]+=sv[n]*(1+su[n]*u)*x/4;
  }
  const long double g[3]={a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
  return ::sqrtl(g[0]*g[0]+g[1]*g[1]+g[2]*g[2]);
}
struct Truth { long double force[4]{},potential=0; };
inline Truth Q4Truth(const Q4Fixture& fixture,unsigned kind) {
  Truth truth; const long double area=4*Q4Density(fixture,0,0),k=16,a=.03125L,b=.0078125L;
  if (kind==3) return truth;
  for (unsigned n=0;n<4;++n) {
    const bool plus=n==0 || n==3;
    if (kind==1) truth.force[n]=k*area*a*(plus ? 5.L/48 : 1.L/48);
    else truth.force[n]=k*area*(a/4+(kind==2 ? (plus ? b/12 : -b/12) : 0));
  }
  truth.potential=kind==1 ? k*area*a*a/12 : k*area*(a*a+(kind==2 ? b*b/3 : 0))/2;
  return truth;
}
inline Truth T3Truth(unsigned kind) {
  if (kind==0) return {{1.L/3,1.L/3,1.L/3,0},.5L};
  if (kind==1 || kind==5) return {{1.L/16,1.L/96,1.L/96,0},1.L/48};
  if (kind==2) return {{17.L/96,17.L/96,1.L/16,0},7.L/48};
  return {};
}
} // namespace parametric_cuda_test
