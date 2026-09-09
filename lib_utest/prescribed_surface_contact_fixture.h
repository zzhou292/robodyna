#pragma once

#include "lib_src/collision/PrescribedSurfaceContact.h"
#include "lib_utest/q4_prescribed_contact_fixture.h"

namespace prescribed_surface_test {
namespace sc=tlfea::contact;
namespace geometry=q4_planar_test;
struct Fixture {
  std::array<double,15> reference{},base{},endpoint{},velocity{};
  q4_prescribed_test::PhysicalMass<5> mass;
  sc::SurfaceQ4 quad{{2,0,1,3},101,201,0,0},quad_endpoint=quad;
  sc::SurfaceTriangle triangle{{2,3,4},103,203,1,0,sc::SurfaceInterpolation::kLinearTriangle};
  sc::SurfaceTriangle triangle_endpoint=triangle;
  sc::Q4ParametricReference q4_reference;
  sc::T3MaterialMeasure t3_reference;
  sc::PlanarWallGeometry wall;
  sc::PrescribedSurfaceConfig config;
  q4_prescribed_test::Scratch scratch;
  Fixture() {
    const double y[5]={-1,-1,0,0,1},z[5]={.5,-.5,.5,-.5,0};
    for (unsigned n=0;n<5;++n) {
      reference[3*n+1]=y[n]; reference[3*n+2]=z[n]; velocity[3*n]=.125*n;
      velocity[3*n+1]=n+.25; velocity[3*n+2]=-.25*n;
    }
    base=reference; endpoint=reference;
    for (unsigned n=0;n<5;++n) endpoint[3*n]=.03125;
    const auto old=q4_prescribed_test::Config();
    config={old.stiffness_per_area,old.maximum_penetration,old.sweep.exposed_clearance,old.integration,{1e-10,1e-11}};
  }
  sc::VectorView View(const std::array<double,15>& x) const { return {x.data(),5,3,1}; }
  bool Initialize(unsigned variant=0) {
    const auto mesh=geometry::Square(variant);
    return wall.Initialize(mesh.view()).status==sc::PlanarContactStatus::Ok &&
        q4_reference.Initialize(View(reference),&quad,1).status==sc::Q4ParametricStatus::Ok &&
        sc::PrepareT3MaterialMeasure(View(reference),triangle,&t3_reference)==sc::SurfaceMeasureStatus::Ok;
  }
  std::array<sc::PrescribedSurfaceParent,2> Requests() const {
    std::array<sc::PrescribedSurfaceParent,2> requests;
    requests[0].family=sc::PrescribedSurfaceFamily::Q4CenterAreaUniformNatural;
    requests[0].q4={&q4_reference,0,&quad,&quad_endpoint};
    requests[1].family=sc::PrescribedSurfaceFamily::T3NativeLinear;
    requests[1].t3={&t3_reference,&triangle,&triangle_endpoint};
    return requests;
  }
  sc::PrescribedSurfaceInput Input(const sc::PrescribedSurfaceParent* requests,unsigned count=2) const {
    return {View(base),View(velocity),View(endpoint),View(velocity),mass.view(),requests,count};
  }
  sc::PrescribedSurfaceReport Evaluate(sc::PrescribedSurfaceResult* result) {
    const auto requests=Requests();
    return sc::IntegratePrescribedSurfaceContact(wall,Input(requests.data()),config,7,scratch.view(),result);
  }
};
inline std::array<double,5> Forces(const sc::PrescribedSurfaceResult& result) {
  std::array<double,5> forces{};
  for (unsigned p=0;p<result.parent_count;++p) {
    const auto& parent=result.parents[p];
    if (parent.family==sc::PrescribedSurfaceFamily::Q4CenterAreaUniformNatural) {
      for (unsigned n=0;n<4;++n) forces[parent.q4.integration.nodal.nodes[n]]+=parent.q4.integration.nodal.forces[n].x;
    } else {
      for (unsigned n=0;n<3;++n) forces[parent.t3.nodal.nodes[n]]+=parent.t3.nodal.forces[n].x;
    }
  }
  return forces;
}
} // namespace prescribed_surface_test
