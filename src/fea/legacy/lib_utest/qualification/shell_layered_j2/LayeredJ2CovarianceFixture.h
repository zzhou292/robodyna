#pragma once
#include "LayeredJ2Fixture.h"
#include "lib_utest/qualification/qeph/QephForceFixture.h"
#include "lib_utest/qualification/t3/T3PortFixture.h"
#include "lib_utest/qualification/native/law44/NativeRateTestSupport.h"

namespace layered_j2_test::covariance {
namespace frame=qeph_kinematics_test;
namespace source=tl::qualification::law44::rate_test;
using tl::math::Vec3;
using frame::Field;
constexpr double Tolerance=frame::kCovariance; // Existing 2e-11 world-covariance budget.
constexpr double Dt=1.e-6;
constexpr unsigned PreloadSteps=64;

template<class ReferenceInput>
void SourceMaterial(ReferenceInput& r,const sec::PointParameters& p) {
  r.young_modulus=p.young_pa; r.poisson_ratio=p.poisson_ratio;
  r.density=p.density_kg_m3; r.thickness=source::Thickness;
}
inline Vec3 TransformPoint(Vec3 x) {
  const auto p=frame::Rotate(frame::CommonRotation(),x);
  return {p.x+1.25,p.y-.75,p.z+.5}; // Existing QEPH fixture transform.
}
inline t::ReferenceInput Transform(t::ReferenceInput r) {
  for(auto& x:r.position) x=TransformPoint(x);
  return r;
}
inline t::PrescribedInterval Transform(t::PrescribedInterval in) {
  for(unsigned n=0;n<3;++n) {
    in.position[n]=TransformPoint(in.position[n]);
    in.velocity[n]=frame::Rotate(frame::CommonRotation(),in.velocity[n]);
    in.angular_velocity[n]=frame::Rotate(frame::CommonRotation(),in.angular_velocity[n]);
  }
  return in;
}

// A shallow membrane/bending prescribed path. Current positions and midpoint
// translational velocities come from the same piecewise-linear amplitude;
// angular fields are the associated plate rotation rates. The superposed
// world transform is constant, so it introduces no fictitious spin increment.
struct Path {
  double time=0,amplitude=0;
  unsigned step=0;
  void Advance(double rate) { time+=Dt; amplitude+=rate*Dt; ++step; }
  Vec3 Position(Vec3 x) const {
    return {x.x+amplitude*(100*x.x+25*x.y),x.y-amplitude*30*x.y,
      x.z+amplitude*(60000*x.x*x.x-10000*x.y*x.y+15000*x.x*x.y)};
  }
  static Vec3 Velocity(Vec3 x,double rate) {
    return {rate*(100*x.x+25*x.y),-rate*30*x.y,
      rate*(60000*x.x*x.x-10000*x.y*x.y+15000*x.x*x.y)};
  }
  static Vec3 Omega(Vec3 x,double rate) {
    return {rate*(20000*x.y-15000*x.x),rate*(120000*x.x+15000*x.y),0};
  }
  q::PrescribedInterval Interval(const q::ReferenceInput& r,double rate) const {
    q::PrescribedInterval in; in.base_time=time; in.dt=Dt; in.sample_index=step+1;
    auto next=*this; next.Advance(rate);
    for(unsigned n=0;n<4;++n) {
      in.position_endpoint[n]=next.Position(r.position[n]);
      in.velocity_midpoint[n]=Velocity(r.position[n],rate);
      in.omega_midpoint[n]=Omega(r.position[n],rate);
    }
    return in;
  }
  t::PrescribedInterval Interval(const t::ReferenceInput& r,double rate) const {
    t::PrescribedInterval in; in.base_time=time; in.dt=Dt; in.sample_index=step+1;
    auto next=*this; next.Advance(rate);
    for(unsigned n=0;n<3;++n) {
      in.position[n]=next.Position(r.position[n]);
      in.velocity[n]=Velocity(r.position[n],rate);
      in.angular_velocity[n]=Omega(r.position[n],rate);
    }
    return in;
  }
};
inline double ContinuationRate(unsigned step) { return step<16?1.:step<32?0.:-1.; }

inline void Sections(const sec::ShellLayeredJ2History& a,const sec::ShellLayeredJ2History& b,
                     const sec::PointParameters& p) {
  for(unsigned layer=0;layer<3;++layer) {
    SCOPED_TRACE(layer);
    for(unsigned c=0;c<5;++c)
      Field(a.point[layer].stress[c],b.point[layer].stress[c],p.young_pa,Tolerance,"point_stress",c);
    Field(a.point[layer].plastic_strain,b.point[layer].plastic_strain,1.,Tolerance,"point_PLA");
    Field(a.point[layer].filtered_rate_per_s,b.point[layer].filtered_rate_per_s,1./Dt,Tolerance,"point_filter");
  }
}
inline void SectionDiagnostics(const sec::ShellLayeredJ2Diagnostics& a,
    const sec::ShellLayeredJ2Diagnostics& b,const sec::PointParameters& p) {
  Field(a.plastic_work_density_increment,b.plastic_work_density_increment,p.young_pa,Tolerance,"plastic_work_density");
  Field(a.maximum_plastic_strain,b.maximum_plastic_strain,1.,Tolerance,"maximum_PLA");
  Field(a.mean_plastic_strain,b.mean_plastic_strain,1.,Tolerance,"mean_PLA");
  Field(a.minimum_tangent_ratio,b.minimum_tangent_ratio,1.,Tolerance,"minimum_ETSE");
  Field(a.mean_tangent_ratio,b.mean_tangent_ratio,1.,Tolerance,"mean_ETSE");
  Field(a.mean_yield_before_pa,b.mean_yield_before_pa,p.young_pa,Tolerance,"mean_SIGY");
  Field(a.last_point_yield_before_pa,b.last_point_yield_before_pa,p.young_pa,Tolerance,"last_SIGY");
}
template<class A,class B>
void OrdinaryHistory(const A& a,const B& b,const sec::PointParameters& p,double length) {
  for(unsigned i=0;i<5;++i) {
    Field(a.stress[i],b.stress[i],p.young_pa,Tolerance,"total_stress",i);
    Field(a.material_stress[i],b.material_stress[i],p.young_pa,Tolerance,"material_stress",i);
  }
  for(unsigned i=0;i<3;++i)
    Field(a.bending_stress[i],b.bending_stress[i],p.young_pa*source::Thickness/length,Tolerance,"moment",i);
  for(unsigned i=0;i<8;++i)
    Field(a.strain_curvature[i],b.strain_curvature[i],i<5?1.:1./length,Tolerance,"strain_curvature",i);
  Field(a.thickness,b.thickness,source::Thickness,Tolerance,"accepted_thickness");
  Field(a.internal_work[0],b.internal_work[0],p.young_pa*source::Thickness*length*length,Tolerance,"membrane_work");
  Field(a.internal_work[1],b.internal_work[1],p.young_pa*std::pow(source::Thickness,3),Tolerance,"bending_work");
  EXPECT_EQ(a.active,b.active);
}
template<unsigned Nodes,class Trial>
void Loads(const Trial& transformed,const Trial& original,const sec::PointParameters& p,double length) {
  const double force_scale=p.young_pa*source::Thickness*length;
  for(unsigned n=0;n<Nodes;++n) {
    Field(transformed.internal_force[n],frame::Rotate(frame::CommonRotation(),original.internal_force[n]),
      force_scale,Tolerance,"rotated_force",n);
    Field(transformed.internal_couple[n],frame::Rotate(frame::CommonRotation(),original.internal_couple[n]),
      force_scale*length,Tolerance,"rotated_couple",n);
  }
  EXPECT_EQ(transformed.proposed_history.stamp().time,original.proposed_history.stamp().time);
  EXPECT_EQ(transformed.proposed_history.stamp().sample_index,original.proposed_history.stamp().sample_index);
}
inline void Yielded(const sec::ShellLayeredJ2History& h) {
  const double outer=std::max(h.point[0].plastic_strain,h.point[2].plastic_strain);
  EXPECT_GT(outer,1.e-3);
  EXPECT_GT(std::abs(h.point[0].plastic_strain-h.point[2].plastic_strain),1.e-5);
  EXPECT_GT(h.point[0].filtered_rate_per_s,0.);
  EXPECT_GT(std::abs(h.point[0].stress[0])+std::abs(h.point[2].stress[0]),1.e6);
}
} // namespace layered_j2_test::covariance
