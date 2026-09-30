#pragma once
#include "../LayeredJ2CovarianceFixture.h"
#include "../../t3/T3ForcePortFixture.h"
#include "NativeLayeredReference.h"
#include <sstream>
#include <iomanip>
#include <string>

namespace layered_j2_test::recurrence {
namespace cv=covariance;
namespace oracle=tl::qualification::layered_native;
namespace nq=tl::qualification::qeph;
namespace nt=tl::qualification::t3;
using tl::math::Vec3;
constexpr double BaseDt=0x1p-20;
inline void RecordNumber(const std::string& name,double value) {
  std::ostringstream text;text<<std::setprecision(17)<<value;
  ::testing::Test::RecordProperty(name,text.str());
}
inline oracle::Law44 Law(bool rate=true) {
  return {tl::qualification::law44::YarisPlasticStrain.data(),tl::qualification::law44::YarisYieldStress.data(),
    tl::qualification::law44::YarisPlasticStrain.size(),rate?cv::source::SourceC:0,
    rate?cv::source::SourceP:0,rate?cv::source::Cutoff:0};
}
inline Vec3 Add(Vec3 a,Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline Vec3 Scale(Vec3 a,double b) { return {a.x*b,a.y*b,a.z*b}; }
inline Vec3 Cross(Vec3 a,Vec3 b) {return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline double Dot(Vec3 a,Vec3 b) {return a.x*b.x+a.y*b.y+a.z*b.z;}
// Analytic prescribed path: x(t)=c+R(t)[X+a(t)u(X)]. A node director is
// D(t)=R(t)exp(a(t)[b(X)]x), so omega=Omega+R(t)b(X)*a_dot exactly.
// Midpoint velocity includes the rotational transport term. Endpoint geometry
// and midpoint rates are intentionally distinct, matching the native API.
struct Path {
  bool rotating=false;
  unsigned refinement=1,step=0;
  double angular_speed=1800;
  double dt() const {return BaseDt/refinement;}
  double time() const {return step*dt();}
  static double Amplitude(double time) {
    if(time<=80*BaseDt) return time;
    if(time<=96*BaseDt) return 80*BaseDt;
    return 80*BaseDt-(time-96*BaseDt);
  }
  static double Rate(double time) {return time<80*BaseDt?1:time<96*BaseDt?0:-1;}
  static Vec3 Axis() {const double n=std::sqrt(14.);return {1/n,-2/n,3/n};}
  double Angle(double time) const {return rotating?angular_speed*std::max(0.,time-64*BaseDt):0.;}
  Vec3 Spin(double time) const {return Scale(Axis(),rotating&&time>64*BaseDt?angular_speed:0);}
  Vec3 Rotate(Vec3 x,double time) const {
    const auto a=Axis();const double angle=Angle(time),c=std::cos(angle),s=std::sin(angle);
    return Add(Add(Scale(x,c),Scale(Cross(a,x),s)),Scale(a,Dot(a,x)*(1-c)));
  }
  static Vec3 Shape(Vec3 x) {return cv::Path::Velocity(x,1);}
  Vec3 Position(Vec3 x,double time) const {return Rotate(Add(x,Scale(Shape(x),Amplitude(time))),time);}
  Vec3 Velocity(Vec3 x,double time) const {
    return Add(Cross(Spin(time),Position(x,time)),Rotate(Scale(Shape(x),Rate(time)),time));
  }
  Vec3 Omega(Vec3 x,double time) const {return Add(Spin(time),Rotate(cv::Path::Omega(x,Rate(time)),time));}
  q::PrescribedInterval Interval(const q::ReferenceInput& r,bool wrong_phase=false) const {
    q::PrescribedInterval in;in.base_time=time();in.dt=dt();in.sample_index=step+1;
    const double end=time()+dt(),mid=wrong_phase?end:time()+.5*dt();
    for(unsigned n=0;n<4;++n) {
      in.position_endpoint[n]=Position(r.position[n],end);
      in.velocity_midpoint[n]=Velocity(r.position[n],mid);
      in.omega_midpoint[n]=Omega(r.position[n],mid);
    }return in;
  }
  t::PrescribedInterval Interval(const t::ReferenceInput& r,bool wrong_phase=false) const {
    t::PrescribedInterval in;in.base_time=time();in.dt=dt();in.sample_index=step+1;
    const double end=time()+dt(),mid=wrong_phase?end:time()+.5*dt();
    for(unsigned n=0;n<3;++n) {
      in.position[n]=Position(r.position[n],end);in.velocity[n]=Velocity(r.position[n],mid);
      in.angular_velocity[n]=Omega(r.position[n],mid);
    }return in;
  }
};
inline void Sections(const sec::ShellLayeredJ2History& h,const oracle::Points& points) {
  for(unsigned n=0;n<3;++n) {
    SCOPED_TRACE(n);
    for(unsigned c=0;c<5;++c) cv::source::Close(h.point[n].stress[c],points[n][c],1e-8);
    cv::source::Close(h.point[n].plastic_strain,points[n][5],2e-14);
    cv::source::Close(h.point[n].filtered_rate_per_s,points[n][6],2e-11);
  }
}
inline void Diagnostics(const sec::ShellLayeredJ2Diagnostics& p,const oracle::SectionDiagnostics& n,
                        double area,double thickness) {
  cv::source::Close(p.plastic_work_density_increment*thickness*area,n.plastic_work_increment_j,1e-16);
  cv::source::Close(p.maximum_plastic_strain,n.max_plastic_strain,2e-14);
  cv::source::Close(p.mean_plastic_strain,n.mean_plastic_strain,2e-14);
  cv::source::Close(p.minimum_tangent_ratio,n.min_tangent_ratio,2e-14);
  cv::source::Close(p.mean_tangent_ratio,n.mean_tangent_ratio,2e-14);
  cv::source::Close(p.mean_yield_before_pa,n.mean_yield_pa,1e-8);
  cv::source::Close(p.last_point_yield_before_pa,n.last_yield_pa,1e-8);
}
} // namespace layered_j2_test::recurrence
