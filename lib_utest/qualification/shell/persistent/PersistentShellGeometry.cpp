#include "PersistentShellStorage.h"
#include <algorithm>
#include <cmath>

namespace tl::qualification::shell::detail {
namespace {
using V=std::array<double,3>;
V Sub(V a,V b){return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
V Scale(V a,double s){return {a[0]*s,a[1]*s,a[2]*s};}
double Dot(V a,V b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
V Cross(V a,V b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
double Norm(V a){return std::hypot(a[0],a[1],a[2]);}
bool Positive(double x){return std::isfinite(x)&&x>0;}
Report Bad(Status status,const char* message){return {status,message};}
}

Report ValidateConfiguration(const Configuration& c) {
  if(c.element_count<1||c.element_count>MaxElements||c.node_count<4||c.node_count>MaxNodes)
    return Bad(Status::ResourceLimit,"Fixture admits 1..2 elements and 4..8 physical nodes");
  if(c.geometry!=GeometryMode::FrozenReference&&c.geometry!=GeometryMode::Current)
    return Bad(Status::InvalidInput,"Unsupported geometry mode");
  const auto& p=c.stabilization;
  for(double x:{p.H1,p.H2,p.H3,p.SRH1,p.SRH2,p.SRH3,p.HVISC,p.HELAS,p.HVLIN})
    if(!std::isfinite(x)||x<0||x>10)return Bad(Status::InvalidInput,"Invalid stabilization coefficient");
  for(std::size_t e=0;e<c.element_count;++e){
    const auto& el=c.elements[e];
    if(!Positive(el.thickness)||el.thickness<1e-5||el.thickness>1)
      return Bad(Status::InvalidInput,"Thickness outside bounded fixture range");
    for(int i=0;i<4;++i){
      if(el.nodes[i]<0||static_cast<std::size_t>(el.nodes[i])>=c.node_count)
        return Bad(Status::InvalidInput,"Q4 node outside physical node space");
      for(int j=0;j<i;++j)if(el.nodes[i]==el.nodes[j])return Bad(Status::UnsupportedGeometry,"Repeated Q4 node");
    }
  }
  const std::size_t bytes=2*(ElementPlanes*c.element_count+17*c.node_count)*sizeof(double)+4*c.element_count*sizeof(int);
  if(bytes>c.max_device_bytes||bytes>1024*1024)return Bad(Status::ResourceLimit,"Persistent device budget exceeded");
  return {Status::Ok,{}};
}

Report ValidateKinematics(const Configuration& c,fea::HostNodalKinematicsView in,bool initializing) {
  if(in.node_count!=c.node_count||!in.position_xyz||!in.velocity_xyz||!in.angular_velocity_xyz)
    return Bad(Status::InvalidInput,"Invalid borrowed physical-node view");
  for(const double* a:{in.position_xyz,in.velocity_xyz,in.angular_velocity_xyz})
    for(std::size_t i=0;i<3*in.node_count;++i)if(!std::isfinite(a[i]))return Bad(Status::InvalidInput,"Nonfinite kinematics");
  for(std::size_t e=0;e<c.element_count;++e){
    std::array<V,4> x{};
    for(int i=0;i<4;++i)for(int j=0;j<3;++j)x[i][j]=in.position_xyz[3*c.elements[e].nodes[i]+j];
    double scale=0;
    for(int i=0;i<4;++i)for(int j=0;j<i;++j)scale=std::max(scale,Norm(Sub(x[i],x[j])));
    if(!std::isfinite(scale)||scale<1e-6||scale>1e6)return Bad(Status::UnsupportedGeometry,"Invalid Q4 geometry scale");
    V normal=Cross(Scale(Sub(x[1],x[0]),1/scale),Scale(Sub(x[2],x[0]),1/scale));
    const double length=Norm(normal);
    if(!(length>1e-10))return Bad(Status::UnsupportedGeometry,"Degenerate Q4 plane");
    normal=Scale(normal,1/length);
    if(std::abs(Dot(Scale(Sub(x[3],x[0]),1/scale),normal))>1e-10)
      return Bad(Status::UnsupportedGeometry,"Warped geometry awaits its separate qualification");
    for(int i=0;i<4;++i){
      const V a=Scale(Sub(x[(i+1)%4],x[i]),1/scale),b=Scale(Sub(x[(i+2)%4],x[(i+1)%4]),1/scale);
      if(!(Dot(Cross(a,b),normal)>1e-10))return Bad(Status::UnsupportedGeometry,"Q4 must be strictly convex");
      if(initializing&&c.geometry==GeometryMode::FrozenReference&&std::abs(Dot(a,b))>1e-10)
        return Bad(Status::UnsupportedGeometry,"Frozen-reference fixture starts from a rectangle");
    }
  }
  return {Status::Ok,{}};
}

Report ValidateState(const Configuration& c,const std::vector<double>& h,bool material) {
  if(!std::all_of(h.begin(),h.end(),[](double x){return std::isfinite(x);}))
    return Bad(Status::InvalidOutput,"Nonfinite trial output");
  const int ne=static_cast<int>(c.element_count);
  auto f=[&](int field,int e){return h[field*ne+e];};
  for(int e=0;e<ne;++e){
    if(!Positive(f(Area,e))||!Positive(f(Thickness,e))||!Positive(f(StepThickness,e))||
       !Positive(f(StepThicknessSquared,e)))return Bad(Status::InvalidOutput,"Nonpositive area or thickness");
    if(f(Off,e)!=(c.geometry==GeometryMode::FrozenReference?2:1))return Bad(Status::InvalidOutput,"Unexpected OFF lifecycle");
    V u{},v{},w{};
    for(int j=0;j<3;++j){u[j]=f(Frame+j,e);v[j]=f(Frame+3+j,e);w[j]=f(Frame+6+j,e);}
    if(std::abs(Norm(u)-1)>1e-10||std::abs(Norm(v)-1)>1e-10||Norm(Sub(Cross(u,v),w))>1e-10)
      return Bad(Status::InvalidOutput,"Improper local frame");
    if(!material)continue;
    for(int ip=0;ip<3;++ip){
      for(int j=0;j<5;++j)if(std::abs(f(Stress+3*j+ip,e))>Yield*.1)
        return Bad(Status::InvalidOutput,"Stress exceeds qualified elastic range");
      if(f(Pla+ip,e)!=0||f(Rate+ip,e)!=0||f(Dpla+ip,e)!=0||f(Temp+ip,e)!=Temperature)
        return Bad(Status::InvalidOutput,"Nonelastic point history in elastic fixture");
      for(int j=0;j<3;++j)if(f(Back+3*j+ip,e)!=0)return Bad(Status::InvalidOutput,"Unexpected backstress");
    }
  }
  return {Status::Ok,{}};
}

void Decode(const Configuration& c,const std::vector<double>& h,Snapshot& s) {
  const int ne=static_cast<int>(c.element_count),nn=static_cast<int>(c.node_count);
  auto f=[&](int field,int e){return h[field*ne+e];};
  s.element_count=c.element_count;s.node_count=c.node_count;
  for(int e=0;e<ne;++e){
    auto& out=s.elements[e];
    out.off=f(Off,e);out.area=f(Area,e);out.thickness=f(Thickness,e);out.step_thickness=f(StepThickness,e);
    out.px1=f(Px1,e);out.px2=f(Px2,e);out.py1=f(Py1,e);out.py2=f(Py2,e);out.vhx=f(Vhx,e);out.vhy=f(Vhy,e);
    for(int j=0;j<9;++j)out.frame[j]=f(Frame+j,e);
    for(int j=0;j<6;++j)out.reference_coordinates[j]=f(Smstr+j,e);
    for(int j=0;j<8;++j)out.generalized_strain[j]=f(Gstr+j,e);
    for(int j=0;j<5;++j){out.normalized_force[j]=f(Force+j,e);out.hour[j]=f(Hour+j,e);}
    for(int j=0;j<3;++j)out.normalized_moment[j]=f(Moment+j,e);
    for(int j=0;j<2;++j)out.work[j]=f(Energy+j,e);
    for(int ip=0;ip<3;++ip){
      auto& p=out.points[ip];
      for(int j=0;j<5;++j)p.stress[j]=f(Stress+3*j+ip,e);
      p.plastic_strain=f(Pla+ip,e);p.plastic_rate=f(Rate+ip,e);p.plastic_increment=f(Dpla+ip,e);p.temperature=f(Temp+ip,e);
      for(int j=0;j<3;++j)p.backstress[j]=f(Back+3*j+ip,e);
    }
  }
  const int base=ElementPlanes*ne;
  for(int i=0;i<3*nn;++i){s.position[i]=h[base+i];s.velocity[i]=h[base+3*nn+i];s.angular_velocity[i]=h[base+6*nn+i];}
  for(int i=0;i<nn;++i)for(int j=0;j<3;++j){s.force[3*i+j]=h[base+(9+j)*nn+i];s.couple[3*i+j]=h[base+(12+j)*nn+i];}
}
}  // namespace tl::qualification::shell::detail
