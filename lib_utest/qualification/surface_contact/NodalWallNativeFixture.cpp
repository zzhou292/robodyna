#include "NodalWallNativeFixture.h"
#include "../t3/mixed_binding/ShellBatchBindingFixture.h"
#include <algorithm>

namespace nodal_wall_native_test {
namespace oracle=shell_binding_test;
namespace wide=tl::qualification::t3::test;
namespace {
tl::math::Vec3 Position(const NativeFixture& f,unsigned n) {
  return {f.reference[3*n],f.reference[3*n+1],f.reference[3*n+2]};
}
fe::t3::ReferenceInput Triangle(const NativeFixture& f,unsigned p) {
  fe::t3::ReferenceInput in;
  in.density=1024; in.thickness=1./32; in.young_modulus=2e6; in.poisson_ratio=.3;
  for(unsigned l=0;l<3;++l) {
    const auto n=f.triangles[p].nodes[l]; in.position[l]=Position(f,n);
    in.node_ids[l]=f.layout==Layout::Mixed && n<4?100+n:oracle::WideId+n;
  }
  return in;
}
long double Area(const NativeFixture& f,const std::uint32_t* nodes,unsigned arity) {
  const auto a=Position(f,nodes[0]),b=Position(f,nodes[1]),c=Position(f,nodes[2]);
  long double area=wide::CrossNorm(wide::Difference(b,a),wide::Difference(c,a))/2;
  if(arity==4) area+=wide::CrossNorm(wide::Difference(c,a),wide::Difference(Position(f,nodes[3]),a))/2;
  return area;
}
void Encloses(sc::Q4CertifiedIntegral c,long double truth) {
  // Independent long-double world-area and polynomial load oracle. Division
  // by three is rounded in long double; the production binary64 certificates
  // must enclose this more precise calculation without a widened test budget.
  EXPECT_LE(static_cast<long double>(c.lower),truth);
  EXPECT_GE(static_cast<long double>(c.upper),truth);
  EXPECT_LE(std::abs(static_cast<long double>(c.value)-truth),c.error);
}
} // namespace
NativeFixture::NativeFixture(Layout selected,bool edge_on,unsigned cyclic):layout(selected) {
  n=layout==Layout::SingleT3?4:5; count=layout==Layout::SingleT3?1:2;
  reference.fill(0); x.fill(0); inverse.fill(0); inverse_j.fill(0);
  const double mixed[5][2]{{0,0},{1,0},{1,1},{0,1},{1.75,.25}};
  const double tri[5][2]{{0,0},{0,0},{1,0},{.25,.75},{1.5,1}};
  const double scale=edge_on?.125:1;
  for(unsigned i=first();i<n;++i) {
    const auto* uv=layout==Layout::Mixed?mixed[i]:tri[i];
    const double a=scale*(uv[0]-.875),b=scale*(uv[1]-.5);
    reference[3*i]=edge_on?a:0; reference[3*i+1]=edge_on?0:a; reference[3*i+2]=b;
  }
  if(first()) {
    // No physical parent references global 0. It is a valid fully fixed owner
    // sentinel: positive gap would reject any accidental fourth T3 share.
    fixed[0]=7; rotation_fixed[0]=1; reference[1]=-.95;
  }
  parents[0]={{0,1,2,3},201,101,0,0};
  const std::uint32_t nodes[2][3]{{1,2,3},{2,4,3}};
  for(unsigned p=0;p<2;++p) {
    triangles[p].parent_element_id=layout==Layout::Mixed?102:101+p;
    triangles[p].feature_id=202+p;
    triangles[p].interpolation=sc::SurfaceInterpolation::kLinearTriangle;
    for(unsigned l=0;l<3;++l) triangles[p].nodes[l]=nodes[p][(l+cyclic)%3];
  }
  if(layout==Layout::Mixed) {
    const std::uint32_t t[3]{1,4,2};
    for(unsigned l=0;l<3;++l) triangles[0].nodes[l]=t[(l+cyclic)%3];
    const std::uint32_t qnodes[4]{0,1,2,3};
    for(unsigned l=0;l<4;++l) parents[0].nodes[l]=qnodes[(l+cyclic)%4];
  }
  x=reference;
  double maximum=-std::numeric_limits<double>::infinity();
  for(unsigned i=first();i<n;++i) maximum=std::max(maximum,x[3*i]);
  for(unsigned i=first();i<n;++i) x[3*i]+=1./32-maximum;
  if(first()) x[0]=.25;
}
void NativeFixture::Depth(double depth) {
  for(unsigned i=first();i<n;++i) x[3*i]=depth;
}
bool NativeFixture::PrepareNative(bool reverse) {
  if(preparation_started_) return false;
  preparation_started_=true;
  const bool mixed=layout==Layout::Mixed;
  if(mixed) {
    oracle::Input in=oracle::Edge();
    for(unsigned l=0;l<4;++l) {
      const auto n=parents[0].nodes[l]; in.qeph_nodes[l]=n;
      in.qeph.position[l]=Position(*this,n); in.qeph.node_ids[l]=100+n;
    }
    in.t3=Triangle(*this,0);
    for(unsigned l=0;l<3;++l) in.t3_nodes[l]=triangles[0].nodes[l];
    fe::ShellBatchBinding binding;
    const auto r=binding.Initialize(in); EXPECT_EQ(r.status,fe::ShellBindingStatus::Success)<<r.message;
    if(r.status!=fe::ShellBindingStatus::Success) return false;
    const auto truth=oracle::Truth(in);
    for(unsigned i=0;i<n;++i) {
      mass[i]=binding.nodes()[i].native.mass; total_j[i]=binding.nodes()[i].native.isotropic_inertia;
      truth_mass[i]=truth[i].mass; truth_j[i]=truth[i].total;
    }
    if(ref.Initialize(View(reference),parents,1).status!=sc::Q4ParametricStatus::Ok) return false;
  } else for(unsigned p=0;p<count;++p) {
    const auto in=Triangle(*this,p); fe::t3::ReferenceData data;
    const auto status=fe::t3::InitializeReference(in,data); EXPECT_EQ(status,fe::t3::Status::kSuccess);
    if(status!=fe::t3::Status::kSuccess) return false;
    const auto truth=wide::Independent(oracle::NativeInput(in));
    for(unsigned l=0;l<3;++l) {
      const auto i=triangles[p].nodes[l]; mass[i]+=data.nodal_mass[l]; total_j[i]+=data.isotropic_inertia[l];
      truth_mass[i]+=truth.mass*truth.weight[l]; truth_j[i]+=truth.total*truth.weight[l];
    }
  }
  sc::NodalWallParentInput inputs[2]{};
  for(unsigned p=0;p<count;++p) {
    const bool q4=mixed && p==0; const unsigned ti=mixed?0:p;
    if(q4) inputs[p]={&ref,0,nullptr};
    else {
      if(sc::PrepareT3MaterialMeasure(View(reference),triangles[ti],&measures[ti])!=sc::SurfaceMeasureStatus::Ok) return false;
      inputs[p]={nullptr,0,&measures[ti]};
    }
    const auto* nodes=q4?parents[0].nodes:triangles[ti].nodes; const unsigned arity=q4?4:3;
    // Fixture IDs are strictly ascending (Q4 then T3 when mixed). Reversing
    // the input array below therefore retains this sorted output indexing.
    parent_area[p]=Area(*this,nodes,arity);
    for(unsigned l=0;l<arity;++l) node_area[nodes[l]]+=parent_area[p]/arity;
  }
  if(reverse && count==2) std::swap(inputs[0],inputs[1]);
  for(unsigned i=first();i<n;++i) { inverse[i]=1/mass[i]; inverse_j[i]=1/total_j[i]; }
  return weights.Initialize(n,inputs,count).status==sc::NodalWallStatus::Ok;
}
void CheckMass(const NativeFixture& f) {
  for(unsigned i=f.first();i<f.n;++i) {
    SCOPED_TRACE(i); oracle::Near(f.mass[i],f.truth_mass[i]); oracle::Near(f.total_j[i],f.truth_j[i]);
    EXPECT_EQ(f.inverse[i],1/f.mass[i]); EXPECT_EQ(f.inverse_j[i],1/f.total_j[i]);
  }
  if(f.first()) { EXPECT_EQ(f.inverse[0],0); EXPECT_EQ(f.inverse_j[0],0); }
}
void CheckIndependent(const NativeFixture& f,const sc::NodalWallDeviceResults& r,
    const std::array<double,3*Capacity>& x,const std::array<double,3*Capacity>& v,double stiffness) {
  long double force=0,potential=0,my=0,mz=0,power=0;
  for(unsigned j=0;j<r.diagnostics.node_count;++j) {
    const auto i=r.nodes[j].node;
    const long double d=std::max(0.L,static_cast<long double>(x[3*i]));
    const long double fi=f.fixed[i]?0:stiffness*f.node_area[i]*d,ui=fi*d/2;
    Encloses(r.nodes[j].force,fi); Encloses(r.nodes[j].potential,ui);
    Near(r.nodes[j].force_world.x,-fi); Near(r.nodes[j].surface_power,-fi*v[3*i]);
    force+=fi; potential+=ui; my+=fi*x[3*i+2]; mz-=fi*x[3*i+1]; power-=fi*v[3*i];
  }
  Encloses(r.diagnostics.resultant,force); Encloses(r.diagnostics.potential,potential);
  Near(r.diagnostics.wall_moment.y,my); Near(r.diagnostics.wall_moment.z,mz); Near(r.diagnostics.surface_power,power);
  for(unsigned p=0;p<f.count;++p) {
    const auto& parent=f.weights.parent(p); long double pf=0,pu=0;
    for(unsigned l=0;l<parent.arity;++l) {
      const auto i=parent.nodes[l]; const long double d=std::max(0.L,static_cast<long double>(x[3*i]));
      const long double fi=f.fixed[i]?0:stiffness*f.parent_area[p]/parent.arity*d;
      Encloses(r.parents[p].force[l],fi); pf+=fi; pu+=fi*d/2;
    }
    Encloses(r.parents[p].resultant,pf); Encloses(r.parents[p].potential,pu);
    if(parent.arity==3) Same(r.parents[p].force[3],sc::Q4CertifiedIntegral{});
  }
}
void CheckFaces(const NativeFixture& f,const sc::NodalWallDeviceResults& result,const std::array<double,3*Capacity>& x) {
  sc::PlanarWallGeometry wall; ASSERT_EQ(wall.Initialize(f.wall.view()).status,sc::PlanarContactStatus::Ok);
  for(unsigned j=0;j<result.diagnostics.node_count;++j) {
    const auto i=result.nodes[j].node; unsigned face=UINT32_MAX; sc::TrianglePointGeometry point;
    ASSERT_EQ(sc::planar_detail::FindOwner({0,x[3*i+1],x[3*i+2]},wall.faces().data(),
        wall.faces().size(),wall.tolerance(),&face,&point),sc::Status::kOk);
    ASSERT_NE(face,UINT32_MAX); EXPECT_EQ(result.wall_face[j],wall.faces()[face].geometry.face_id);
  }
}
} // namespace nodal_wall_native_test
