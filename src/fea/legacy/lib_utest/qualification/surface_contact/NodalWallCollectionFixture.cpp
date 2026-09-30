#include "NodalWallCollectionFixture.h"

namespace nodal_wall_collection_test {
namespace {
tl::math::Vec3 Position(const Fixture& f,unsigned n) {
  return {f.reference[3*n],f.reference[3*n+1],f.reference[3*n+2]};
}
template<class Input> void Material(Input& in) {
  in.density=1024; in.thickness=1./32; in.young_modulus=2e6; in.poisson_ratio=.3;
}
}
bool Fixture::Prepare(unsigned count,bool reverse) {
  if(started || (count!=94 && count!=128)) return false;
  started=true;
  for(unsigned r=0;r<8;++r) for(unsigned c=0;c<16;++c) {
    const unsigned n=16*r+c;
    reference[3*n+1]=(static_cast<double>(c)-7.5)/16;
    reference[3*n+2]=(static_cast<double>(r)-3.5)/8;
    q[4*n]=1;
  }
  unsigned split=0;
  for(unsigned r=0;r<7;++r) for(unsigned c=0;c<15;++c) {
    // Fourteen nonadjacent interior omissions retain all 128 physical nodes.
    if(count==94 && (r==1 || r==3) && c%2==1) continue;
    const unsigned a=16*r+c,b=a+1,d=a+16,e=d+1;
    const bool triangle=count==94 ? (r==6 && c>=12) : split<23;
    if(triangle) {
      ++split;
      const unsigned local[2][3]{{a,b,e},{a,e,d}};
      for(unsigned t=0;t<2;++t) {
        auto& p=triangles[t3_count++]; p.parent_element_id=WideId+2*(15*r+c)+t+1;
        p.feature_id=p.parent_element_id+1000; p.interpolation=sc::SurfaceInterpolation::kLinearTriangle;
        for(unsigned l=0;l<3;++l) p.nodes[l]=local[t][l];
      }
    } else {
      auto& p=quads[q4_count++]; p.parent_element_id=WideId+2*(15*r+c)+1;
      p.feature_id=p.parent_element_id+1000;
      const unsigned local[4]{a,b,e,d}; for(unsigned l=0;l<4;++l) p.nodes[l]=local[l];
    }
  }
  parent_count=q4_count+t3_count;
  if(parent_count!=count) return false;
  // Native mass/J sums deliberately use Q4 then T3 producer order; contact
  // weights independently sort physical parent identity before reduction.
  for(unsigned p=0;p<q4_count;++p) {
    fe::qeph::ReferenceInput in; Material(in);
    // QEPH's native source-node slots are uint32. Both families name the
    // shared nodes identically; wide parent/feature IDs remain independent.
    for(unsigned l=0;l<4;++l) { const unsigned n=quads[p].nodes[l]; in.position[l]=Position(*this,n); in.node_ids[l]=1000+n; }
    fe::qeph::ReferenceData out;
    if(fe::qeph::InitializeReference(in,out)!=fe::qeph::Status::kSuccess ||
       references[p].Initialize(View(reference),&quads[p],1).status!=sc::Q4ParametricStatus::Ok) return false;
    input[p]={&references[p],0,nullptr};
    for(unsigned l=0;l<4;++l) {
      const unsigned n=quads[p].nodes[l]; mass[n]+=out.nodal_mass[l]; inertia[n]+=out.isotropic_inertia[l]; area[n]+=1.L/512;
    }
  }
  for(unsigned p=0;p<t3_count;++p) {
    fe::t3::ReferenceInput in; Material(in);
    for(unsigned l=0;l<3;++l) { const unsigned n=triangles[p].nodes[l]; in.position[l]=Position(*this,n); in.node_ids[l]=1000+n; }
    fe::t3::ReferenceData out;
    if(fe::t3::InitializeReference(in,out)!=fe::t3::Status::kSuccess ||
       sc::PrepareT3MaterialMeasure(View(reference),triangles[p],&measures[p])!=sc::SurfaceMeasureStatus::Ok) return false;
    input[q4_count+p]={nullptr,0,&measures[p]};
    for(unsigned l=0;l<3;++l) {
      const unsigned n=triangles[p].nodes[l]; mass[n]+=out.nodal_mass[l]; inertia[n]+=out.isotropic_inertia[l]; area[n]+=1.L/768;
    }
  }
  for(unsigned n=0;n<Nodes;++n) {
    if(!(mass[n]>0 && inertia[n]>0)) return false;
    inverse[n]=1/mass[n]; inverse_j[n]=1/inertia[n];
  }
  if(reverse) std::reverse(input.begin(),input.begin()+count);
  if(weights.Initialize(Nodes,input.data(),count).status!=sc::NodalWallStatus::Ok || weights.node_count()!=Nodes) return false;
  x=reference;
  for(unsigned n=0;n<Nodes;++n) { x[3*n]=(static_cast<int>(n%7)-3)/4096.; v[3*n]=(static_cast<int>(n%5)-2)/64.; }
  x[3*(Nodes-1)]=1./32;
  return true;
}
sc::NodalWallDeviceConfig Fixture::Config(fe::NodalStamp stamp) const {
  if(!stamp.owner_id) {
    stamp.owner_id=77; stamp.node_count=Nodes; stamp.fixed_dt=Step; stamp.has_rotations=true;
    stamp.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  }
  sc::NodalWallDeviceConfig c; c.owner=stamp; c.configuration_id=991;
  c.qualification_id=Qualification; c.wall_binding_id=771; c.law={0,16,.5,ForceBudget,EnergyBudget}; return c;
}
bool Fixture::Owner(fe::FENodalState& owner) const {
  fe::NodalStateConfig c; c.node_count=Nodes; c.fixed_dt=Step;
  c.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  const auto r=owner.Initialize(c,{x.data(),v.data(),omega.data(),Nodes,q.data()},inverse.data(),
                               {fixed.data(),rotation_fixed.data(),inverse_j.data()});
  EXPECT_EQ(r.status,fe::NodalStatus::Ok)<<r.message; return r.status==fe::NodalStatus::Ok;
}
bool Fixture::Bind(fe::FENodalState& owner,sc::NodalWallContactDevice& contact,sc::NodalWallDeviceConfig c) const {
  c.owner=owner.accepted(); const auto r=contact.Initialize(c,wall.view(),weights,View(x),inverse.data(),fixed.data(),motion);
  EXPECT_EQ(r.status,Code::Ok)<<r.message; return r.status==Code::Ok;
}
sc::NodalWallResult Fixture::Host(const Vectors& positions,const Vectors& velocities,
    std::uint64_t epoch,std::uint64_t attempt,sc::NodalWallConfig c) const {
  std::array<std::uint8_t,Nodes> bits{}; for(unsigned n=0;n<Nodes;++n) bits[n]=fixed[n]==7;
  sc::NodalWallResult out;
  EXPECT_EQ(sc::EvaluateNodalWallContact(weights,View(positions),View(velocities),
      {inverse.data(),bits.data(),Nodes,epoch,sc::TranslationMassModel::kIsotropicLumped},c,attempt,&out).status,
      sc::NodalWallStatus::Ok);
  return out;
}
void CheckLoads(const Fixture& f,const sc::NodalWallDeviceResults& result,const Vectors& x,const Vectors& v,double k) {
  long double force=0,potential=0,my=0,mz=0,power=0;
  for(unsigned n=0;n<Nodes;++n) {
    const long double depth=std::max(0.L,static_cast<long double>(x[3*n]));
    const long double fn=f.fixed[n]?0:k*f.area[n]*depth,en=fn*depth/2;
    const auto& node=result.nodes[n]; EXPECT_EQ(node.node,n);
    EXPECT_LE(static_cast<long double>(node.force.lower),fn); EXPECT_GE(static_cast<long double>(node.force.upper),fn);
    EXPECT_LE(static_cast<long double>(node.potential.lower),en); EXPECT_GE(static_cast<long double>(node.potential.upper),en);
    force+=fn; potential+=en; my+=fn*x[3*n+2]; mz-=fn*x[3*n+1]; power-=fn*v[3*n];
    EXPECT_GT(result.wall_face[n],WideId);
  }
  Near(result.diagnostics.resultant.value,force); Near(result.diagnostics.potential.value,potential);
  Near(result.diagnostics.wall_moment.y,my); Near(result.diagnostics.wall_moment.z,mz);
  Near(result.diagnostics.surface_power,power);
}
Snapshot Read(fe::FENodalState& owner) {
  Snapshot out;
  EXPECT_EQ(owner.CopyAccepted({out.x.data(),out.v.data(),Nodes,out.q.data(),out.omega.data(),
                                out.reaction.data(),out.couple.data()},&out.stamp).status,fe::NodalStatus::Ok);
  return out;
}
void Same(const Snapshot& a,const Snapshot& b) {
  EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.v,b.v); EXPECT_EQ(a.q,b.q); EXPECT_EQ(a.omega,b.omega);
  EXPECT_EQ(a.reaction,b.reaction); EXPECT_EQ(a.couple,b.couple);
  EXPECT_EQ(a.stamp.owner_id,b.stamp.owner_id); EXPECT_EQ(a.stamp.epoch,b.stamp.epoch);
  EXPECT_EQ(a.stamp.time,b.stamp.time); EXPECT_EQ(a.stamp.velocity_time,b.stamp.velocity_time);
  EXPECT_EQ(a.stamp.velocity_phase,b.stamp.velocity_phase); EXPECT_EQ(a.stamp.reaction_kick_dt,b.stamp.reaction_kick_dt);
}
Forces ReadForces(const fe::NodalAssemblyView& view) {
  Forces out{}; const double* fields[]{view.forces.force_x,view.forces.force_y,view.forces.force_z,
                                     view.forces.couple_x,view.forces.couple_y,view.forces.couple_z};
  for(unsigned c=0;c<6;++c) EXPECT_EQ(cudaMemcpyAsync(out.data()+c*Nodes,fields[c],Nodes*sizeof(double),
      cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess); return out;
}
Snapshot ReadPrepared(const fe::NodalPreparedView& p) {
  Snapshot out;
  EXPECT_EQ(cudaMemcpyAsync(out.x.data(),p.kinematics.position_xyz,3*Nodes*sizeof(double),cudaMemcpyDeviceToHost,p.stream),cudaSuccess);
  EXPECT_EQ(cudaMemcpyAsync(out.v.data(),p.kinematics.velocity_xyz,3*Nodes*sizeof(double),cudaMemcpyDeviceToHost,p.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(p.stream),cudaSuccess); return out;
}
} // namespace nodal_wall_collection_test
