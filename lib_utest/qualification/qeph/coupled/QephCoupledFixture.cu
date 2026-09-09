#include "QephCoupledFixture.h"

namespace qeph_coupled_test {
bool InitializeCoupled(Rig& r,double h) {
  r.h=h; r.mass.fill(0); r.inertia.fill(0); r.physical.fill(0); r.added.fill(0);
  for(unsigned e=0;e<r.count;++e) {
    auto input=r.element[e].reference.input;
    input.young_modulus=Young; input.density=Density; input.thickness=Thickness; input.poisson_ratio=Poisson;
    for(unsigned i=0;i<4;++i) {
      auto& x=input.position[i]; x={Side*x.x,Side*x.y,Side*x.z};
      const auto n=r.element[e].nodes[i]; r.x[3*n]=x.x; r.x[3*n+1]=x.y; r.x[3*n+2]=x.z;
    }
    const auto code=q::InitializeReference(input,r.element[e].reference);
    EXPECT_EQ(code,q::Status::kSuccess); if(code!=q::Status::kSuccess) return false;
    const auto& ref=r.element[e].reference;
    for(unsigned i=0;i<4;++i) { const auto n=r.element[e].nodes[i];
      r.mass[n]+=ref.nodal_mass[i]; r.inertia[n]+=ref.isotropic_inertia[i];
      r.physical[n]+=ref.physical_inertia[i]; r.added[n]+=ref.added_inertia[i];
    }
  }
  for(unsigned n=0;n<r.n;++n) { r.inverse[n]=1/r.mass[n]; r.inverse_j[n]=1/r.inertia[n]; }
  if(!r.InitializeOwner()) return false;
  auto config=r.Config(q::BatchUsage::CoupledForces);
  config.qualification_id=CoupledQualification; config.configuration_id=0x4251334d4f444531ULL;
  const auto report=r.batch.Initialize(config,r.element.data());
  EXPECT_EQ(report.status,q::BatchStatus::Success)<<report.message;
  return report.status==q::BatchStatus::Success&&r.Bind();
}
Loads Amplitude(const Rig& r) {
  Loads load;
  for(unsigned n=0;n<r.n;++n) {
    const unsigned station=n/2;
    if(r.count==1) load.couple[3*n+1]=(station?1.:-1.)*.5*BendingScale()*Theta;
    else load.force[3*n+2]=(station==1?-1.:.5)*BendingScale()*Delta/(Side*Side);
  }
  return load;
}
Loads Applied(const Rig& r,double time) {
  auto load=Amplitude(r); const double factor=time<2*H0?1:(time<3*H0?0:-1);
  for(unsigned i=0;i<3*r.n;++i) { load.force[i]*=factor; load.couple[i]*=factor; } return load;
}
bool PrepareCoupled(Rig& r,const Loads& load,Prepared& p) {
  fe::NodalAssemblyView v; auto code=r.owner.BeginTrial(&p.token,&v);
  EXPECT_EQ(code.status,fe::NodalStatus::Ok); if(code.status!=fe::NodalStatus::Ok) return false;
  const auto assembled=r.batch.AssembleAccepted(v);
  EXPECT_EQ(assembled.status,q::BatchStatus::Success)<<assembled.message;
  if(assembled.status!=q::BatchStatus::Success) return false;
  AddLoads<<<1,1,0,v.stream>>>(v,load,1.);
  auto error=cudaStreamSynchronize(v.stream); EXPECT_EQ(error,cudaSuccess); if(error!=cudaSuccess) return false;
  const double* force[]{v.forces.force_x,v.forces.force_y,v.forces.force_z};
  const double* couple[]{v.forces.couple_x,v.forces.couple_y,v.forces.couple_z};
  std::array<double,N> values{};
  for(unsigned a=0;a<3;++a) {
    error=cudaMemcpy(values.data(),force[a],r.n*sizeof(double),cudaMemcpyDeviceToHost);
    EXPECT_EQ(error,cudaSuccess); if(error!=cudaSuccess) return false;
    for(unsigned n=0;n<r.n;++n) p.assembled.force[3*n+a]=values[n];
    error=cudaMemcpy(values.data(),couple[a],r.n*sizeof(double),cudaMemcpyDeviceToHost);
    EXPECT_EQ(error,cudaSuccess); if(error!=cudaSuccess) return false;
    for(unsigned n=0;n<r.n;++n) p.assembled.couple[3*n+a]=values[n];
  }
  code=r.owner.SealAssembly(p.token); EXPECT_EQ(code.status,fe::NodalStatus::Ok);
  if(code.status!=fe::NodalStatus::Ok) return false;
  code=fe::AdvanceStaggeredHistory(r.owner,p.token,{v.owner_id,v.accepted.base_epoch,v.attempt,r.h,1e-3,CoupledQualification});
  EXPECT_EQ(code.status,fe::NodalStatus::Ok)<<code.message; if(code.status!=fe::NodalStatus::Ok) return false;
  code=r.owner.BorrowPrepared(p.token,&p.view); EXPECT_EQ(code.status,fe::NodalStatus::Ok);
  if(code.status!=fe::NodalStatus::Ok) return false;
  const auto& k=p.view.kinematics;
  const double* source[]{k.position_xyz,k.velocity_xyz,k.angular_velocity_xyz,k.orientation_wxyz};
  double* destination[]{p.state.x.data(),p.state.v.data(),p.state.omega.data(),p.state.q.data()};
  for(unsigned i=0;i<4;++i) {
    error=cudaMemcpy(destination[i],source[i],(i==3?4:3)*r.n*sizeof(double),cudaMemcpyDeviceToHost);
    EXPECT_EQ(error,cudaSuccess); if(error!=cudaSuccess) return false;
  }
  return true;
}
q::PrescribedInterval Interval(const Rig& r,unsigned e,const Snapshot& s,double time,std::uint64_t epoch) {
  q::PrescribedInterval p; p.base_time=time; p.dt=r.h; p.sample_index=epoch+1;
  for(unsigned i=0;i<4;++i) { const auto n=r.element[e].nodes[i];
    p.position_endpoint[i]={s.x[3*n],s.x[3*n+1],s.x[3*n+2]};
    p.velocity_midpoint[i]={s.v[3*n],s.v[3*n+1],s.v[3*n+2]};
    p.omega_midpoint[i]={s.omega[3*n],s.omega[3*n+1],s.omega[3*n+2]};
  } return p;
}
void OwnerAgreement(const Rig& r,const Snapshot& actual,const Snapshot& expected) {
  for(unsigned i=0;i<3*r.n;++i) {
    SCOPED_TRACE(i);
    EXPECT_NEAR(actual.x[i],expected.x[i],2e-12*Side);
    EXPECT_NEAR(actual.v[i],expected.v[i],VelocityBudget());
    EXPECT_NEAR(actual.omega[i],expected.omega[i],SpinBudget());
  }
  for(unsigned i=0;i<4*r.n;++i) EXPECT_NEAR(actual.q[i],expected.q[i],2e-12);
}
} // namespace qeph_coupled_test
