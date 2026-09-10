#pragma once
#include "NodalWallOwnerFixture.h"
#include <cmath>

namespace nodal_wall_owner_test {
static __global__ void SetEntry(double* array,unsigned index,double value) { array[index]=value; }
static __global__ void Noop() {}
inline std::array<double,6*Capacity> Forces(const fe::NodalAssemblyView& v) {
  std::array<double,6*Capacity> out{};
  const double* components[]{v.forces.force_x,v.forces.force_y,v.forces.force_z,
                             v.forces.couple_x,v.forces.couple_y,v.forces.couple_z};
  for (unsigned c=0;c<6;++c)
    EXPECT_EQ(cudaMemcpyAsync(out.data()+c*Capacity,components[c],v.forces.node_count*sizeof(double),
                              cudaMemcpyDeviceToHost,v.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(v.stream),cudaSuccess); return out;
}
inline bool Prepare(fe::FENodalState& owner,const fe::NodalTrialToken& token,const fe::NodalAssemblyView& v,
             fe::NodalPreparedView& prepared) {
  auto r=owner.SealAssembly(token); EXPECT_EQ(r.status,fe::NodalStatus::Ok); if (r.status!=fe::NodalStatus::Ok) return false;
  r=fe::AdvanceStaggeredHistory(owner,token,{v.owner_id,v.accepted.base_epoch,v.attempt,
                                           owner.accepted().fixed_dt,.1,UnitQualification});
  EXPECT_EQ(r.status,fe::NodalStatus::Ok)<<r.message; if (r.status!=fe::NodalStatus::Ok) return false;
  r=owner.BorrowPrepared(token,&prepared); EXPECT_EQ(r.status,fe::NodalStatus::Ok); return r.status==fe::NodalStatus::Ok;
}
inline bool Commit(fe::FENodalState& owner,const fe::NodalTrialToken& token,const fe::NodalPreparedView& v) {
  const auto r=fe::CompleteNodalValidation(owner,token,{v.owner_id,v.kinematics.base_epoch,v.attempt,UnitQualification,true});
  EXPECT_EQ(r.status,fe::NodalStatus::Ok); if (r.status!=fe::NodalStatus::Ok) return false;
  const auto c=owner.Commit(token); EXPECT_EQ(c.status,fe::NodalStatus::Ok); return c.status==fe::NodalStatus::Ok;
}
inline Snapshot PreparedSnapshot(const fe::NodalPreparedView& v) {
  Snapshot s; const auto n=v.kinematics.node_count;
  EXPECT_EQ(cudaMemcpyAsync(s.x.data(),v.kinematics.position_xyz,3*n*sizeof(double),cudaMemcpyDeviceToHost,v.stream),cudaSuccess);
  EXPECT_EQ(cudaMemcpyAsync(s.v.data(),v.kinematics.velocity_xyz,3*n*sizeof(double),cudaMemcpyDeviceToHost,v.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(v.stream),cudaSuccess); return s;
}
inline void Ledger(double actual,long double expected,long double terms,long double scale) {
  const long double budget=256*std::numeric_limits<double>::epsilon()*terms+1e-12L*scale;
  EXPECT_LE(std::abs(static_cast<long double>(actual)-expected),budget);
}

} // namespace nodal_wall_owner_test
