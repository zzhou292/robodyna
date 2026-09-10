#pragma once
#include "GroupOwnerFixture.h"
#include <cstring>

namespace prepared_snapshot_test {
using namespace rigid_owner_test;
struct Output {
  nt::Snapshot nodes;
  fe::NodalPreparedView prepared;
  Output() {
    nodes.x.fill(11);nodes.v.fill(12);nodes.omega.fill(13);nodes.q.fill(14);
    nodes.reaction.fill(15);nodes.couple.fill(16);
    prepared.owner_id=71;prepared.attempt=72;prepared.proposed_time=-1;
  }
};
inline void SameFields(const nt::Snapshot& a,const nt::Snapshot& b,std::size_t n) {
  const double* x[]{a.x.data(),a.v.data(),a.omega.data(),a.q.data(),a.reaction.data(),a.couple.data()};
  const double* y[]{b.x.data(),b.v.data(),b.omega.data(),b.q.data(),b.reaction.data(),b.couple.data()};
  for(unsigned field=0;field<6;++field)
    EXPECT_EQ(std::memcmp(x[field],y[field],(field==3?4:3)*n*sizeof(double)),0)<<field;
}
inline void SameOutput(const Output& a,const Output& b) {
  nt::SameState(a.nodes,b.nodes);
  EXPECT_TRUE(fe::trial_identity::SamePrepared(a.prepared,b.prepared));
}
using TokenImage=std::array<unsigned char,sizeof(fe::NodalTrialToken)>;
inline TokenImage Image(const fe::NodalTrialToken& token) {
  TokenImage bytes;std::memcpy(bytes.data(),&token,bytes.size());return bytes;
}
inline bool Copy(fe::FENodalState& owner,const fe::NodalTrialToken& token,Output& out) {
  const auto r=owner.CopyPrepared(token,out.nodes.buffer(),&out.prepared);
  EXPECT_EQ(r.status,Code::Ok)<<r.message;return r.status==Code::Ok;
}
inline double*& Field(fe::NodalSnapshotBuffer& out,unsigned index) {
  switch(index) {
    case 0:return out.position_xyz;case 1:return out.velocity_xyz;
    case 2:return out.orientation_wxyz;case 3:return out.angular_velocity_xyz;
    case 4:return out.reaction_force_xyz;default:return out.reaction_couple_xyz;
  }
}
} // namespace prepared_snapshot_test
