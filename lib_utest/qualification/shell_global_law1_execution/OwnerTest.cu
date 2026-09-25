#include "OwnerFixture.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "../shell_global_law1/Fixture.h"
#include "lib_src/elements/publication/PhysicalState.h"
#include "lib_src/elements/qeph/mapped/Stiffness.h"
#include "lib_src/elements/t3/mapped/Stiffness.h"
namespace global_law1_execution_test {
namespace nq=tl::qualification::qeph;namespace nt=tl::qualification::t3;
namespace native=tl::qualification::global_law1_native;
using global_law1_test::ForceBits;
void SameSection(const fe::ShellBatchLayeredSection& a,const fe::ShellBatchLayeredSection& b) {
  EXPECT_EQ(a.law(),b.law());
  if(a.plastic()) {
    ASSERT_NE(b.plastic(),nullptr);const auto& x=*a.plastic();const auto& y=*b.plastic();
    for(unsigned p=0;p<3;++p) {
      for(unsigned c=0;c<5;++c)global_law1_test::DoubleBits(x.history.point[p].stress[c],y.history.point[p].stress[c]);
      global_law1_test::DoubleBits(x.history.point[p].plastic_strain,y.history.point[p].plastic_strain);
      global_law1_test::DoubleBits(x.history.point[p].filtered_rate_per_s,y.history.point[p].filtered_rate_per_s);
    }
    const auto& u=x.diagnostics;const auto& v=y.diagnostics;
    const double first[]{u.plastic_work_density_increment,u.maximum_plastic_strain,u.mean_plastic_strain,u.minimum_tangent_ratio,u.mean_tangent_ratio,u.mean_yield_before_pa,u.last_point_yield_before_pa,x.cumulative_plastic_work_J};
    const double second[]{v.plastic_work_density_increment,v.maximum_plastic_strain,v.mean_plastic_strain,v.minimum_tangent_ratio,v.mean_tangent_ratio,v.mean_yield_before_pa,v.last_point_yield_before_pa,y.cumulative_plastic_work_J};
    for(unsigned i=0;i<8;++i)global_law1_test::DoubleBits(first[i],second[i]);
  }
}
void SameAccepted(const Snapshot& a,const Snapshot& b) {
  EXPECT_TRUE(fe::trial_identity::SameStamp(a.stamp,b.stamp));
  EXPECT_EQ(a.x,b.x);EXPECT_EQ(a.v,b.v);EXPECT_EQ(a.w,b.w);EXPECT_EQ(a.orientation,b.orientation);
  EXPECT_EQ(a.reaction,b.reaction);EXPECT_EQ(a.couple,b.couple);EXPECT_EQ(a.m,b.m);EXPECT_EQ(a.j,b.j);
  ASSERT_EQ(a.quads.size(),b.quads.size());ASSERT_EQ(a.triangles.size(),b.triangles.size());
  for(unsigned i=0;i<a.quads.size();++i) {
    ForceBits(a.quads[i],b.quads[i]);SameSection(a.qsections[i],b.qsections[i]);
    EXPECT_EQ(a.qfailure[i].policy(),b.qfailure[i].policy());EXPECT_EQ(a.qfailure[i].active,b.qfailure[i].active);
    if(a.qsections[i].elastic()) {
      ASSERT_NE(b.qsections[i].elastic(),nullptr);
      for(unsigned p=0;p<3;++p)for(unsigned j=0;j<5;++j)
        global_law1_test::DoubleBits(a.qsections[i].elastic()->point[p].stress[j],b.qsections[i].elastic()->point[p].stress[j]);
    }
  }
  for(unsigned i=0;i<a.triangles.size();++i) {
    ForceBits(a.triangles[i],b.triangles[i]);SameSection(a.tsections[i],b.tsections[i]);
    EXPECT_EQ(a.tfailure[i].policy(),b.tfailure[i].policy());EXPECT_EQ(a.tfailure[i].active,b.tfailure[i].active);
    if(a.tsections[i].elastic()) {
      ASSERT_NE(b.tsections[i].elastic(),nullptr);
      for(unsigned p=0;p<3;++p)for(unsigned j=0;j<5;++j)
        global_law1_test::DoubleBits(a.tsections[i].elastic()->point[p].stress[j],b.tsections[i].elastic()->point[p].stress[j]);
    }
  }
  EXPECT_TRUE(fe::shell_publication_detail::SamePhysicalDiagnostics(a.common,b.common));
}
template<std::size_t N> tl::math::Vec3 Node(const std::vector<double>& values,const OwnerFixture& f,
    const std::array<std::size_t,N>& source,unsigned slot) {
  const auto node=f.physical.mapping.owner_index(source[slot]);
  return {values[3*node],values[3*node+1],values[3*node+2]};
}
q::PrescribedInterval QuadInterval(const OwnerFixture& f,const Snapshot& before,const Snapshot& after) {
  q::PrescribedInterval in;in.base_time=before.stamp.time;in.dt=before.stamp.fixed_dt;in.sample_index=after.stamp.epoch;
  const auto& nodes=f.physical.shells.qeph_nodes(0);
  for(unsigned i=0;i<4;++i){in.position_endpoint[i]=Node(after.x,f,nodes,i);in.velocity_midpoint[i]=Node(after.v,f,nodes,i);in.omega_midpoint[i]=Node(after.w,f,nodes,i);}return in;
}
t::PrescribedInterval TriangleInterval(const OwnerFixture& f,const Snapshot& before,const Snapshot& after) {
  t::PrescribedInterval in;in.base_time=before.stamp.time;in.dt=before.stamp.fixed_dt;in.sample_index=after.stamp.epoch;
  const auto& nodes=f.physical.shells.t3_nodes(0);
  for(unsigned i=0;i<3;++i){in.position[i]=Node(after.x,f,nodes,i);in.velocity[i]=Node(after.v,f,nodes,i);in.angular_velocity[i]=Node(after.w,f,nodes,i);}return in;
}
void PointAvailability(const Snapshot& value) {
  for(const auto* rows:{&value.qsections,&value.tsections}) {
    ASSERT_EQ(rows->size(),3u);
    EXPECT_EQ((*rows)[0].law(),fe::ShellSectionLaw::GlobalLaw1Npt0);
    EXPECT_EQ((*rows)[0].elastic(),nullptr);EXPECT_EQ((*rows)[0].plastic(),nullptr);EXPECT_EQ((*rows)[0].one_point(),nullptr);
    EXPECT_EQ((*rows)[1].law(),fe::ShellSectionLaw::LayeredLaw1Nip3);ASSERT_NE((*rows)[1].elastic(),nullptr);
    EXPECT_EQ((*rows)[2].law(),fe::ShellSectionLaw::LayeredLaw44Nip3);ASSERT_NE((*rows)[2].plastic(),nullptr);
  }
}
TEST(GlobalLaw1ExecutionCuda,MixedSourceOwnerPublishesNativeGlobalHistoryForcesAndStiffness) {
  for(auto mode:{Thickness::Reference,Thickness::Accepted}) {
    const Profile profile{mode,.001};OwnerFixture f(profile);ASSERT_NO_THROW(f.Initialize());
    auto before=f.Read();PointAvailability(before);
    EXPECT_EQ(f.physical.execution.counts().constitutive,6u);EXPECT_EQ(f.physical.execution.counts().rigid_skin,0u);
    EXPECT_EQ(f.physical.execution.counts().material_points,12u);
    nq::Reference qr;nt::Reference tr;
    const auto& qref=f.physical.shells.qeph_reference(0);const auto& tref=f.physical.shells.t3_reference(0);
    ASSERT_EQ(nq::Initialize(qeph_startup_test::NativeInput(qref.input),qr),nq::Status::kSuccess);
    ASSERT_EQ(nt::Initialize(t3_port_test::Native(tref.input),tr),nt::Status::kSuccess);
    nq::History qh;nt::History th;
    ASSERT_EQ(nq::InitializeHistory(qr,{},qh),nq::Status::kSuccess);
    ASSERT_EQ(nt::InitializeHistory(tr,{},th),nt::Status::kSuccess);
    const auto allocations=f.owner.allocations();
    for(unsigned step=0;step<16;++step) {
      SCOPED_TRACE(step);
      Attempt a;ASSERT_NO_THROW(f.Begin(a));
      ASSERT_NO_THROW(f.Prepare(a));
      ASSERT_NO_THROW(Check(f.Commit(a)));
      auto after=f.Read();PointAvailability(after);
      EXPECT_EQ(after.stamp.epoch,before.stamp.epoch+1);EXPECT_EQ(after.m,before.m);EXPECT_EQ(after.j,before.j);
      const auto qi=QuadInterval(f,before,after);const auto ti=TriangleInterval(f,before,after);
      nq::ForceTrial qnative;nt::ForceTrial tnative;
      ASSERT_EQ(native::Evaluate(qr,qh,qeph_kinematics_test::NativeInterval(qi),mode==Thickness::Accepted,qnative),nq::Status::kSuccess);
      ASSERT_EQ(native::Evaluate(tr,th,t3_port_test::Native(ti),mode==Thickness::Accepted,tnative),nt::Status::kSuccess);
      qeph_force_port_test::ForceAgreement(after.quads[0],qnative,qref.input,qi,2e-11);
      t3_force_port_test::Agreement(tref,ti,after.triangles[0],tnative,2e-11);
      EXPECT_EQ(after.quads[0].diagnostics.effective_thickness,mode==Thickness::Accepted?before.quads[0].proposed_history.data().thickness:qref.input.thickness);
      EXPECT_EQ(after.triangles[0].diagnostics.effective_thickness,mode==Thickness::Accepted?before.triangles[0].proposed_history.data().thickness:tref.input.thickness);
      qh=qnative.proposed_history;th=tnative.proposed_history;before=std::move(after);
      ASSERT_FALSE(::testing::Test::HasFailure());
    }
    EXPECT_NE(before.quads[0].proposed_history.data().thickness,qref.input.thickness);
    EXPECT_NE(before.triangles[0].proposed_history.data().thickness,tref.input.thickness);
    EXPECT_EQ(f.owner.allocations().device_bytes,allocations.device_bytes);
    EXPECT_EQ(f.owner.allocations().device_allocations,allocations.device_allocations);
  }
}
TEST(GlobalLaw1ExecutionCuda,RejectedCommonCommitAndDiscardLeaveAllAcceptedParticipantsUnchanged) {
  OwnerFixture f({Thickness::Accepted,1.});ASSERT_NO_THROW(f.Initialize());
  Attempt first;ASSERT_NO_THROW(f.Begin(first));
  ASSERT_NO_THROW(f.Prepare(first));
  ASSERT_NO_THROW(Check(f.Commit(first)));
  const auto original=f.Read();Attempt rejected;ASSERT_NO_THROW(f.Begin(rejected));
  ASSERT_NO_THROW(f.Prepare(rejected));
  EXPECT_NE(f.Commit(rejected,false).status,fe::ShellPublicationStatus::Success);
  SameAccepted(original,f.Read());f.Discard();SameAccepted(original,f.Read());
  Attempt retry;ASSERT_NO_THROW(f.Begin(retry));
  ASSERT_NO_THROW(f.Prepare(retry));
  ASSERT_NO_THROW(Check(f.Commit(retry)));
  const auto after=f.Read();EXPECT_EQ(after.stamp.epoch,original.stamp.epoch+1);PointAvailability(after);
  Attempt abandoned;ASSERT_NO_THROW(f.Begin(abandoned));
  ASSERT_NO_THROW(f.Prepare(abandoned));
  f.Discard();SameAccepted(after,f.Read());
}
TEST(GlobalLaw1ExecutionCuda,ActualVirginAssemblyUsesDimensionedGlobalFloorAndRetainsRawMass) {
  for(double length:{1.,.001}) {
    Profile profile{Thickness::Accepted,length};OwnerFixture f(profile);
    const double thickness=.5*fe::shell_global_law1::NativeThicknessFloor*length;
    for(auto& q:f.source.quads)q.reference.thickness=thickness;
    for(auto& t:f.source.triangles)t.reference.thickness=thickness;
    f.source.sections[0].thickness_m=thickness;
    ASSERT_NO_THROW(f.Initialize());const auto before=f.Read();Attempt a;
    ASSERT_NO_THROW(f.Begin(a));fe::NodalCinAssemblyView cin;
    ASSERT_EQ(f.owner.BorrowCinAssembly(a.token,&cin).status,fe::NodalStatus::Ok);
    std::vector<double> translation(f.source.nodes),rotation(f.source.nodes);
    struct Drain {cudaStream_t stream;~Drain(){cudaStreamSynchronize(stream);}} drain{cin.stream};
    ASSERT_EQ(cudaMemcpyAsync(translation.data(),cin.translational_stiffness,translation.size()*sizeof(double),cudaMemcpyDeviceToHost,cin.stream),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(rotation.data(),cin.rotational_stiffness,rotation.size()*sizeof(double),cudaMemcpyDeviceToHost,cin.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(cin.stream),cudaSuccess);
    q::mapped::NodalStiffness qp;t::mapped::NodalStiffness tp;
    ASSERT_TRUE(q::mapped::InitialStiffness(f.physical.shells.qeph_reference(0),fe::ShellSectionLaw::GlobalLaw1Npt0,qp,&profile));
    ASSERT_TRUE(t::mapped::InitialStiffness(f.physical.shells.t3_reference(0),fe::ShellSectionLaw::GlobalLaw1Npt0,tp,&profile));
    for(unsigned slot=0;slot<4;++slot) {
      const auto node=f.physical.mapping.owner_index(f.physical.shells.qeph_nodes(0)[slot]);
      global_law1_test::DoubleBits(translation[node],qp.translation[slot]);global_law1_test::DoubleBits(rotation[node],qp.rotation[slot]);
    }
    for(unsigned slot=0;slot<3;++slot) {
      const auto node=f.physical.mapping.owner_index(f.physical.shells.t3_nodes(0)[slot]);
      global_law1_test::DoubleBits(translation[node],tp.translation[slot]);global_law1_test::DoubleBits(rotation[node],tp.rotation[slot]);
    }
    f.Discard();const auto after=f.Read();SameAccepted(before,after);
    EXPECT_EQ(after.m,f.physical.mass);EXPECT_EQ(after.j,f.physical.inertia);
  }
}
} // namespace global_law1_execution_test
