#include "case/vehicle_dynamics/native_contact/Group.h"
#include "lib_utest/qualification/native_group_activity/Fixture.h"
namespace native_group_activity_adapter_test {
namespace app=crash::cases::vehicle_dynamics::native_contact;
namespace gate=native_group_activity_test;
using gate::Check;using gate::Attempt;
struct Rig {
  gate::Rig base;
  std::unique_ptr<app::Group> group;
  app::GroupObservation observation;
  void Initialize(){base.Initialize(2,false);std::array<app::GroupInput,2> input;
    input[0]={app::Role::Self,std::move(base.contacts[0])};input[1]={app::Role::MeshWall,std::move(base.contacts[1])};
    group=app::Group::Adopt(std::move(input),2);auto&p=base.physical;
    group->Bind(p.owner,p.publication,p.fixture.physical,p.Participants(),p.Identity());}
  void Prepare(Attempt&a){auto&p=base.physical;p.Begin(a);group->Assemble(p.owner,a.token,a.assembly,observation);p.Prepare(a);}
  void Seal(Attempt&a){auto&p=base.physical;group->SealCandidate(p.owner,p.publication,a.token,a.prepared,a.common,observation);}
  void Commit(Attempt&a){auto&p=base.physical;Check(p.publication.SealPhysicalScratchParticipation(p.owner,a.token,group->scratch_receipts()));Check(p.Commit(a));group->Committed();}
};
TEST(NativeGroupedActivityAdapterCuda, ActualAllActiveGroupReusesTraversalAndMatchesSequential){
 Rig wrapped;gate::Rig direct;ASSERT_NO_THROW(wrapped.Initialize());ASSERT_NO_THROW(direct.Initialize());
 for(unsigned i=0;i<3;++i){Attempt a,b;ASSERT_NO_THROW(wrapped.Prepare(a));ASSERT_NO_THROW(direct.Prepare(b));gate::Watch();ASSERT_NO_THROW(wrapped.Seal(a));const auto once=gate::Stop();
  gate::Watch();ASSERT_NO_THROW(direct.Sequential(b));const auto twice=gate::Stop();EXPECT_GT(once.copies,0u);EXPECT_EQ(twice.copies,2*once.copies);
  ASSERT_NO_THROW(wrapped.Commit(a));ASSERT_NO_THROW(direct.Commit(b));ASSERT_NO_FATAL_FAILURE(gate::Same(wrapped.base.Read(),direct.Read(),2,false));}
}
TEST(NativeGroupedActivityAdapterCuda, FreshCorruptionRejectsWithoutPublishingAppObservation){
 Rig rig;ASSERT_NO_THROW(rig.Initialize());const auto before=rig.base.Read();Attempt a;ASSERT_NO_THROW(rig.Prepare(a));
 tl::fea::t3::ForceTrial saved;const void*pointer=nullptr;ASSERT_NO_THROW(pointer=rig.base.CaptureForce(a,saved));auto invalid=saved;invalid.diagnostics.native_sound_speed=std::nan("31");
 ASSERT_EQ(cudaMemcpy(const_cast<void*>(pointer),&invalid,sizeof(invalid),cudaMemcpyHostToDevice),cudaSuccess);rig.observation.count=77;
 EXPECT_THROW(rig.Seal(a),app::StageError);EXPECT_EQ(rig.observation.count,77u);EXPECT_EQ(rig.group->scratch_receipts().native_interfaces.count,0u);
 ASSERT_EQ(cudaMemcpy(const_cast<void*>(pointer),&saved,sizeof(saved),cudaMemcpyHostToDevice),cudaSuccess);rig.group->Discard();rig.base.physical.Discard();
 ASSERT_NO_FATAL_FAILURE(gate::Same(before,rig.base.Read()));Attempt retry;ASSERT_NO_THROW(rig.Prepare(retry));ASSERT_NO_THROW(rig.Seal(retry));ASSERT_NO_THROW(rig.Commit(retry));
}
}
