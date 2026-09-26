#pragma once
#include "case/vehicle_dynamics/native_contact/Group.h"
#include "lib_utest/qualification/radioss_type25_multi_interface/Fixture.h"
namespace native_app_group_test {
namespace app=crash::cases::vehicle_dynamics::native_contact;
namespace base=native_group_test;
using base::Check;
struct Rig {
    base::Rig backend;
    std::unique_ptr<app::Group> group;
    app::GroupObservation observed;
    void Initialize(bool bind=true) {
        backend.Initialize(false);
        std::array<app::GroupInput,2> input;
        input[0]={app::Role::Self,std::move(backend.native[0])};
        input[1]={app::Role::MeshWall,std::move(backend.native[1])};
        group=app::Group::Adopt(std::move(input),2);
        if(bind)Bind();
    }
    void Bind() {
        group->Bind(backend.physical.owner,*backend.physical.publication,backend.physical.fixture.physical,
            backend.physical.Participants(),backend.physical.Identity());
    }
    void Assemble(base::Attempt& a) {
        backend.Begin(a);group->Assemble(backend.physical.owner,a.token,a.assembly,observed);
    }
    void Prepare(base::Attempt& a) {
        backend.PrepareMaterials(a);
        group->SealCandidate(backend.physical.owner,a.token,a.prepared,a.common,observed);
        Check(backend.physical.publication->SealPhysicalScratchParticipation(
            backend.physical.owner,a.token,group->scratch_receipts()));
    }
    void Step() {
        base::Attempt a;Assemble(a);Prepare(a);Check(backend.physical.Commit(a));group->Committed();
    }
    void Discard() {group->Discard();backend.Discard();}
    base::Snapshot Read() {
        base::Snapshot result;auto& value=result.physical;auto& owner=backend.physical.owner;
        Check(owner.CopyAccepted({value.x.data(),value.v.data(),18,value.q.data(),value.omega.data(),
            value.reaction.data(),value.couple.data()},&value.stamp));
        double numerical=0;base::fe::NodalStamp stamp;
        Check(owner.CopyAcceptedCin({value.mass.data(),value.inertia.data(),nullptr,nullptr,&numerical,18,0},&stamp));
        for(std::size_t i=0;i<2;++i)Check(group->transaction(i).CopyAccepted(
            {result.history[i].data(),result.flags[i].data(),18},&result.contact[i]));
        return result;
    }
};
inline void NumericalParity(const base::Snapshot& a,const base::Snapshot& b) {
    base::Bits(a.physical.x,b.physical.x);base::Bits(a.physical.v,b.physical.v);
    base::Bits(a.physical.q,b.physical.q);base::Bits(a.physical.omega,b.physical.omega);
    base::Bits(a.physical.mass,b.physical.mass);base::Bits(a.physical.inertia,b.physical.inertia);
    EXPECT_EQ(a.physical.stamp.epoch,b.physical.stamp.epoch);EXPECT_EQ(a.physical.stamp.time,b.physical.stamp.time);
    for(unsigned i=0;i<2;++i) {
        EXPECT_EQ(a.contact[i].generation,b.contact[i].generation);EXPECT_EQ(a.flags[i],b.flags[i]);
        for(unsigned row=0;row<18;++row)type25_geometry_test::Same(a.history[i][row],b.history[i][row],true);
    }
}
}
