#pragma once
#include "../Internal.h"
#include "lib_utest/qualification/radioss_type25_runtime/RuntimeObservation.h"
#include <array>
#include <stdexcept>
namespace crash::cases::native_scene::qualification {
// Owning qualification only. Invokes the exact closed production stages and
// returns copied observations; no setters, callbacks or receipt manufacture.
class NativeSceneAccess {
 public:
    static tl::fea::NodalRigidGroupSnapshot Rigid(NativeSceneDynamics& d) {
        auto& owner=d.storage_->owner;
        if(owner.accepted().rigid_groups.group_count!=1)throw std::runtime_error("Rigid fixture needs one actual group");
        tl::fea::NodalRigidGroupSnapshot result;tl::fea::NodalStamp stamp;
        const auto report=owner.CopyAcceptedRigidGroups({&result,1},&stamp);
        if(report.status!=tl::fea::NodalStatus::Ok||!tl::fea::trial_identity::SameStamp(stamp,owner.accepted()))
            throw std::runtime_error("Rigid accepted snapshot is unavailable or stale");
        return result;
    }
    static void BeginMaterials(NativeSceneDynamics& d){d.storage_->BeginMaterials();}
    static void AssembleContact(NativeSceneDynamics& d){d.storage_->AssembleContact();}
    static void FinishPrepare(NativeSceneDynamics& d){d.storage_->FinishPrepare();}
    static bool Observe(NativeSceneDynamics& d,native::runtime_qualification::Observation& out) {
        auto& s=*d.storage_;return native::runtime_qualification::Access::Read(s.contact,s.owner,s.token,s.assembly,&out);
    }
    static tl::fea::NativeContactPublicationSnapshot Contact(const NativeSceneDynamics& d){return d.storage_->contact.accepted();}
    static bool AcceptedNormals(NativeSceneDynamics& d,native::runtime_qualification::NormalObservation& out) {
        return native::runtime_qualification::Access::ReadAcceptedNormals(d.storage_->contact,&out);
    }
    static void Force(NativeSceneDynamics& d,std::array<double,54>& values,std::array<double,18>& stiffness) {
        auto& s=*d.storage_;
        if(!s.trial||(s.stage!=1&&s.stage!=2)||s.assembly.forces.node_count!=18||
            s.owner.AuthenticateAssemblyView(s.token,s.assembly).status!=tl::fea::NodalStatus::Ok)
            throw std::runtime_error("Native trajectory observation needs genuine accepted assembly");
        std::array<double,54> component{};
        struct Drain{cudaStream_t stream;~Drain(){cudaStreamSynchronize(stream);}} drain{s.assembly.stream};
        const auto check=[](cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));};
        check(cudaMemcpyAsync(component.data(),s.assembly.forces.force_x,18*sizeof(double),cudaMemcpyDeviceToHost,s.assembly.stream));
        check(cudaMemcpyAsync(component.data()+18,s.assembly.forces.force_y,18*sizeof(double),cudaMemcpyDeviceToHost,s.assembly.stream));
        check(cudaMemcpyAsync(component.data()+36,s.assembly.forces.force_z,18*sizeof(double),cudaMemcpyDeviceToHost,s.assembly.stream));
        check(cudaMemcpyAsync(stiffness.data(),s.cin.translational_stiffness,18*sizeof(double),cudaMemcpyDeviceToHost,s.assembly.stream));
        check(cudaStreamSynchronize(s.assembly.stream));
        for(unsigned i=0;i<18;++i)for(unsigned j=0;j<3;++j)values[3*i+j]=component[18*j+i];
    }
};
}
