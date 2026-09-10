#include "NativeRotation.h"
#include "lib_src/elements/qeph/QephKinematics.h"
#include "lib_src/elements/t3/T3Kinematics.h"
#include "lib_src/constraints/NodalRigidGroupMath.h"
#include <cmath>
namespace crash::cases::source_assembly_dynamics {
namespace q=tl::fea::qeph;namespace t=tl::fea::t3;namespace value=tl::fea::rigid::detail;
NativeRotationReferences::NativeRotationReferences(std::size_t p,std::size_t n,std::uint64_t instance)
    :parent_count_(p),node_count_(n),source_instance_(instance),parents_(new NativeRotationReference[p]),grouped_(new std::uint8_t[n]{}){}
Report PrepareNativeRotation(const source_assembly::SourceAssemblyBindings& b,double dt,std::unique_ptr<const NativeRotationReferences>& output) {
    const auto& shells=b.shells();const auto nq=shells.qeph_count(),nt=shells.t3_count(),nodes=shells.node_count();
    if(!shells.prepared()||!nodes||nodes>2048||!nq||!nt||nq+nt>1024||!std::isfinite(dt)||dt<=0||!b.rigid_groups()||!b.rigid_groups()->prepared()||
       b.rigid_groups()->source_instance_id()!=b.source_instance_id()||b.rigid_groups()->global_node_count()!=nodes)
        return {Status::SourceMismatch,"Incomplete source-native rotation startup"};
    auto next=std::unique_ptr<NativeRotationReferences>(new NativeRotationReferences(nq+nt,nodes,b.source_instance_id()));
    const auto& groups=*b.rigid_groups();
    for(std::size_t m=0;m<groups.member_count();++m) {
        const auto& member=groups.members()[m];const auto n=member.global_node;
        if(n>=nodes||next->grouped_[n]||shells.nodes()[n].source_id!=member.source_node_id)
            return {Status::SourceMismatch,"Native rotation rigid member identity differs",0,n};
        next->grouped_[n]=1;
    }
    auto store=[&](std::size_t index,std::uint64_t id,const tl::math::Matrix3& frame,const tl::math::Vec3* normals,std::size_t arity) {
        auto& r=next->parents_[index];r.source_parent=id;r.arity=arity;r.frame=frame;
        for(std::size_t n=0;n<arity;++n)r.world_normals[n]=value::ToWorld(frame,normals[n]);
        NativeRotationMeasure measure;return MeasureNativeRotation(r,frame,normals,arity,NativeRotationQualifiedBound,measure);
    };
    // Reference geometry is evaluated by the same qualified native geometry
    // operation as candidate forces. Zero rates are observation setup only;
    // these values never replace actual nodal velocities/spins or force packets.
    for(std::size_t e=0;e<nq;++e) {
        const auto& ref=shells.qeph_reference(e);q::PrescribedInterval in;in.dt=dt;in.sample_index=1;
        for(unsigned n=0;n<4;++n)in.position_endpoint[n]=ref.input.position[n];q::Kinematics k;
        if(q::EvaluatePrescribed(ref,in,k)!=q::Status::kSuccess)
            return {Status::SourceMismatch,"Original source QEPH rotation geometry failed",shells.qeph_source_id(e)};
        const auto report=store(e,shells.qeph_source_id(e),k.frame,k.local_normals,4);if(!report)return report;
    }
    for(std::size_t e=0;e<nt;++e) {
        const auto& ref=shells.t3_reference(e);t::PrescribedInterval in;in.dt=dt;in.sample_index=1;
        for(unsigned n=0;n<3;++n)in.position[n]=ref.input.position[n];t::Kinematics k;
        if(t::EvaluatePrescribed(ref,in,k)!=t::Status::kSuccess)
            return {Status::SourceMismatch,"Original source T3 rotation geometry failed",shells.t3_source_id(e)};
        const auto report=store(nq+e,shells.t3_source_id(e),k.frame,NativeTriangleNormals,3);if(!report)return report;
    }
    output=std::move(next);return {Status::Ok,"Source-native rotation reference prepared"};
}
}
