#include "Internal.h"
namespace crash::cases::vehicle_wall::native::owner_source_detail {
void Check(const EnvelopeExecutionSource& execution,const vehicle_startup::TiedSearchPostKinChk& post,
    const modelio::type45::VehicleType45Source& joints,EnvelopeOwnerLimits limits) {
    const auto& source=execution.mechanical();
    const auto& origin=source.wall().vehicle_origin();
    const auto& canonical=source.vehicle_references().source().canonical().data();
    const auto& declaration=post.classification().context().auxiliary().declaration();
    const EnvelopeOwnerLimits hard;
    Require(limits.host_bytes&&limits.host_bytes<=hard.host_bytes&&
        limits.joints.max_joints&&limits.joints.max_joints<=hard.joints.max_joints&&
        limits.joints.max_nodes&&limits.joints.max_nodes<=hard.joints.max_nodes&&
        limits.joints.max_host_bytes&&limits.joints.max_host_bytes<=hard.joints.max_host_bytes&&
        limits.witnesses.host_bytes&&limits.witnesses.host_bytes<=hard.witnesses.host_bytes&&
        limits.witnesses.witnesses&&limits.witnesses.witnesses<=hard.witnesses.witnesses,
        "Invalid bounded combined owner-source limits");
    Require(&declaration.canonical().data()==&canonical&&
        &joints.source_domain().source().tied_source().canonical().data()==&canonical&&
        joints.source_domain().domain().SharesStorage(source.embedding().original())&&
        joints.source_domain().policy()==origin.policy()&&
        joints.policy()==modelio::type45::Policy::OriginalDirectSdiType45VehicleSupportsV5,
        "Combined execution/CIN/joint inputs do not share exact original source authority");
    Require(post.phase()==vehicle_startup::TiedPostKinChkPhase::ObservedKinetAfterKinChk&&
        post.result().source_instance_id()==source.domain().source_instance_id()&&
        post.receipt().native_profile==vehicle_startup::native_search::KinChkProfile::NoWallRbeOrCyclic&&
        post.receipt().effective_primitive_walls==0&&post.receipt().rbe2_roles==0&&
        post.receipt().rbe3_roles==0&&post.receipt().cyclic_roles==0&&post.result().slaves().count==11165,
        "Combined source requires the authentic complete retained post-KINCHK read set");
    Require(joints.data().rows.size()==44&&joints.data().required==44&&joints.data().boundaries==0&&
        limits.joints.max_joints>=44&&limits.joints.max_nodes>=source.domain().node_count(),
        "Complete retained TYPE45 source census or native capacity differs");
    for(const auto& row:joints.data().rows) {
        Require(row.disposition==modelio::type45::Disposition::Required,
            "Combined source cannot drop an original retained joint");
        for(unsigned endpoint=0;endpoint<2;++endpoint) {
            const auto& node=row.nodes[endpoint];
            Require(node.domain_index<source.embedding().original().node_count()&&
                source.domain().nodes()[node.domain_index].source_id==node.source_id&&
                source.rigid_assembly().FindMember(node.domain_index),
                "Original joint endpoint is not an actual combined rigid member");
        }
    }
    const auto* physical_execution=execution.physical().execution();
    const auto* actual_rigid=physical_execution?physical_execution->rigid():nullptr;
    Require(execution.physical().domain()->SharesStorage(source.domain())&&
        execution.physical().coefficients()->Matches(source.coefficients())&&actual_rigid&&
        actual_rigid->domain()->SharesStorage(source.domain())&&
        actual_rigid->groups().data()==source.rigid_assembly().groups().data()&&
        actual_rigid->members().data()==source.rigid_assembly().members().data(),
        "Combined physical execution lost its actual coefficient/rigid graph");
    // The new fixed boundary is an explicit disjoint supplement. It cannot
    // modify the original observed slave read set, and no new TYPE2 or
    // primitive-wall/RBE/cyclic declaration is introduced by this profile.
    const auto slaves=post.result().slaves();
    for(std::size_t i=0;i<slaves.count;++i)
        for(const auto id:source.wall().ids().nodes)
            Require(slaves.data[i].before.source_id!=id,
                "Declared wall enters the original post-KINCHK read set");
}
}
