#include "SourcePartCommonFields.h"
#include "SourcePartFieldPrimitives.h"
namespace crash::cases::source_part_elastic {
using namespace output;
using field_detail::UInt;
using field_detail::Scalar;
void CheckSourcePartPhase(const Snapshot& f,const std::array<double,3>& startup_velocity) {
    const auto& s=f.stamp;
    Require(s.owner_id&&s.node_count==NodeCount&&s.has_rotations&&std::isfinite(s.fixed_dt)&&s.fixed_dt>0&&
        std::isfinite(s.time)&&s.time>=0&&s.temporal_scheme==tl::fea::NodalTemporalScheme::StaggeredHalfKickStart,
        "Invalid source-part field owner or scheme");
    if(!s.epoch) {
        Require(s.time==0&&s.velocity_phase==tl::fea::NodalVelocityPhase::Collocated&&s.velocity_time==0&&
            !s.reactions_valid&&s.reaction_base_epoch==0&&s.reaction_time==0&&s.reaction_kick_dt==0,
            "Source initial field timing is not collocated startup");
        for(std::size_t n=0;n<NodeCount;++n) {
            for(unsigned j=0;j<4;++j)Require(f.orientation[4*n+j]==(j==0?1:0),"Source initial orientation is not identity");
            for(unsigned j=0;j<3;++j)Require(f.velocity[3*n+j]==startup_velocity[j]&&f.omega[3*n+j]==0&&
                f.synchronized_velocity[3*n+j]==startup_velocity[j]&&f.synchronized_omega[3*n+j]==0,"Source initial velocities are not rest");
        }
    } else Require(s.velocity_phase==tl::fea::NodalVelocityPhase::PreviousMidpoint&&s.reactions_valid&&
        s.reaction_base_epoch==s.epoch-1&&std::isfinite(s.reaction_time)&&s.reaction_time>=0&&
        Bits(s.time)==Bits(s.reaction_time+s.fixed_dt)&&Bits(s.velocity_time)==Bits(s.reaction_time+.5*s.fixed_dt)&&
        Bits(s.reaction_kick_dt)==Bits(s.epoch==1?.5*s.fixed_dt:s.fixed_dt),
        "Source field velocity or first-kick phase is invalid");
}
void AppendSourcePartInputTables(Document& doc,const SourcePartElasticCase& run) {
    Value nodes(rapidjson::kArrayType),parents(rapidjson::kArrayType);
    for(std::size_t n=0;n<NodeCount;++n) {
        Value item(rapidjson::kObjectType); const auto& s=run.source().nodes()[n]; const auto& b=run.binding().nodes()[n];
        UInt(item,doc,"local_node",n); UInt(item,doc,"source_node_id",s.source_id); UInt(item,doc,"canonical_index",s.canonical_index);
        UInt(item,doc,"source_line",s.source_line); const double x[]{b.position.x,b.position.y,b.position.z};
        item.AddMember("reference_xyz_m",FiniteArray(doc,x,3),doc.GetAllocator());
        Scalar(item,doc,"mass_kg",b.native.mass); Scalar(item,doc,"isotropic_inertia_kg_m2",b.native.isotropic_inertia);
        Scalar(item,doc,"physical_inertia_kg_m2",b.native.physical_inertia); Scalar(item,doc,"added_inertia_kg_m2",b.native.added_inertia);
        nodes.PushBack(item,doc.GetAllocator());
    }
    std::size_t qi=0,ti=0;
    for(std::size_t p=0;p<source::ParentCount;++p) {
        const auto& s=run.source().parents()[p]; Value item(rapidjson::kObjectType),connectivity(rapidjson::kArrayType);
        UInt(item,doc,"source_parent_index",p); UInt(item,doc,"source_element_id",s.source_id); UInt(item,doc,"source_line",s.source_line);
        UInt(item,doc,"canonical_index",s.canonical_index); UInt(item,doc,"arity",s.arity);
        UInt(item,doc,"family_index",s.arity==4?qi++:ti++);
        item.AddMember("family",Value(s.arity==4?"QEPH":"T3",doc.GetAllocator()),doc.GetAllocator());
        for(unsigned j=0;j<s.arity;++j) connectivity.PushBack(s.local_node_indices[j],doc.GetAllocator());
        item.AddMember("local_connectivity",connectivity,doc.GetAllocator()); parents.PushBack(item,doc.GetAllocator());
    }
    doc.AddMember("reference_nodes",nodes,doc.GetAllocator()); doc.AddMember("source_parents",parents,doc.GetAllocator());
}
void AppendSourcePartKinematics(Document& doc,const Snapshot& f) {
    const auto& s=f.stamp;
    Integer(doc,"owner_id",s.owner_id); Integer(doc,"accepted_epoch",s.epoch); Number(doc,"accepted_time_s",s.time);
    Number(doc,"fixed_dt_s",s.fixed_dt); String(doc,"temporal_scheme","staggered_half_kick_start");
    String(doc,"velocity_phase",s.velocity_phase==tl::fea::NodalVelocityPhase::Collocated?"collocated":"previous_midpoint");
    Number(doc,"velocity_time_s",s.velocity_time); Boolean(doc,"reactions_valid",s.reactions_valid);
    Integer(doc,"reaction_base_epoch",s.reaction_base_epoch); Number(doc,"reaction_time_s",s.reaction_time);
    Number(doc,"reaction_kick_dt_s",s.reaction_kick_dt);
    FiniteArray(doc,"position_xyz_m",f.position.data(),f.position.size());
    FiniteArray(doc,"orientation_wxyz",f.orientation.data(),f.orientation.size());
    FiniteArray(doc,"velocity_xyz_m_per_s",f.velocity.data(),f.velocity.size());
    FiniteArray(doc,"omega_world_xyz_rad_per_s",f.omega.data(),f.omega.size());
    FiniteArray(doc,"synchronized_velocity_xyz_m_per_s",f.synchronized_velocity.data(),f.synchronized_velocity.size());
    FiniteArray(doc,"synchronized_omega_world_xyz_rad_per_s",f.synchronized_omega.data(),f.synchronized_omega.size());
}
}
