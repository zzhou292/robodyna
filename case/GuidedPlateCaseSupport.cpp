#include "GuidedPlateCaseSupport.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace crash::case_data::guided_detail {
contact::PlanarWallView ContactWall::view() const noexcept {
    return {vertices.data(),static_cast<std::uint32_t>(vertices.size()),
            triangles.data(),static_cast<std::uint32_t>(triangles.size())};
}
bool CopyCanonicalWall(const CanonicalWall& source,ContactWall& output,std::string& error) {
    // These are declared provenance pins, not authentication of source bytes.
    // The caller loads the retained canonical artifact through CanonicalWall.
    if (!source.loaded() || source.vertices().size()!=62 || source.triangles().size()!=100 ||
        source.source_quads().size()!=46 || source.provenance().model_archive_reference_sha256!=
            "aff8194c456726a678d6cc11f644316ca70f3d9b37c4db622726b7b2985b0451") {
        error="Guided plate requires the canonical 62-vertex/100-triangle/46-quad Yaris wall";
        return false;
    }
    ContactWall next; next.provenance=source.provenance();
    next.vertices.reserve(source.vertices().size()); next.triangles.reserve(source.triangles().size());
    for (const auto& v:source.vertices())
        next.vertices.push_back({{v.position_m[0],v.position_m[1],v.position_m[2]},v.source_node_id,v.assembled_source_node_id});
    for (const auto& t:source.triangles())
        next.triangles.push_back({{t.vertex_indices[0],t.vertex_indices[1],t.vertex_indices[2]},
                                 t.triangle_id,t.source_quad_id,t.assembled_source_quad_id});
    output=std::move(next); error.clear(); return true;
}
bool PrepareTessellatedContact(const WallTessellation& wall,const ref::GuidedPlateModel& model,
                              contact::Q4PlanarGeometry& output,std::string& error) {
    const auto& data=model.shell().data(); const auto& guided=model.data();
    InitialState initial(data.reference_configuration);
    const contact::Q4SurfaceView surface{{initial.x.data(),ref::kCouponNodes,3,1},
        {initial.v.data(),ref::kCouponNodes,3,1},guided.parents.data(),ref::kCouponElements};
    const contact::Q4FixedYZMassView mass{data.inverse_mass.data(),guided.translation_fixed_bits.data(),ref::kCouponNodes,0};
    contact::Q4PlanarGeometry next;
    const auto coverage=wall.CheckCoverage(surface,mass,guided.exposed_clearance,next);
    if (coverage.status!=WallTessellationStatus::Ok) { error=coverage.diagnostic; return false; }
    const auto original=model.contact_geometry().view(),derived=next.view();
    auto interval=[](const contact::Q4IntegralInterval& a,const contact::Q4IntegralInterval& b) {
        return a.lower==b.lower && a.upper==b.upper;
    };
    bool same=original.parent_count==derived.parent_count && original.global_node_count==derived.global_node_count &&
        original.wall_x==derived.wall_x && original.wall_tolerance==derived.wall_tolerance;
    for (unsigned p=0;same && p<original.parent_count;++p) {
        const auto& a=original.parents[p]; const auto& b=derived.parents[p];
        same=a.covered && b.covered && a.projected_area==b.projected_area && interval(a.area_enclosure,b.area_enclosure) &&
            a.parent.feature_id==b.parent.feature_id && a.parent.parent_element_id==b.parent.parent_element_id &&
            a.parent.parent_face_id==b.parent.parent_face_id && a.parent.half_thickness==b.parent.half_thickness;
        for (unsigned n=0;same && n<4;++n)
            same=a.parent.nodes[n]==b.parent.nodes[n] && a.reference_projection[n].x==b.reference_projection[n].x &&
                a.reference_projection[n].y==b.reference_projection[n].y && a.reference_projection[n].z==b.reference_projection[n].z;
    }
    contact::Q4PlanarStiffness stiffness;
    const auto& audited=model.contact_stiffness();
    if (!same || contact::BuildQ4PlanarStiffness(derived,mass,guided.stiffness_per_area,&stiffness)!=contact::PlanarContactStatus::Ok ||
        !stiffness.valid || !audited.valid || stiffness.count!=audited.count || stiffness.rate_bound!=audited.rate_bound) {
        error="Derived wall changes the original physical Q4 preparation or contact rate"; return false;
    }
    for (unsigned i=0;i<stiffness.count;++i) {
        same=same && stiffness.nodes[i]==audited.nodes[i] && stiffness.inverse_mass[i]==audited.inverse_mass[i];
        for (unsigned j=0;j<stiffness.count;++j) same=same && interval(stiffness.entry[i][j],audited.entry[i][j]);
    }
    if (!same) { error="Derived wall changes the original all-active contact stiffness table"; return false; }
    output=next; error.clear(); return true;
}
std::string Describe(const shell::ShellBatchReport& r) {
    return std::string(r.message)+"; shell_status="+std::to_string(static_cast<int>(r.status))+
        ", element="+std::to_string(r.element)+", node="+std::to_string(r.node)+
        ", element_status="+std::to_string(static_cast<int>(r.element_status));
}
std::string Describe(const contact::Q4PlanarContactReport& r) {
    return std::string(r.message)+"; contact_status="+std::to_string(static_cast<int>(r.status))+
        ", parent="+std::to_string(r.parent)+", node="+std::to_string(r.node)+
        ", geometry_status="+std::to_string(static_cast<int>(r.geometry))+
        ", integral_status="+std::to_string(static_cast<int>(r.integration.status));
}
visual::Binding SurfaceBinding(const fea::NodalStamp& owner,const ref::GuidedPlateData& data) {
    visual::Binding result; result.identity={owner.owner_id,1,kGuidedPlateQualification};
    result.tl_node_count=ref::kCouponNodes;
    for (unsigned n=0;n<ref::kCouponNodes;++n) result.vertices.push_back({n,{3,1,n+1}});
    for (const auto& parent:data.parents) {
        const auto* n=parent.nodes;
        result.triangles.push_back({{n[0],n[1],n[2]},3,1,parent.parent_element_id,1,parent.parent_face_id,0});
        result.triangles.push_back({{n[0],n[2],n[3]},3,1,parent.parent_element_id,1,parent.parent_face_id,1});
    }
    return result;
}
std::array<shell::ReissnerShellBatchElement,ref::kCouponElements> ShellElements(const ref::ElasticCouponData& data) {
    std::array<shell::ReissnerShellBatchElement,ref::kCouponElements> result;
    for (unsigned e=0;e<ref::kCouponElements;++e) {
        result[e].reference=data.reference[e]; result[e].section=data.section[e];
        for (unsigned n=0;n<4;++n) result[e].nodes[n]=data.connectivity[e][n];
    }
    return result;
}
InitialState::InitialState(const ref::ElasticCouponConfiguration& configuration) {
    for (unsigned n=0;n<ref::kCouponNodes;++n) {
        const auto p=configuration.position[n]; const auto r=configuration.rotation[n];
        x[3*n]=p.x; x[3*n+1]=p.y; x[3*n+2]=p.z;
        q[4*n]=r.w; q[4*n+1]=r.x; q[4*n+2]=r.y; q[4*n+3]=r.z;
    }
}
bool SameStamp(const fea::NodalStamp& a,const fea::NodalStamp& b) {
    return a.owner_id==b.owner_id && a.epoch==b.epoch && a.node_count==b.node_count &&
        a.time==b.time && a.fixed_dt==b.fixed_dt && a.has_rotations==b.has_rotations &&
        a.reactions_valid==b.reactions_valid && a.reaction_base_epoch==b.reaction_base_epoch && a.reaction_time==b.reaction_time;
}
bool Matches(const shell::ShellBatchDiagnostics& s,const contact::Q4PlanarContactDiagnostics& c,
             std::uint64_t owner,std::uint64_t epoch,std::uint64_t attempt,bool candidate,std::uint64_t wall_binding,
             contact::Q4PlanarIntegrationBackend backend) {
    return s.valid && ValidGuidedContactPartition(c,backend) && owner && attempt && s.owner_id==owner && c.owner_id==owner &&
        s.base_epoch==epoch && c.base_epoch==epoch && s.attempt==attempt && c.attempt==attempt &&
        s.configuration_id==kGuidedPlateQualification && c.configuration_id==kGuidedPlateQualification &&
        wall_binding && c.wall_binding_id==wall_binding &&
        s.phase==(candidate?shell::ShellBatchPhase::kPreparedCandidate:shell::ShellBatchPhase::kAcceptedBase) &&
        c.phase==(candidate?contact::Q4PlanarContactPhase::PreparedCandidate:contact::Q4PlanarContactPhase::AcceptedBase);
}
bool MatchesPrepared(const fea::NodalPreparedView& p,const fea::NodalAssemblyView& assembly,const fea::NodalStamp& base) {
    return p.owner_id==base.owner_id && p.owner_id==assembly.owner_id && p.attempt==assembly.attempt &&
        p.kinematics.base_epoch==base.epoch && p.base_kinematics.base_epoch==base.epoch &&
        assembly.accepted.base_epoch==base.epoch && p.stream==assembly.stream &&
        p.kinematics.node_count==base.node_count && p.base_kinematics.node_count==base.node_count &&
        std::isfinite(base.fixed_dt) && base.fixed_dt>0 &&
        std::isfinite(p.proposed_time) && p.proposed_time==base.time+base.fixed_dt && p.proposed_time>base.time;
}
bool MatchesAcceptedResults(const shell::ShellBatchDiagnostics& s,const contact::Q4PlanarContactDiagnostics& c,
                            const fea::NodalStamp& stamp,std::uint64_t wall_binding,contact::Q4PlanarIntegrationBackend backend) {
    const bool candidate=s.phase==shell::ShellBatchPhase::kPreparedCandidate;
    if (candidate && !stamp.epoch) return false;
    return Matches(s,c,stamp.owner_id,candidate?stamp.epoch-1:stamp.epoch,s.attempt,candidate,wall_binding,backend);
}
bool MatchesContactParents(const std::array<contact::Q4PlanarParentResult,ref::kCouponElements>& parents,
                          const std::array<contact::SurfaceQ4,ref::kCouponElements>& source,
                          const contact::Q4PlanarContactDiagnostics& d,contact::Q4PlanarIntegrationBackend backend) {
    if (!ValidGuidedContactPartition(d,parents.data(),parents.size(),backend)) return false;
    for (unsigned p=0;p<parents.size();++p) {
        const auto& r=parents[p].integration;
        if (!parents[p].covered || r.feature_id!=source[p].feature_id || r.parent_element_id!=source[p].parent_element_id ||
            r.base_epoch!=d.base_epoch || r.attempt!=d.attempt) return false;
        for (unsigned n=0;n<4;++n) if (r.nodal.nodes[n]!=source[p].nodes[n]) return false;
    }
    return true;
}
cudaError_t ReadAuditConfiguration(const fea::NodalPreparedView& prepared,ref::ElasticCouponConfiguration& output) {
    static_assert(sizeof(shell::Vec3)==3*sizeof(double) && sizeof(shell::Quaternion)==4*sizeof(double));
    auto result=cudaGetLastError();
    if (result==cudaSuccess) result=cudaMemcpyAsync(output.position.data(),prepared.kinematics.position_xyz,
        sizeof(output.position),cudaMemcpyDeviceToHost,prepared.stream);
    if (result==cudaSuccess) result=cudaMemcpyAsync(output.rotation.data(),prepared.kinematics.orientation_wxyz,
        sizeof(output.rotation),cudaMemcpyDeviceToHost,prepared.stream);
    if (result==cudaSuccess) result=cudaStreamSynchronize(prepared.stream);
    return result;
}
bool AccumulateInterval(const shell::ShellBatchDiagnostics& s,const contact::Q4PlanarContactDiagnostics& base,
                        const contact::Q4PlanarContactDiagnostics& endpoint,GuidedPlateMetrics& next) {
    if (!ValidGuidedContactPartition(base,next.contact.integration_backend) ||
        !ValidGuidedContactPartition(endpoint,next.contact.integration_backend)) return false;
    const double h=next.stamp.fixed_dt;
    next.shell=s; next.contact=endpoint; next.applied_contact=base;
    next.wall_impulse=contact::Add(next.wall_impulse,contact::Scale(base.wall_reaction,h));
    next.wall_moment_impulse=contact::Add(next.wall_moment_impulse,contact::Scale(base.wall_moment,h));
    next.shell_midpoint_work+=s.kinetic_midpoint_work; next.contact_midpoint_work+=endpoint.kinetic_midpoint_work;
    next.shell_coordinate_work+=s.force_coordinate_work; next.contact_coordinate_work+=endpoint.force_coordinate_work;
    next.maximum_relative_energy_error=std::max(next.maximum_relative_energy_error,next.work.relative_energy_error);
    next.peak_penetration=std::max(next.peak_penetration,endpoint.maximum_penetration);
    for (double value:{next.shell_midpoint_work,next.contact_midpoint_work,next.shell_coordinate_work,
                       next.contact_coordinate_work,next.maximum_relative_energy_error,next.peak_penetration})
        if (!std::isfinite(value)) return false;
    return contact::IsFinite(next.wall_impulse) && contact::IsFinite(next.wall_moment_impulse);
}
} // namespace crash::case_data::guided_detail
