#include "Internal.h"
#include <cstring>
namespace crash::cases::vehicle_wall::native::wall_interface::detail {
namespace hash=vehicle_self_contact::native::detail::digest;
namespace {
std::uint32_t Bits(float value){std::uint32_t out;std::memcpy(&out,&value,sizeof(out));return out;}
}
std::string Digest(const Fields& fields,const Controls& controls,const Provenance& source,
    Declaration declaration,std::size_t cap) {
    hash::Fields out("finite-wall-interface-before-initialization-v1:"+source.source_digest+":"+source.vehicle_digest+":"+source.wall_digest,cap);
    const std::uint64_t identity[]{source.interface_id,declaration.topology_generation,declaration.source_generation,
        std::uint64_t(declaration.profile),fields.nodes.size(),fields.nsv.size(),1,2};
    out.Add<std::uint64_t>("identity_scope",1,std::size(identity),[&](auto i){return identity[i];});
    out.Add<std::uint64_t>("complete_nids",fields.ids.size(),1,[&](auto i){return fields.ids[i];});
    out.Add<double>("native_source_positions",fields.ids.size(),3,[&](auto i){return fields.positions[i];});
    out.Add<std::int32_t>("native_constraint_skew",fields.nodes.size(),2,[&](auto i){return i%2?fields.nodes[i/2].skew:fields.nodes[i/2].constraint;});
    out.Add<std::uint32_t>("nsv",fields.nsv.size(),1,[&](auto i){return fields.nsv[i];});
    out.Add<std::uint32_t>("msr",fields.msr.size(),1,[&](auto i){return fields.msr[i];});
    out.Add<double>("global_contact_K",fields.global_k.size(),1,[&](auto i){return fields.global_k[i];});
    out.Add<double>("secondary_K",fields.secondary_k.size(),1,[&](auto i){return fields.secondary_k[i];});
    out.Add<double>("secondary_gap",fields.secondary_gap.size(),1,[&](auto i){return fields.secondary_gap[i];});
    out.Add<double>("expanded_main_K",2,1,[&](auto i){return fields.main_k[i];});
    out.Add<double>("pre_Buc_main_gaps",2,5,[&](auto i){const auto& row=fields.main_gaps[i/5];return i%5<4?row.corner[i%5]:row.maximum;});
    out.Add<std::uint32_t>("main_nodes",2,4,[&](auto i){return fields.starter.mains[i/4].nodes[i%4];});
    out.Add<std::int32_t>("main_role_refs",2,5,[&](auto i){const auto& m=fields.starter.mains[i/5];return i%5==0?m.segment_type:m.normal_reference[i%5-1];});
    const auto normal=[&](const char* name,const s::NormalView& view) {
        out.Add<std::uint32_t>(name,2,12,[&](auto i){const auto x=view.face_normals[i/3];return Bits(i%3==0?x.x:i%3==1?x.y:x.z);});
    };
    normal("Starter_normal_bits",fields.starter.starter);normal("fixed_ready_normal_bits",fields.ready.normals);
    const auto reference=[&](const char* name,const s::NormalView& view) {
        out.Add<std::uint32_t>(name,view.reference_count,7,[&](auto i) {
            const auto& row=view.references[i/7];const auto lane=i%7;
            if(!lane)return std::uint32_t(row.boundary);
            const auto v=row.bisector[(lane-1)/3];return Bits((lane-1)%3==0?v.x:(lane-1)%3==1?v.y:v.z);
        });
    };
    reference("Starter_reference_bits",fields.starter.starter);reference("fixed_ready_reference_bits",fields.ready.normals);
    out.Add<std::uint32_t>("normal_offsets",fields.starter.starter.reference_count+1,1,[&](auto i){return fields.starter.normal_offsets[i];});
    out.Add<std::uint32_t>("normal_mains",fields.starter.normal_incidence_count,1,[&](auto i){return fields.starter.normal_mains[i];});
    // Named source policy fixes all flags; retain independent dimensional values.
    const double values[]{controls.runtime.units.length_m,controls.runtime.units.mass_kg,controls.runtime.units.time_s,
        controls.runtime.friction_coefficients.base,controls.runtime.normal.damping_factor,
        controls.runtime.lifecycle.maximum_coefficient,controls.gaps.maximum_secondary,controls.gaps.maximum_main};
    out.Add<double>("declared_policy_values",1,std::size(values),[&](auto i){return values[i];});
    return out.Finish().sha256;
}
}
