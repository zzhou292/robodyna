#include "TopologyAssessmentInternal.h"
#include "TopologyDigestFields.h"
#include <algorithm>
#include <cstring>
#include <limits>
namespace crash::cases::vehicle_self_contact::native::detail {
using digest::Fields;
namespace {
std::uint32_t FloatBits(float value) {
  static_assert(sizeof(float)==sizeof(std::uint32_t));
  std::uint32_t bits;std::memcpy(&bits,&value,sizeof(bits));return bits;
}

}
Digest InputDigest(const Inputs& in,const Config& config,std::size_t cap) {
  output::Require(in.node_ids.size()<=SIZE_MAX/3 && in.positions.size()==3*in.node_ids.size() &&
      in.origin.size()==in.primary.size(),"Incomplete topology input digest extents");
  Fields fields("canonical-record-order:si-to-declared-native-working-units",cap);
  fields.Add<std::uint64_t>("policy",1,3,[&](std::size_t i){return std::uint64_t(i==0?int(config.selection):i==1?int(config.order):int(config.working_units));});
  fields.Add<double>("declared_unit_scale",1,3,[&](std::size_t i){return i==0?in.units.length_m:i==1?in.units.mass_kg:in.units.time_s;});
  fields.Add<std::uint64_t>("node_ids",in.node_ids.size(),1,[&](auto i){return in.node_ids[i];});
  fields.Add<double>("canonical_position_m",in.node_ids.size(),3,[&](auto i){return in.positions[i];});
  fields.Add<std::uint64_t>("primary_source_ids",in.primary.size(),1,[&](auto i){return in.primary[i].source_id;});
  fields.Add<std::uint32_t>("primary_nodes",in.primary.size(),4,[&](auto i){return in.primary[i/4].nodes[i%4];});
  fields.Add<std::uint32_t>("primary_layout",in.primary.size(),1,[&](auto i){return std::uint32_t(in.primary[i].layout);});
  fields.Add<std::uint64_t>("primary_pid",in.origin.size(),1,[&](auto i){return in.origin[i].pid;});
  fields.Add<std::uint32_t>("canonical_row",in.origin.size(),1,[&](auto i){return in.origin[i].canonical_row;});
  fields.Add<std::uint32_t>("source_line",in.origin.size(),1,[&](auto i){return in.origin[i].source_line;});
  return fields.Finish();
}
Digest TopologyDigest(const s::Snapshot& v,const std::string& input,std::size_t cap) {
  output::Require(input.size()==64,"Topology digest requires its source input binding");
  output::arrays::CheckHash(input);
  output::Require(v.mains&&v.starter.face_normals&&v.starter.references&&v.expanded_to_primary&&v.primary_to_partner&&
      v.normal_offsets&&v.normal_mains,"Incomplete successful topology digest view");
  Fields f(input,cap);const auto g=v.main_count,r=v.starter.reference_count;
  const std::uint64_t header[]{v.node_count,v.primary_count,g,r,v.normal_incidence_count,v.source_generation,std::uint64_t(v.profile),std::uint64_t(v.topology)};
  f.Add<std::uint64_t>("header",1,8,[&](auto i){return header[i];});
  f.Add<std::uint64_t>("main_source_ids",g,1,[&](auto i){return v.mains[i].source_id;});
  f.Add<std::uint32_t>("main_nodes",g,4,[&](auto i){return v.mains[i/4].nodes[i%4];});
  f.Add<std::int32_t>("main_global_ids",g,1,[&](auto i){return std::int32_t(v.mains[i].global_id);});
  f.Add<std::int32_t>("main_segment_type",g,1,[&](auto i){return std::int32_t(v.mains[i].segment_type);});
  f.Add<std::int32_t>("main_neighbors",g,4,[&](auto i){return std::int32_t(v.mains[i/4].neighbors[i%4]);});
  f.Add<std::int32_t>("main_neighbor_edges",g,4,[&](auto i){return std::int32_t(v.mains[i/4].neighbor_edges[i%4]);});
  f.Add<std::int32_t>("main_references",g,4,[&](auto i){return std::int32_t(v.mains[i/4].normal_reference[i%4]);});
  f.Add<std::uint32_t>("expanded_to_primary",g,1,[&](auto i){return v.expanded_to_primary[i];});
  f.Add<std::uint32_t>("primary_to_partner",v.primary_count,1,[&](auto i){return v.primary_to_partner[i];});
  f.Add<std::uint32_t>("normal_offsets",r+1,1,[&](auto i){return v.normal_offsets[i];});
  f.Add<std::uint32_t>("normal_mains",v.normal_incidence_count,1,[&](auto i){return v.normal_mains[i];});
  f.Add<std::uint32_t>("starter_normals_f32",4*g,3,[&](auto i){const auto& x=v.starter.face_normals[i/3];return FloatBits(i%3==0?x.x:i%3==1?x.y:x.z);});
  f.Add<std::int32_t>("starter_boundary",r,1,[&](auto i){return std::int32_t(v.starter.references[i].boundary);});
  f.Add<std::uint32_t>("starter_bisectors_f32",r,6,[&](auto i){const auto& x=v.starter.references[i/6].bisector[(i%6)/3];return FloatBits(i%3==0?x.x:i%3==1?x.y:x.z);});
  if (v.primary_role_count) {
    output::Require(v.primary_roles && v.primary_role_count == v.primary_count,
                    "Incomplete resolved topology role provenance");
    f.Add<std::uint32_t>("primary_source_roles", v.primary_role_count, 1,
        [&](auto i) { return std::uint32_t(v.primary_roles[i]); });
  }
  return f.Finish();
}
} // namespace crash::cases::vehicle_self_contact::native::detail
