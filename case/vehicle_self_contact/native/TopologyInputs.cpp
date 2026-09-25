#include "TopologyAssessmentInternal.h"
#include <algorithm>
#include <limits>
namespace crash::cases::vehicle_self_contact::native::detail {
namespace {
void Add(std::size_t& total,std::size_t count,std::size_t width) {
  output::Require(width && count<=(SIZE_MAX-total)/width,"Assessment forecast overflow");total+=count*width;
}
bool Retained(const source::CanonicalData& c,std::uint64_t pid) {
  return std::binary_search(c.selected_parts.begin(),c.selected_parts.end(),pid);
}
void Check(const source::CanonicalData& c,const selection::Data& d,Config config,Limits cap) {
  const Limits hard;const s::Limits topology;
  output::Require(config.selection==SelectionPolicy::CanonicalRetainedOriginalShells &&
      config.order==Order::CanonicalRecordOrder && config.working_units==WorkingUnits::SourceDeclaredUnits,
      "Unknown topology assessment policy");
  output::Require(cap.host_bytes&&cap.host_bytes<=hard.host_bytes&&cap.metadata_bytes&&cap.metadata_bytes<=hard.metadata_bytes&&
      cap.nodes&&cap.nodes<=hard.nodes&&cap.parents&&cap.parents<=hard.parents&&
      cap.topology.max_output_bytes<=topology.max_output_bytes&&cap.topology.max_scratch_bytes<=topology.max_scratch_bytes,
      "Invalid topology assessment limits");
  output::Require(c.canonical_nodes&&c.canonical_nodes<=cap.nodes&&c.canonical_shells&&c.canonical_shells<=cap.parents&&
      d.counts.retained_shells&&d.counts.retained_shells<=cap.parents&&d.counts.retained_shells<=c.canonical_shells&&
      d.parts.size()==d.selected_part_ids.size()&&d.parts.size()<=selection::Limits{}.parts&&
      d.profile==selection::OriginalSelectionProfile::AutomaticSingleSurfacePartSetV1,
      "Invalid complete canonical/original selection counts");
  source::CheckUnits(c.inputs.units);
  output::Require(std::is_sorted(c.selected_parts.begin(),c.selected_parts.end())&&
      std::is_sorted(d.selected_part_ids.begin(),d.selected_part_ids.end()),"Unordered retained source policy");
  std::size_t retained=0;
  for(std::size_t i=0;i<d.parts.size();++i) {
    const auto& part=d.parts[i];
    output::Require(part.part_id==d.selected_part_ids[i]&&(!i||d.parts[i-1].part_id<part.part_id)&&
        part.retained_shell_part==Retained(c,part.part_id)&&
        (!part.shells||part.shell_section),"Original part disposition differs from canonical source");
    if(part.retained_shell_part)Add(retained,part.shells,1);
  }
  output::Require(retained==d.counts.retained_shells,"Original retained shell census differs");
}
template<class T> std::vector<T> Decode(const source::CanonicalData& c,const char* name,std::size_t rows,std::size_t columns) {
  const auto& a=source::FindArray(c,name);
  output::Require(a.descriptor.layout.rows==rows&&a.descriptor.layout.columns==columns,"Canonical assessment array shape differs");
  return output::arrays::Decode<T>(a.descriptor,a.bytes);
}
const selection::PartDisposition* Part(const selection::Data& d,std::uint64_t pid) {
  const auto it=std::lower_bound(d.parts.begin(),d.parts.end(),pid,[](const auto& a,auto value){return a.part_id<value;});
  return it==d.parts.end()||it->part_id!=pid?nullptr:&*it;
}
}
s::Input Inputs::View() const {
  s::Input in;in.profile=s::Profile::OrdinaryExteriorMovingMain;in.topology=s::TopologyPolicy::NativeOrdinaryShell;
  in.node_source_ids=node_ids.data();in.node_count=node_ids.size();in.positions={positions.data(),std::uint32_t(node_ids.size()),3,1};
  in.primary=primary.data();in.primary_count=primary.size();in.coordinates=s::Coordinates::Si;in.units=units;
  in.source_generation=1; // Local immutable assessment generation, not a physical epoch/authority.
  return in;
}
Forecast Plan(const source::CanonicalData& c,const selection::Data& d,Config config,Limits cap) {
  Check(c,d,config,cap);Forecast f;
  f.canonical_source_reservation=c.limits.host_bytes;
  f.selection_source_reservation=std::max(d.startup_budget_bytes,d.owned_payload_bytes);
  f.input_owned_bytes=sizeof(Inputs);
  Add(f.input_owned_bytes,c.canonical_nodes,sizeof(std::uint64_t)+3*sizeof(double));
  Add(f.input_owned_bytes,d.counts.retained_shells,sizeof(s::PrimaryFace)+sizeof(ParentOrigin));
  // Decode creates typed copies through the existing bounded codec. Keep a
  // conservative two-copy reservation for complete source rows/indices/lines.
  for(const auto* name:{"node_ids","node_positions","shells_records","shells_node_indices","shells_source_lines"})
    Add(f.input_decode_peak_bytes,source::FindArray(c,name).descriptor.bytes,2);
  // Copied report and original dispositions survive the arenas. Charge them
  // independently of a caller-selected metadata serialization cap.
  f.result_owned_bytes=sizeof(Result);Add(f.result_owned_bytes,d.parts.size(),sizeof(selection::PartDisposition));
  for(const auto* text:{&c.inputs.canonical_manifest.file,&c.inputs.canonical_manifest.sha256,
      &c.inputs.scope_report.file,&c.inputs.scope_report.sha256,&c.inputs.source_member.file,&c.inputs.source_member.sha256,
      &d.auxiliary_sha256,&d.combine_sha256,&c.inputs.units.length,&c.inputs.units.mass,&c.inputs.units.time})
    Add(f.result_owned_bytes,text->capacity()+1,1);
  // Input/output roots plus the two field-vector growth capacities (16 each).
  // The generated names/dtypes/hash strings fit the fixed per-field allowance.
  Add(f.result_owned_bytes,2,128);Add(f.result_owned_bytes,32,sizeof(FieldDigest)+256);
  f.digest_scratch_bytes=4u<<20;
  s::Input in;in.profile=s::Profile::OrdinaryExteriorMovingMain;in.topology=s::TopologyPolicy::NativeOrdinaryShell;
  in.node_count=c.canonical_nodes;in.primary_count=d.counts.retained_shells;f.topology=s::Preflight(in,cap.topology);
  f.main_size=sizeof(s::Main);f.reference_size=sizeof(s::NormalReference);f.primary_size=sizeof(s::PrimaryFace);
  std::size_t common=f.canonical_source_reservation;Add(common,f.selection_source_reservation,1);Add(common,f.input_owned_bytes,1);
  Add(common,f.result_owned_bytes,1);
  // Additional document construction reserve, not a substitute for copied result storage or measured RSS.
  Add(common,cap.metadata_bytes,2);
  auto inputs=common;Add(inputs,f.input_decode_peak_bytes,1);Add(inputs,f.digest_scratch_bytes,1);
  auto topology=common;Add(topology,f.topology.output_bytes,1);Add(topology,f.topology.scratch_bytes,1);
  auto digest=common;Add(digest,f.topology.output_bytes,1);Add(digest,f.digest_scratch_bytes,1);
  f.peak_host_bytes=std::max({inputs,topology,digest});
  f.admitted=f.topology.status==s::Status::Ok && f.peak_host_bytes<=cap.host_bytes;return f;
}
Inputs PrepareInputs(const source::CanonicalData& c,const selection::Data& d,Config config,Limits cap,Counts& counts) {
  Check(c,d,config,cap);Inputs out;
  out.units={c.inputs.units.length_to_m,c.inputs.units.mass_to_kg,c.inputs.units.time_to_s};
  out.node_ids=Decode<std::uint64_t>(c,"node_ids",c.canonical_nodes,1);
  out.positions=Decode<double>(c,"node_positions",c.canonical_nodes,3);
  const auto rows=Decode<std::uint64_t>(c,"shells_records",c.canonical_shells,6);
  const auto indices=Decode<std::uint32_t>(c,"shells_node_indices",c.canonical_shells,4);
  const auto lines=Decode<std::uint32_t>(c,"shells_source_lines",c.canonical_shells,1);
  out.primary.reserve(d.counts.retained_shells);out.origin.reserve(d.counts.retained_shells);
  counts.original=d.counts;counts.canonical_nodes=c.canonical_nodes;counts.canonical_shells=c.canonical_shells;
  std::vector<std::size_t> by_part(d.parts.size(),0);
  for(std::size_t row=0;row<c.canonical_shells;++row) {
    const auto* part=Part(d,rows[6*row+1]);
    if(!part||!part->retained_shell_part)continue; // Exactly the explicit source selection, no geometric pruning.
    output::Require(out.primary.size()<d.counts.retained_shells,"Selected input exceeds complete census");
    s::PrimaryFace face;face.source_id=rows[6*row];
    const bool triangle=indices[4*row+2]==indices[4*row+3];
    face.layout=triangle?tlfea::contact::radioss_type25::ShellLayout::Triangle3:tlfea::contact::radioss_type25::ShellLayout::Quad4;
    for(unsigned k=0;k<4;++k) {
      face.nodes[k]=indices[4*row+k];
      output::Require(face.nodes[k]<out.node_ids.size()&&out.node_ids[face.nodes[k]]==rows[6*row+2+k],
          "Canonical selected connectivity/source identity differs");
    }
    output::Require(face.source_id&&lines[row],"Selected source identity/line is missing");
    out.primary.push_back(face);out.origin.push_back({std::uint32_t(row),lines[row],part->part_id});
    ++by_part[std::size_t(part-d.parts.data())];++(triangle?counts.selected_t3:counts.selected_q4);
  }
  for(std::size_t i=0;i<d.parts.size();++i)if(d.parts[i].retained_shell_part) {
    output::Require(by_part[i]==d.parts[i].shells,"Complete selected part coverage differs");++counts.selected_parts;
  }
  counts.selected_parents=out.primary.size();
  output::Require(counts.selected_parents==d.counts.retained_shells,"Complete selected parent coverage differs");
  return out; // Full raw records/indices/lines expire before TL arenas are allocated.
}
Location MapLocation(const Inputs& inputs,std::size_t primary,std::size_t node) {
  Location result;
  if(primary!=SIZE_MAX) {
    output::Require(primary<inputs.primary.size(),"TL returned an invalid primary location");
    const auto& origin=inputs.origin[primary];result.source_eid=inputs.primary[primary].source_id;
    result.source_pid=origin.pid;result.canonical_row=origin.canonical_row;result.source_line=origin.source_line;
  }
  if(node!=SIZE_MAX){output::Require(node<inputs.node_ids.size(),"TL returned an invalid node location");result.source_node_id=inputs.node_ids[node];}
  return result;
}
} // namespace crash::cases::vehicle_self_contact::native::detail
