#include "TopologyAssessment.h"
#include <stdexcept>
#include "output/BoundedArrayJson.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
namespace crash::cases::vehicle_self_contact::native {
namespace {
using output::Boolean;using output::Integer;using output::Number;using output::String;using output::array_json::Child;
output::Document Object(){output::Document d;d.SetObject();return d;}
void Bound(const output::Document& d,std::size_t cap) {
  output::Require(cap&&cap<=1u<<20,"Invalid assessment metadata cap");
  rapidjson::StringBuffer buffer;rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
  output::Require(d.Accept(writer)&&buffer.GetSize()<cap,"Assessment document exceeds byte cap"); // WriteJson adds one newline.
}
const char* Status(s::Status status) {
  switch(status) {
    case s::Status::Ok:return "ok";case s::Status::InvalidInput:return "invalid_input";
    case s::Status::UnsupportedProfile:return "unsupported_profile";case s::Status::UnsupportedTopology:return "unsupported_topology";
    case s::Status::NonfiniteResult:return "nonfinite_result";case s::Status::UnsupportedArithmetic:return "unsupported_arithmetic";
    case s::Status::ResourceLimit:return "resource_limit";
  }
  throw std::runtime_error("Unknown TL topology result");
}
void Optional(output::Document& d,const char* key,std::size_t value) {
  if(value==SIZE_MAX)d.AddMember(output::Value(key,d.GetAllocator()),output::Value(),d.GetAllocator());
  else Integer(d,key,value);
}
output::Document LocationDocument(const Location& x) {
  auto d=Object();Boolean(d,"source_parent_available",x.source_eid!=0);Boolean(d,"source_node_available",x.source_node_id!=0);
  Optional(d,"canonical_row",x.canonical_row);Optional(d,"expanded_main",x.expanded_main);Optional(d,"edge",x.edge);
  Integer(d,"source_eid",x.source_eid);Integer(d,"source_pid",x.source_pid);Integer(d,"source_node_id",x.source_node_id);Integer(d,"source_line",x.source_line);return d;
}
output::Document Record(const output::full_shell::RecordFile& x) {
  auto d=Object();String(d,"file",x.file);String(d,"sha256",x.sha256);Integer(d,"bytes",x.bytes);return d;
}
output::Document DigestDocument(const Digest& value) {
  output::Require(value.fields.size()<=32,"Too many assessment digest fields");
  output::arrays::CheckHash(value.sha256);
  for(const auto& field:value.fields) {
    output::Require(field.name.size()<=128 && field.type.size()<=8,"Oversized assessment field label");
    output::arrays::CheckHash(field.sha256);
  }
  auto d=Object();String(d,"algorithm","robo_dyna.native_topology_fields.v1");String(d,"sha256",value.sha256);
  String(d,"encoding","typed little-endian scalar words;65536-word chunks;ordered field/type/shape and chunk hashes;float32 preserved asUInt32 bits");
  output::Value fields(rapidjson::kArrayType);
  for(const auto& f:value.fields) {
    auto row=Object();String(row,"name",f.name);String(row,"type",f.type);String(row,"sha256",f.sha256);
    Integer(row,"rows",f.rows);Integer(row,"columns",f.columns);Integer(row,"chunks",f.chunks);
    output::Value child;child.CopyFrom(row,d.GetAllocator());fields.PushBack(child,d.GetAllocator());
  }
  d.AddMember("fields",fields,d.GetAllocator());return d;
}
output::Document OriginalCounts(const selection::Counts& x) {
  auto d=Object();Integer(d,"selected_parts",x.selected_parts);Integer(d,"shell_parts",x.shell_parts);
  Integer(d,"retained_shell_parts",x.retained_shell_parts);Integer(d,"excluded_shell_parts",x.excluded_shell_parts);
  Integer(d,"non_shell_parts",x.non_shell_parts);Integer(d,"shells",x.shells);Integer(d,"retained_shells",x.retained_shells);
  Integer(d,"excluded_shells",x.excluded_shells);Integer(d,"unassessed_solids",x.solids);Integer(d,"unassessed_beams",x.beams);return d;
}
}
output::Document ForecastDocument(const Forecast& x,std::size_t cap) {
  auto d=Object();String(d,"schema","robo_dyna.native_topology_forecast.v1");Boolean(d,"admitted",x.admitted);
  String(d,"scope","application reservations and compiled TL counts/sizeof;not measured residency or source/physical admission");
  Integer(d,"canonical_source_reservation",x.canonical_source_reservation);Integer(d,"selection_source_reservation",x.selection_source_reservation);
  Integer(d,"input_owned_bytes",x.input_owned_bytes);Integer(d,"input_decode_peak_bytes",x.input_decode_peak_bytes);
  Integer(d,"result_owned_bytes",x.result_owned_bytes);
  Integer(d,"digest_scratch_bytes",x.digest_scratch_bytes);Integer(d,"peak_host_bytes",x.peak_host_bytes);
  Integer(d,"sizeof_main",x.main_size);Integer(d,"sizeof_reference",x.reference_size);Integer(d,"sizeof_primary",x.primary_size);
  const auto& t=x.topology;auto tl=Object();String(tl,"status",Status(t.status));
  Integer(tl,"output_bytes",t.output_bytes);Integer(tl,"scratch_bytes",t.scratch_bytes);Integer(tl,"ready_output_bytes",t.ready_output_bytes);
  Integer(tl,"ready_scratch_bytes",t.ready_scratch_bytes);Integer(tl,"expanded_mains",t.expanded_mains);
  Integer(tl,"maximum_references",t.maximum_references);Integer(tl,"maximum_incidence",t.maximum_incidence);
  Child(d,"tl",tl);Bound(d,cap);return d;
}
output::Document ResultDocument(const Result& x,std::size_t cap) {
  output::Require(x.parts.size()<=selection::Limits{}.parts &&
      x.config.selection==SelectionPolicy::CanonicalRetainedOriginalShells &&
      x.config.order==Order::CanonicalRecordOrder && x.config.working_units==WorkingUnits::SourceDeclaredUnits,
      "Invalid assessment result profile or part extent");
  output::Require(x.output_complete==(x.topology_report.status==s::Status::Ok)&&
      x.input_digest.sha256.size()==64&&(!x.output_complete||x.output_digest.sha256.size()==64),"Incomplete assessment result");
  auto d=Object();String(d,"schema","robo_dyna.native_topology_assessment.v1");
  String(d,"scope","complete selected canonical ordinary-shell geometry assessment;not runtime source/solver/GPU admission");
  String(d,"status",Status(x.topology_report.status));Boolean(d,"output_complete",x.output_complete);
  String(d,"selection","OriginalSelection retained shell PIDs;all canonical nodes");
  String(d,"order","canonical shell-record order;not proven native full-case order");
  String(d,"coordinates","untransformed canonical SI coordinates;existing TL division into declared source working length");
  String(d,"roundtrip_limit","SI-to-native working values do not establish original source-decimal double identity");
  String(d,"startup_profile","OrdinaryExteriorMovingMain/NativeOrdinaryShell;local assessment generation1");
  Boolean(d,"geometry_deletion_retry",false);Boolean(d,"physical_shell_formulations_admitted",false);
  Child(d,"forecast",ForecastDocument(x.forecast,cap));auto counts=Object();Child(counts,"original",OriginalCounts(x.counts.original));
  Integer(counts,"canonical_nodes",x.counts.canonical_nodes);Integer(counts,"canonical_shells",x.counts.canonical_shells);
  Integer(counts,"selected_parents",x.counts.selected_parents);Integer(counts,"selected_q4",x.counts.selected_q4);Integer(counts,"selected_t3",x.counts.selected_t3);
  Integer(counts,"selected_parts",x.counts.selected_parts);Integer(counts,"output_mains",x.counts.output_mains);
  Integer(counts,"output_references",x.counts.output_references);Integer(counts,"output_incidences",x.counts.output_incidences);
  Integer(counts,"topology_zero_neighbor_slots",x.counts.free_edge_slots);Child(d,"counts",counts);
  auto provenance=Object();Child(provenance,"canonical",Record(x.provenance.canonical_manifest));Child(provenance,"scope",Record(x.provenance.scope_report));
  Child(provenance,"source_member",Record(x.provenance.source_member));String(provenance,"auxiliary_sha256",x.provenance.auxiliary_sha256);String(provenance,"combine_sha256",x.provenance.combine_sha256);
  const auto& u=x.provenance.declared_units;auto units=Object();String(units,"length",u.length);String(units,"mass",u.mass);String(units,"time",u.time);
  Number(units,"length_to_m",u.length_to_m);Number(units,"mass_to_kg",u.mass_to_kg);Number(units,"time_to_s",u.time_to_s);Child(provenance,"declared_units",units);Child(d,"provenance",provenance);
  output::Value parts(rapidjson::kArrayType);
  for(const auto& p:x.parts) {
    auto row=Object();Integer(row,"pid",p.part_id);Integer(row,"mid",p.material_id);Integer(row,"sid",p.section_id);
    Boolean(row,"shell_section",p.shell_section);Boolean(row,"retained_shell_part",p.retained_shell_part);Boolean(row,"excluded_shell_part",p.excluded_shell_part);
    Integer(row,"shells",p.shells);Integer(row,"unassessed_solids",p.solids);Integer(row,"unassessed_beams",p.beams);
    output::Value child;child.CopyFrom(row,d.GetAllocator());parts.PushBack(child,d.GetAllocator());
  }
  d.AddMember("original_part_dispositions",parts,d.GetAllocator());
  Child(d,"failure",LocationDocument(x.failure));Integer(d,"native_neighbor_warning_count",x.topology_report.neighbor_warnings.count);
  Child(d,"first_native_warning",LocationDocument(x.first_warning));Child(d,"input_digest",DigestDocument(x.input_digest));
  if(x.output_complete)Child(d,"output_digest",DigestDocument(x.output_digest));
  else d.AddMember("output_digest",output::Value(),d.GetAllocator());
  auto timing=Object();Number(timing,"input_preparation_wall_s",x.input_wall_s);Number(timing,"tl_startup_wall_s",x.topology_wall_s);Number(timing,"output_digest_wall_s",x.digest_wall_s);
  String(timing,"scope","host assessment wall costs only;not matched simulation performance");Child(d,"timing",timing);Bound(d,cap);return d;
}
} // namespace crash::cases::vehicle_self_contact::native
