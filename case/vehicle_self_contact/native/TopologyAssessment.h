#pragma once
#include "modelio/self_contact/OriginalSelection.h"
#include "lib_src/collision/RadiossType25FixedMainStartup.h"
#include <vector>
namespace crash::cases::vehicle_self_contact::native {
namespace selection=modelio::self_contact;
namespace source=output::full_shell::source;
namespace s=tlfea::contact::radioss_type25::startup;
enum class SelectionPolicy { CanonicalRetainedOriginalShells };
enum class Order { CanonicalRecordOrder };
enum class WorkingUnits { SourceDeclaredUnits };
struct Config {
  SelectionPolicy selection=SelectionPolicy::CanonicalRetainedOriginalShells;
  Order order=Order::CanonicalRecordOrder;
  WorkingUnits working_units=WorkingUnits::SourceDeclaredUnits;
};
struct Limits {
  std::size_t host_bytes=std::size_t{2}<<30,metadata_bytes=1u<<20;
  std::size_t nodes=524288,parents=524288;
  s::Limits topology;
};
struct Forecast {
  bool admitted=false;
  std::size_t canonical_source_reservation=0,selection_source_reservation=0;
  std::size_t input_owned_bytes=0,input_decode_peak_bytes=0,digest_scratch_bytes=0,result_owned_bytes=0;
  std::size_t peak_host_bytes=0;
  s::Forecast topology;
  std::size_t main_size=0,reference_size=0,primary_size=0;
};
struct Counts {
  selection::Counts original;
  std::size_t canonical_nodes=0,canonical_shells=0;
  std::size_t selected_parents=0,selected_q4=0,selected_t3=0,selected_parts=0;
  std::size_t output_mains=0,output_references=0,output_incidences=0,free_edge_slots=0;
};
struct FieldDigest {
  std::string name,type,sha256;
  std::size_t rows=0,columns=0,chunks=0;
};
struct Digest {
  std::string sha256;
  std::vector<FieldDigest> fields;
};
struct Location {
  std::size_t canonical_row=SIZE_MAX,expanded_main=SIZE_MAX,edge=SIZE_MAX;
  std::uint64_t source_eid=0,source_pid=0,source_node_id=0;
  std::uint32_t source_line=0;
};
struct Provenance {
  output::full_shell::RecordFile canonical_manifest,scope_report,source_member;
  std::string auxiliary_sha256,combine_sha256;
  source::Units declared_units;
};
struct Result {
  Config config;
  Forecast forecast;
  Counts counts;
  Provenance provenance;
  // Keep every original disposition visible, including unassessed solids/non-shell parts.
  std::vector<selection::PartDisposition> parts;
  s::Report topology_report;
  Location failure,first_warning;
  bool output_complete=false;
  Digest input_digest,output_digest;
  double input_wall_s=0,topology_wall_s=0,digest_wall_s=0;
};
// No physical source, topology borrow, owner, cache or clock is returned.
// OriginalSelection keeps the authenticated CanonicalSource backing alive.
Forecast Preflight(const selection::OriginalSelection&,Config={},Limits={});
Result Assess(const selection::OriginalSelection&,Config={},Limits={});
output::Document ForecastDocument(const Forecast&,std::size_t byte_cap=1u<<20);
output::Document ResultDocument(const Result&,std::size_t byte_cap=1u<<20);
} // namespace crash::cases::vehicle_self_contact::native
