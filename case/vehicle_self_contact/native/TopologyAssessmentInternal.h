#pragma once
#include "TopologyAssessment.h"
#include "output/BoundedArrayIO.h"
#include <array>
namespace crash::cases::vehicle_self_contact::native::detail {
struct ParentOrigin {std::uint32_t canonical_row=0,source_line=0;std::uint64_t pid=0;};
struct Inputs {
  std::vector<std::uint64_t> node_ids;
  std::vector<double> positions;
  std::vector<s::PrimaryFace> primary;
  std::vector<ParentOrigin> origin;
  tlfea::contact::radioss_type25::UnitScale units;
  s::Input View() const;
};
// Value seams for small tests; these do not authenticate fabricated CanonicalData.
Forecast Plan(const source::CanonicalData&,const selection::Data&,Config,Limits);
Inputs PrepareInputs(const source::CanonicalData&,const selection::Data&,Config,Limits,Counts&);
Digest InputDigest(const Inputs&,const Config&,std::size_t metadata_cap);
Digest TopologyDigest(const s::Snapshot&,const std::string& input_digest,std::size_t metadata_cap);
Result EvaluateValues(const source::CanonicalData&,const selection::Data&,Config,Limits);
Location MapLocation(const Inputs&,std::size_t primary,std::size_t node);
} // namespace crash::cases::vehicle_self_contact::native::detail
