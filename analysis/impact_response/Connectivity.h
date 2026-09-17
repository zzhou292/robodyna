#pragma once

#include "Plasticity.h"
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace crash::analysis::impact_response {

struct ConnectivitySourceAuthority {
    std::string canonical_sha256;
    std::string member_sha256;
    std::string tire_policy;
};

struct ConnectivityParentEvidence {
    std::size_t parent_index = SIZE_MAX;
    std::size_t source_nodes = 0;
    std::size_t matched_constitutive_relations = 0;
    std::uint8_t combined_node_role_bits = 0;
    std::vector<std::uint64_t> transfer_component_labels;
    std::vector<std::uint64_t> incident_relations_by_kind;
    std::array<std::uint64_t, 4> incident_relations_by_role{};
    std::vector<std::vector<std::uint64_t>> incident_part_ids_by_kind;
};

struct ConnectivityEvidence {
    std::string schema;
    std::string scope;
    std::string archive_sha256;
    std::string member_sha256;
    std::string canonical_sha256;
    std::string tire_policy;
    std::vector<std::string> kind_codes;
    std::vector<std::uint64_t> relations_by_kind;
    std::size_t nodes = 0;
    std::size_t relations = 0;
    std::size_t ordered_slots = 0;
    std::size_t element_components = 0;
    std::size_t potential_transfer_components = 0;
    std::size_t yielded_source_nodes = 0;
    std::size_t incident_relations = 0;
    std::vector<std::uint64_t> incident_relations_by_kind;
    std::array<std::uint64_t, 4> incident_relations_by_role{};
    std::vector<std::uint64_t> yielded_transfer_component_labels;
    std::vector<ConnectivityParentEvidence> parents;
};

// Streaming/SAX interpretation of the existing bounded connectivity report.
// Only nodes and relations incident to yielded parents are retained. This is
// static weak potential-transfer evidence, never a reaction, rank, or loaded
// transfer proof.
ConnectivityEvidence AnalyzeConnectivity(const std::string& report_bytes,
    const PlasticityResult&, const ConnectivitySourceAuthority&,
    AnalysisLimits = {});

}  // namespace crash::analysis::impact_response
