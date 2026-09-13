#include "Connectivity.h"

#include "output/BoundedArrayIO.h"
#include "chrono_thirdparty/rapidjson/memorystream.h"
#include "chrono_thirdparty/rapidjson/reader.h"
#include <algorithm>
#include <map>
#include <set>

namespace crash::analysis::impact_response {
namespace {

using output::Require;

struct ParentScratch {
    std::set<std::uint64_t> transfer_labels;
    std::vector<std::set<std::uint64_t>> part_ids_by_kind;
};

class ConnectivityHandler
    : public rapidjson::BaseReaderHandler<rapidjson::UTF8<>, ConnectivityHandler> {
  public:
    ConnectivityHandler(const PlasticityResult& plasticity,
        const ConnectivitySourceAuthority& authority, AnalysisLimits limits)
        : plasticity_(plasticity), authority_(authority), limits_(limits) {
        for (std::size_t parent = 0; parent < plasticity.parents.size(); ++parent) {
            if (!plasticity.parents[parent].ever_positive) continue;
            ConnectivityParentEvidence evidence;
            evidence.parent_index = parent;
            output_.parents.push_back(std::move(evidence));
            scratch_.emplace_back();
            const auto& source = plasticity.catalog[parent];
            std::set<std::uint64_t> unique(
                source.source_nodes.begin(),
                source.source_nodes.begin() + source.node_count());
            expected_source_nodes_.push_back(unique.size());
            for (const auto node : unique)
                wanted_nodes_[node].push_back(output_.parents.size() - 1);
        }
    }

    bool Null() { return ScalarRejected("Unexpected null in connectivity report"); }
    bool Bool(bool) { return ScalarIgnoredOrRejected(); }
    bool Int(int value) {
        return value < 0 ? ScalarRejected("Negative connectivity integer") :
            Unsigned(static_cast<std::uint64_t>(value));
    }
    bool Uint(unsigned value) { return Unsigned(value); }
    bool Int64(std::int64_t value) {
        return value < 0 ? ScalarRejected("Negative connectivity integer") :
            Unsigned(static_cast<std::uint64_t>(value));
    }
    bool Uint64(std::uint64_t value) { return Unsigned(value); }
    bool Double(double) {
        return ScalarRejected("Unexpected real value in connectivity report");
    }
    bool RawNumber(const char*, rapidjson::SizeType, bool) {
        return ScalarRejected("Unexpected raw number in connectivity report");
    }

    bool String(const char* text, rapidjson::SizeType length, bool) {
        if (stack_.empty()) return Fail("Connectivity string has no container");
        const std::string value(text, length);
        switch (stack_.back().kind) {
            case Kind::Root:
                if (stack_.back().key == "schema") {
                    output_.schema = value;
                    seen_schema_ = true;
                } else if (stack_.back().key == "scope") {
                    output_.scope = value;
                    seen_scope_ = true;
                } else if (stack_.back().key == "archive_sha256") {
                    output_.archive_sha256 = value;
                } else if (stack_.back().key == "member_sha256") {
                    output_.member_sha256 = value;
                } else if (stack_.back().key == "canonical_sha256") {
                    output_.canonical_sha256 = value;
                } else if (stack_.back().key == "tire_policy") {
                    output_.tire_policy = value;
                }
                return true;
            case Kind::Kinds:
                output_.kind_codes.push_back(value);
                return true;
            case Kind::IgnoreArray:
            case Kind::IgnoreObject:
                return true;
            default:
                return Fail("Unexpected connectivity string");
        }
    }

    bool Key(const char* text, rapidjson::SizeType length, bool) {
        if (stack_.empty()) return Fail("Connectivity key has no object");
        const auto kind = stack_.back().kind;
        if (kind != Kind::Root && kind != Kind::Counts &&
            kind != Kind::IgnoreObject)
            return Fail("Connectivity key appears inside an array");
        stack_.back().key.assign(text, length);
        return true;
    }

    bool StartObject() {
        if (stack_.empty()) {
            if (root_started_) return Fail("Connectivity report has multiple roots");
            root_started_ = true;
            stack_.push_back({Kind::Root, {}});
            return true;
        }
        const auto parent = stack_.back().kind;
        if (parent == Kind::IgnoreArray || parent == Kind::IgnoreObject) {
            stack_.push_back({Kind::IgnoreObject, {}});
            return true;
        }
        if (parent == Kind::Root && stack_.back().key == "counts") {
            seen_counts_ = true;
            stack_.push_back({Kind::Counts, {}});
            return true;
        }
        if (parent == Kind::Root) {
            stack_.push_back({Kind::IgnoreObject, {}});
            return true;
        }
        return Fail("Unexpected nested connectivity object");
    }

    bool EndObject(rapidjson::SizeType) {
        if (stack_.empty()) return Fail("Connectivity object stack underflow");
        const auto kind = stack_.back().kind;
        if (kind != Kind::Root && kind != Kind::Counts &&
            kind != Kind::IgnoreObject)
            return Fail("Connectivity object ended in the wrong container");
        if (kind == Kind::Root) root_complete_ = true;
        stack_.pop_back();
        return true;
    }

    bool StartArray() {
        if (stack_.empty()) return Fail("Connectivity array has no container");
        const auto parent = stack_.back().kind;
        if (parent == Kind::IgnoreArray || parent == Kind::IgnoreObject) {
            stack_.push_back({Kind::IgnoreArray, {}});
            return true;
        }
        if (parent == Kind::Root) {
            const auto& key = stack_.back().key;
            if (key == "kind_codes") {
                seen_kinds_ = true;
                stack_.push_back({Kind::Kinds, {}});
            } else if (key == "nodes") {
                seen_nodes_ = true;
                stack_.push_back({Kind::Nodes, {}});
            } else if (key == "relations") {
                seen_relations_ = true;
                stack_.push_back({Kind::Relations, {}});
            } else {
                stack_.push_back({Kind::IgnoreArray, {}});
            }
            return true;
        }
        if (parent == Kind::Counts && stack_.back().key == "by_kind") {
            seen_by_kind_ = true;
            stack_.push_back({Kind::ByKind, {}});
            return true;
        }
        if (parent == Kind::Nodes) {
            node_values_.clear();
            stack_.push_back({Kind::NodeRow, {}});
            return true;
        }
        if (parent == Kind::Relations) {
            relation_header_.clear();
            relation_slots_.clear();
            relation_tail_.clear();
            relation_slots_done_ = false;
            stack_.push_back({Kind::RelationRow, {}});
            return true;
        }
        if (parent == Kind::RelationRow && relation_header_.size() == 5 &&
            !relation_slots_done_) {
            stack_.push_back({Kind::Slots, {}});
            return true;
        }
        return Fail("Unexpected nested connectivity array");
    }

    bool EndArray(rapidjson::SizeType) {
        if (stack_.empty()) return Fail("Connectivity array stack underflow");
        const auto kind = stack_.back().kind;
        if (kind == Kind::NodeRow) {
            if (!ProcessNode()) return false;
        } else if (kind == Kind::Slots) {
            if (relation_slots_.empty())
                return Fail("Connectivity relation has no support slots");
            relation_slots_done_ = true;
        } else if (kind == Kind::RelationRow) {
            if (!ProcessRelation()) return false;
        } else if (kind != Kind::Kinds && kind != Kind::ByKind &&
                   kind != Kind::Nodes && kind != Kind::Relations &&
                   kind != Kind::IgnoreArray) {
            return Fail("Connectivity array ended in the wrong container");
        }
        stack_.pop_back();
        return true;
    }

    ConnectivityEvidence Finish() {
        Require(error_.empty() && root_started_ && root_complete_ && stack_.empty() &&
                seen_schema_ && seen_scope_ && seen_counts_ && seen_kinds_ &&
                seen_by_kind_ && seen_nodes_ && seen_relations_,
            "Connectivity report is incomplete");
        Require(output_.schema == "robo_dyna.original_physical_connectivity.v2" &&
                output_.scope ==
                    "Static weak incidence/potential transfer; no current activity, DOF rank or complete load-path admission" &&
                output_.canonical_sha256 == authority_.canonical_sha256 &&
                output_.member_sha256 == authority_.member_sha256 &&
                output_.tire_policy == authority_.tire_policy,
            "Connectivity report source/scope differs from run authority");
        output::arrays::CheckHash(output_.archive_sha256);
        output::arrays::CheckHash(output_.canonical_sha256);
        output::arrays::CheckHash(output_.member_sha256);
        if (observed_by_kind_.empty())
            observed_by_kind_.assign(output_.kind_codes.size(), 0);
        Require(output_.nodes && output_.nodes <= 524288 &&
                output_.relations <= 524288 &&
                output_.ordered_slots <= 4u * 1024u * 1024u &&
                !output_.kind_codes.empty() && output_.kind_codes.size() <= 64 &&
                node_index_ == output_.nodes && relation_index_ == output_.relations &&
                output_.kind_codes.size() == output_.relations_by_kind.size() &&
                observed_by_kind_ == output_.relations_by_kind,
            "Connectivity report counts differ from streamed rows");
        Require(found_wanted_nodes_.size() == wanted_nodes_.size(),
            "Connectivity report omits a yielded parent source node");
        if (output_.incident_relations_by_kind.empty())
            output_.incident_relations_by_kind.assign(
                output_.kind_codes.size(), 0);

        std::set<std::uint64_t> all_labels;
        for (std::size_t i = 0; i < output_.parents.size(); ++i) {
            auto& parent = output_.parents[i];
            Require(parent.source_nodes == expected_source_nodes_[i] &&
                    parent.matched_constitutive_relations == 1,
                "Connectivity report does not bind a yielded parent exactly once");
            parent.transfer_component_labels.assign(
                scratch_[i].transfer_labels.begin(), scratch_[i].transfer_labels.end());
            all_labels.insert(parent.transfer_component_labels.begin(),
                parent.transfer_component_labels.end());
            parent.incident_part_ids_by_kind.resize(output_.kind_codes.size());
            for (std::size_t kind = 0; kind < output_.kind_codes.size(); ++kind)
                parent.incident_part_ids_by_kind[kind].assign(
                    scratch_[i].part_ids_by_kind[kind].begin(),
                    scratch_[i].part_ids_by_kind[kind].end());
        }
        output_.yielded_transfer_component_labels.assign(
            all_labels.begin(), all_labels.end());
        output_.yielded_source_nodes = found_wanted_nodes_.size();
        return std::move(output_);
    }

    const std::string& error() const noexcept { return error_; }

  private:
    enum class Kind {
        Root,
        Counts,
        IgnoreObject,
        IgnoreArray,
        Kinds,
        ByKind,
        Nodes,
        NodeRow,
        Relations,
        RelationRow,
        Slots
    };
    struct Frame {
        Kind kind;
        std::string key;
    };

    bool Fail(const char* message) {
        if (error_.empty()) error_ = message;
        return false;
    }
    bool ScalarRejected(const char* message) {
        if (!stack_.empty() && (stack_.back().kind == Kind::IgnoreArray ||
                stack_.back().kind == Kind::IgnoreObject))
            return true;
        return Fail(message);
    }
    bool ScalarIgnoredOrRejected() {
        if (!stack_.empty() && (stack_.back().kind == Kind::IgnoreArray ||
                stack_.back().kind == Kind::IgnoreObject ||
                stack_.back().kind == Kind::Root))
            return true;
        return Fail("Unexpected connectivity boolean");
    }
    bool Unsigned(std::uint64_t value) {
        if (stack_.empty()) return Fail("Connectivity integer has no container");
        switch (stack_.back().kind) {
            case Kind::Root:
            case Kind::IgnoreArray:
            case Kind::IgnoreObject:
                return true;
            case Kind::Counts: {
                const auto& key = stack_.back().key;
                if (key == "nodes") output_.nodes = CheckedSize(value);
                else if (key == "relations") output_.relations = CheckedSize(value);
                else if (key == "ordered_slots") output_.ordered_slots = CheckedSize(value);
                else if (key == "element_components") output_.element_components = CheckedSize(value);
                else if (key == "potential_transfer_components")
                    output_.potential_transfer_components = CheckedSize(value);
                return error_.empty();
            }
            case Kind::ByKind:
                output_.relations_by_kind.push_back(value);
                return true;
            case Kind::NodeRow:
                node_values_.push_back(value);
                return true;
            case Kind::RelationRow:
                if (!relation_slots_done_) relation_header_.push_back(value);
                else relation_tail_.push_back(value);
                return true;
            case Kind::Slots:
                relation_slots_.push_back(CheckedSize(value));
                return error_.empty();
            default:
                return Fail("Unexpected connectivity integer");
        }
    }
    std::size_t CheckedSize(std::uint64_t value) {
        if (value > SIZE_MAX) {
            Fail("Connectivity integer exceeds host size");
            return 0;
        }
        return static_cast<std::size_t>(value);
    }

    bool ProcessNode() {
        if (node_values_.size() != 4 || node_index_ >= output_.nodes)
            return Fail("Connectivity node row has the wrong shape or count");
        if (!node_values_[0] || !node_values_[1] || !node_values_[2] ||
            node_values_[3] > 15 || node_index_ > UINT32_MAX)
            return Fail("Connectivity node identity/role is invalid");
        const auto source_node = node_values_[0];
        const auto found = wanted_nodes_.find(source_node);
        if (found != wanted_nodes_.end()) {
            if (!found_wanted_nodes_.insert(source_node).second)
                return Fail("Connectivity report duplicates a yielded source node");
            const auto domain = static_cast<std::uint32_t>(node_index_);
            domain_source_nodes_[domain] = source_node;
            domain_parents_[domain] = found->second;
            for (const auto parent : found->second) {
                auto& evidence = output_.parents[parent];
                ++evidence.source_nodes;
                evidence.combined_node_role_bits |=
                    static_cast<std::uint8_t>(node_values_[3]);
                scratch_[parent].transfer_labels.insert(node_values_[2]);
            }
        }
        ++node_index_;
        return true;
    }

    bool ProcessRelation() {
        if (relation_header_.size() != 5 || !relation_slots_done_ ||
            relation_tail_.size() > 1 || relation_index_ >= output_.relations)
            return Fail("Connectivity relation row has the wrong shape or count");
        const auto kind = CheckedSize(relation_header_[0]);
        const auto role = CheckedSize(relation_header_[1]);
        if (!error_.empty() || kind >= output_.kind_codes.size() || role >= 4)
            return Fail("Connectivity relation kind/role is out of range");
        if (observed_by_kind_.empty())
            observed_by_kind_.assign(output_.kind_codes.size(), 0);
        ++observed_by_kind_[kind];

        std::vector<std::size_t> affected;
        for (const auto slot : relation_slots_) {
            if (slot >= output_.nodes || slot > UINT32_MAX)
                return Fail("Connectivity relation references an absent node");
            const auto found = domain_parents_.find(static_cast<std::uint32_t>(slot));
            if (found != domain_parents_.end())
                affected.insert(affected.end(), found->second.begin(), found->second.end());
        }
        std::sort(affected.begin(), affected.end());
        affected.erase(std::unique(affected.begin(), affected.end()), affected.end());
        if (!affected.empty()) {
            if (output_.incident_relations >= limits_.incident_relations)
                return Fail("Yielded-parent incident relation cap is exhausted");
            ++output_.incident_relations;
            if (output_.incident_relations_by_kind.empty())
                output_.incident_relations_by_kind.assign(output_.kind_codes.size(), 0);
            ++output_.incident_relations_by_kind[kind];
            ++output_.incident_relations_by_role[role];
            for (const auto parent : affected) {
                auto& evidence = output_.parents[parent];
                if (evidence.incident_relations_by_kind.empty()) {
                    evidence.incident_relations_by_kind.assign(output_.kind_codes.size(), 0);
                    scratch_[parent].part_ids_by_kind.resize(output_.kind_codes.size());
                }
                ++evidence.incident_relations_by_kind[kind];
                ++evidence.incident_relations_by_role[role];
                if (relation_header_[3])
                    scratch_[parent].part_ids_by_kind[kind].insert(relation_header_[3]);
            }
        }

        if (role == 0 && kind <= 2) {
            const auto source_id = relation_header_[2];
            const auto part_id = relation_header_[3];
            for (std::size_t evidence_index = 0;
                 evidence_index < output_.parents.size(); ++evidence_index) {
                const auto parent_index = output_.parents[evidence_index].parent_index;
                const auto& source = plasticity_.catalog[parent_index];
                if (source.source_element != source_id ||
                    source.source_part != part_id)
                    continue;
                if (relation_slots_.size() != source.node_count())
                    return Fail("Yielded shell connectivity width differs from source mapping");
                for (std::size_t slot = 0; slot < relation_slots_.size(); ++slot) {
                    const auto found = domain_source_nodes_.find(
                        static_cast<std::uint32_t>(relation_slots_[slot]));
                    if (found == domain_source_nodes_.end() ||
                        found->second != source.source_nodes[slot])
                        return Fail("Yielded shell connectivity order differs from source mapping");
                }
                ++output_.parents[evidence_index].matched_constitutive_relations;
            }
        }
        ++relation_index_;
        return error_.empty();
    }

    const PlasticityResult& plasticity_;
    const ConnectivitySourceAuthority& authority_;
    AnalysisLimits limits_;
    ConnectivityEvidence output_;
    std::vector<ParentScratch> scratch_;
    std::vector<std::size_t> expected_source_nodes_;
    std::map<std::uint64_t, std::vector<std::size_t>> wanted_nodes_;
    std::set<std::uint64_t> found_wanted_nodes_;
    std::map<std::uint32_t, std::vector<std::size_t>> domain_parents_;
    std::map<std::uint32_t, std::uint64_t> domain_source_nodes_;
    std::vector<Frame> stack_;
    std::vector<std::uint64_t> node_values_, relation_header_, relation_tail_;
    std::vector<std::size_t> relation_slots_;
    std::vector<std::uint64_t> observed_by_kind_;
    std::size_t node_index_ = 0, relation_index_ = 0;
    bool relation_slots_done_ = false;
    bool root_started_ = false, root_complete_ = false;
    bool seen_schema_ = false, seen_scope_ = false, seen_counts_ = false;
    bool seen_kinds_ = false, seen_by_kind_ = false;
    bool seen_nodes_ = false, seen_relations_ = false;
    std::string error_;
};

}  // namespace

ConnectivityEvidence AnalyzeConnectivity(const std::string& report_bytes,
    const PlasticityResult& plasticity,
    const ConnectivitySourceAuthority& authority, AnalysisLimits limits) {
    Require(!report_bytes.empty() && report_bytes.size() <= 32u << 20 &&
            plasticity.catalog.size() == plasticity.parents.size() &&
            !authority.canonical_sha256.empty() &&
            !authority.member_sha256.empty() && !authority.tire_policy.empty(),
        "Connectivity analysis input or cap is invalid");
    output::arrays::CheckHash(authority.canonical_sha256);
    output::arrays::CheckHash(authority.member_sha256);
    ConnectivityHandler handler(plasticity, authority, limits);
    rapidjson::MemoryStream stream(report_bytes.data(), report_bytes.size());
    rapidjson::Reader reader;
    const bool parsed = reader.Parse<rapidjson::kParseIterativeFlag>(stream, handler);
    if (!parsed && !handler.error().empty())
        throw std::runtime_error(handler.error());
    Require(parsed, "Malformed connectivity JSON");
    return handler.Finish();
}

}  // namespace crash::analysis::impact_response
