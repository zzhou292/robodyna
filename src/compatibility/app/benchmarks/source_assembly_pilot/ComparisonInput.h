#pragma once
#include "Comparison.h"
#include "TimeGrid.h"
#include "output/AcceptedReplaySourceAssembly.h"

namespace crash::benchmarks::assembly_pilot {
namespace rd=output::replay_detail;
using output::Document;using output::Value;using output::Require;
Document PhysicalConfiguration(const Value&);
std::string Encode(const Value&);
struct Run {
    rd::Bundle bundle;
    Document configuration,physical;
    std::string manifest_hash,placement,placed_mesh,original_wall;
    std::uint64_t multiple=1;
    void Open(const std::filesystem::path&);
    std::vector<std::uint64_t> Epochs() const;
    Document Frame(std::size_t index) const;
};
void MatchRuns(const Run& reference,const Run& candidate);
Value RunSummary(Document&,const Run&);
} // namespace crash::benchmarks::assembly_pilot
