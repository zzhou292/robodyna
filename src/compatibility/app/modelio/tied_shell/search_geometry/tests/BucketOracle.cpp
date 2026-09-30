#include "BucketOracle.h"
#include "../Internal.h"
#include "lib_utest/qualification/tied_shell_search_bucket/native/Packet.h"
#include <algorithm>
#include <limits>

namespace crash::modelio::tied_shell::test {
NativeBucketResult NativeBucket(const TiedShellSearchGeometry& prepared, std::size_t pair_capacity) {
    const auto& d = prepared.data();
    const auto& declaration = prepared.packing().declaration().data();
    output::Require(!d.masters.empty() && d.masters.size() <= 524288 &&
        !d.working_positions.empty() && d.working_positions.size() <= 1048576 &&
        !d.secondary_working_nodes.empty() && d.secondary_working_nodes.size() <= 65536 &&
        !declaration.master_nodes.empty() && declaration.master_nodes.size() <= d.working_positions.size() &&
        pair_capacity && pair_capacity <= 8388608 && d.maximum_secondary_shell_thickness == 0,
        "Invalid original native bucket profile/counts");
    std::vector<std::array<int,4>> masters;
    std::vector<int> secondaries, master_nodes;
    std::vector<double> bounds_thickness, projection_thickness;
    masters.reserve(d.masters.size());
    bounds_thickness.reserve(d.masters.size());
    projection_thickness.reserve(d.masters.size());
    for (const auto& m : d.masters) {
        std::array<int,4> indices;
        for (unsigned i = 0; i < 4; ++i) indices[i] = int(m.working_nodes[i])+1;
        masters.push_back(indices);
        bounds_thickness.push_back(m.bounds_thickness);
        projection_thickness.push_back(m.projection_thickness);
    }
    secondaries.reserve(d.secondary_working_nodes.size());
    for (auto node : d.secondary_working_nodes) secondaries.push_back(int(node)+1);
    master_nodes.reserve(declaration.master_nodes.size());
    for (auto node : declaration.master_nodes) {
        const auto found = std::lower_bound(d.canonical_nodes.begin(), d.canonical_nodes.end(), node);
        output::Require(found != d.canonical_nodes.end() && *found == node, "Native MSR association missing");
        master_nodes.push_back(int(found-d.canonical_nodes.begin())+1);
    }
    NativeBucketResult next;
    next.selected.resize(secondaries.size());
    next.st.resize(secondaries.size());
    next.distance.resize(secondaries.size());
    next.pairs.resize(pair_capacity);
    int count = -1, status = -1;
    native_tied_bucket(d.working_positions.size(), masters.size(), secondaries.size(), master_nodes.size(),
        pair_capacity, d.working_positions.front().data(), masters.front().data(), secondaries.data(),
        master_nodes.data(), bounds_thickness.data(), projection_thickness.data(), next.selected.data(),
        next.st.front().data(), next.distance.data(), &count, next.pairs.front().data(),
        next.bounds.data(), next.cells.data(), &status);
    output::Require(status == 0 && count >= 0 && std::size_t(count) <= pair_capacity,
                    "Independent native bucket traversal rejected or exceeded its pair capacity");
    next.pairs.resize(count);
    return next;
}
} // namespace crash::modelio::tied_shell::test
