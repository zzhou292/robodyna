#include "NativeOracle.h"
#include "output/ArtifactIO.h"
#include <climits>

extern "C" void native_tied_packing(const int*, const int*, const int*, const int*,
    const int*, const int*, const int*, int*, int*, int*, int*, int*, int*);
namespace crash::modelio::tied_shell::test {
NativePacking ObserveNative(int nodes, const std::vector<std::array<int,4>>& q4,
    const std::vector<std::array<int,3>>& t3, const std::vector<int>& slaves) {
    output::Require(nodes > 0 && nodes <= 1048576 && !slaves.empty() &&
                    q4.size()+t3.size() > 0 && q4.size()+t3.size() <= 524288 &&
                    slaves.size() <= std::size_t(nodes), "Invalid native packing packet extent");
    const auto check = [&](int node) {
        output::Require(node > 0 && node <= nodes, "Invalid native packing node index");
    };
    std::vector<int> quads, triangles;
    for (const auto& row : q4) {
        quads.push_back(0);
        for (auto node : row) { check(node); quads.push_back(node); }
    }
    for (const auto& row : t3) {
        triangles.push_back(0);
        for (auto node : row) { check(node); triangles.push_back(node); }
    }
    std::vector<bool> used(nodes+1, false);
    for (auto node : slaves) {
        check(node);
        output::Require(!used[node], "Native packet needs unique slave nodes");
        used[node] = true;
    }
    const int nq = q4.size(), nt = t3.size(), ns = slaves.size();
    NativePacking result;
    result.order.resize(nq+nt);
    result.rect.resize(4*(nq+nt));
    result.master_nodes.resize(nodes);
    result.slave_nodes.resize(ns);
    int master_count = 0, cleared = 0;
    native_tied_packing(&nodes, &nq, &nt, &ns, quads.data(), triangles.data(), slaves.data(),
        result.order.data(), result.rect.data(), result.master_nodes.data(), &master_count,
        result.slave_nodes.data(), &cleared);
    output::Require(master_count > 0 && master_count <= nodes, "Invalid native master count");
    result.master_nodes.resize(master_count);
    result.cleared = cleared == 1;
    for (auto& row : result.order) --row;
    return result;
}
} // namespace crash::modelio::tied_shell::test
