#include "NativeOracle.h"
#include <cmath>
#include <algorithm>
#include <mutex>
namespace crash::cases::vehicle_self_contact::native::main_coefficients::test {
namespace {
std::mutex lock;
extern "C" void rd_main_support(const int*, const int*, const int*, const double*, const double*,
    const double*, const double*, const int*, int*, int*);
extern "C" void rd_main_order(const int*, const std::uint32_t*, int*);
}
NativeResult NativeSupport(const std::vector<NativeCandidate>& input, const std::array<unsigned, 4>& main) {
    output::Require(!input.empty() && input.size() <= 64, "Native support oracle row cap");
    std::vector<int> qnodes, tnodes;
    std::vector<double> qh, qe, th, te;
    std::vector<std::size_t> qids, tids;
    unsigned nodes = 0;
    for (std::size_t i = 0; i < input.size(); ++i) {
        const auto& row = input[i];
        output::Require(std::isfinite(row.thickness) && row.thickness > 0 &&
            std::isfinite(row.young) && row.young > 0, "Native support oracle operand domain");
        const bool tri = row.layout == n::ShellLayout::Triangle3;
        output::Require(tri || row.layout == n::ShellLayout::Quad4, "Native support oracle layout");
        auto& ids = tri ? tnodes : qnodes;
        for (unsigned k = 0; k < (tri ? 3u : 4u); ++k) {
            output::Require(row.nodes[k] < 256, "Native support oracle node cap");
            ids.push_back(int(row.nodes[k]) + 1);
            nodes = std::max(nodes, row.nodes[k] + 1);
        }
        (tri ? th : qh).push_back(row.thickness);
        (tri ? te : qe).push_back(row.young);
        (tri ? tids : qids).push_back(i);
    }
    int face[4];
    for (unsigned k = 0; k < 4; ++k) {
        output::Require(main[k] < nodes, "Native support oracle pivot outside node domain");
        face[k] = int(main[k]) + 1;
    }
    const int counts[]{int(nodes), int(qh.size()), int(th.size())};
    // Empty families still have a valid dummy address; native counts keep them unread.
    qnodes.resize(std::max<std::size_t>(1, qnodes.size()));
    tnodes.resize(std::max<std::size_t>(1, tnodes.size()));
    for (auto* values : {&qh, &qe, &th, &te}) values->resize(std::max<std::size_t>(1, values->size()));
    int chosen[2]{}, incidence[512]{};
    std::lock_guard<std::mutex> serial(lock);
    rd_main_support(counts, qnodes.data(), tnodes.data(), qh.data(), qe.data(), th.data(), te.data(),
        face, chosen, incidence);
    output::Require(chosen[0] >= 0 && std::size_t(chosen[0]) <= qids.size() && chosen[1] >= 0 &&
        std::size_t(chosen[1]) <= tids.size(), "Native support oracle result extent");
    NativeResult out;
    if (chosen[0]) out.q4 = qids[chosen[0]-1];
    if (chosen[1]) out.t3 = tids[chosen[1]-1];
    out.selected = out.t3 != SIZE_MAX ? out.t3 : out.q4;
    return out;
}
std::vector<unsigned> NativeOrder(const std::vector<std::array<std::uint32_t, 8>>& keys) {
    output::Require(!keys.empty() && keys.size() <= 64, "Native sort-key oracle cap");
    const int count = int(keys.size());
    std::vector<int> permutation(keys.size());
    std::lock_guard<std::mutex> serial(lock);
    rd_main_order(&count, keys[0].data(), permutation.data());
    std::vector<unsigned> out;
    for (const auto row : permutation) {
        output::Require(row > 0 && row <= count, "Native order result extent");
        out.push_back(unsigned(row-1));
    }
    return out;
}
}
