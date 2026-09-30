#pragma once
#include <array>
#include <cstdint>
#include <vector>

namespace crash::modelio::tied_shell::test {
struct NativePacking {
    std::vector<int> order, rect, master_nodes, slave_nodes;
    bool cleared = false;
};
NativePacking ObserveNative(int nodes, const std::vector<std::array<int,4>>& q4,
    const std::vector<std::array<int,3>>& t3, const std::vector<int>& slaves);
} // namespace crash::modelio::tied_shell::test
