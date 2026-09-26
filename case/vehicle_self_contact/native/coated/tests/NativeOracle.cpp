#include "NativeOracle.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace crash::cases::vehicle_self_contact::native::coated::test {
extern "C" void rd_coated_reader(int, int, const int*, const double*, int*);
extern "C" void rd_coated_roles(int, int, int, const double*, const int*, const int*,
                                const int*, const int*, const int*, int*);
extern "C" void rd_coated_order(int, const int*, const int*, int*);
namespace {
void Require(bool condition, const char* message) {
    if (!condition) throw std::invalid_argument(message);
}
std::vector<double> Positions(const std::vector<Node>& nodes) {
    Require(!nodes.empty() && nodes.size() <= 1024, "Native coating oracle node cap");
    std::vector<double> result;
    result.reserve(3 * nodes.size());
    for (const auto& node : nodes) {
        const auto p = node.native_position;
        Require(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z),
                "Native coating oracle consumes finite coordinates");
        result.insert(result.end(), {p.x, p.y, p.z});
    }
    return result;
}
int ShellKind(const Shell& shell, std::size_t nodes) {
    Require(shell.primary.layout == n::ShellLayout::Quad4 ||
            shell.primary.layout == n::ShellLayout::Triangle3, "Unsupported native shell kind");
    for (const auto node : shell.primary.nodes)
        Require(node < nodes, "Native shell node out of range");
    if (shell.primary.layout == n::ShellLayout::Triangle3) {
        Require(shell.primary.nodes[2] == shell.primary.nodes[3], "Native T3 fourth slot differs");
        return 7;
    }
    return 3;
}
}
std::array<std::uint32_t, 8> NativeReader(ReaderKind kind,
        const std::array<std::uint32_t, 8>& declared, const std::vector<Node>& nodes) {
    Require(kind == ReaderKind::Hex8 || kind == ReaderKind::DeclaredPenta6,
            "Unknown native reader reference kind");
    const auto positions = Positions(nodes);
    const unsigned count = kind == ReaderKind::Hex8 ? 8 : 6;
    int input[8]{};
    for (unsigned i = 0; i < count; ++i) {
        Require(declared[i] < nodes.size(), "Native reader node out of range");
        input[i] = static_cast<int>(declared[i]);
    }
    int raw[8]{};
    rd_coated_reader(static_cast<int>(nodes.size()), static_cast<int>(count),
                      input, positions.data(), raw);
    std::array<std::uint32_t, 8> result{};
    for (unsigned i = 0; i < 8; ++i) {
        Require(raw[i] >= 0 && std::size_t(raw[i]) < nodes.size(), "Native reader output node out of range");
        result[i] = static_cast<std::uint32_t>(raw[i]);
    }
    return result;
}

std::vector<int> NativeRoles(const Inputs& input) {
    const auto positions = Positions(input.nodes);
    Require(input.solids.size() <= 256 && input.shells.size() <= 256,
            "Native classification oracle parent cap");
    std::vector<int> solids(11 * std::max<std::size_t>(1, input.solids.size()), 0);
    std::vector<std::vector<int>> incident(input.nodes.size());
    for (std::size_t i = 0; i < input.solids.size(); ++i) {
        const auto& source = input.solids[i];
        Require(source.phase == PacketPhase::ReaderBeforeInitia, "Native reference needs reader-phase slots");
        for (unsigned slot = 0; slot < 8; ++slot) {
            const auto node = source.nodes[slot];
            Require(node < input.nodes.size(), "Native solid node out of range");
            solids[11 * i + slot + 1] = static_cast<int>(node + 1);
            // The oracle explicitly supplies incoming solid order. Repeated
            // occurrences retain their full geometry packet; duplicate CSR
            // visits cannot change IN24's first matching solid.
            if (std::find(source.nodes.begin(), source.nodes.begin() + slot, node) == source.nodes.begin() + slot)
                incident[node].push_back(static_cast<int>(i + 1));
        }
    }
    std::vector<int> offsets(input.nodes.size() + 1, 0), incidences;
    for (std::size_t i = 0; i < incident.size(); ++i) {
        incidences.insert(incidences.end(), incident[i].begin(), incident[i].end());
        offsets[i + 1] = static_cast<int>(incidences.size());
    }
    if (incidences.empty()) incidences.push_back(0); // Unread canonical dummy for an empty native span.
    std::vector<int> shells(4 * std::max<std::size_t>(1, input.shells.size()), 0);
    std::vector<int> kinds(std::max<std::size_t>(1, input.shells.size()), 0);
    for (std::size_t i = 0; i < input.shells.size(); ++i) {
        const auto& source = input.shells[i];
        kinds[i] = ShellKind(source, input.nodes.size());
        for (unsigned slot = 0; slot < 4; ++slot)
            shells[4 * i + slot] = static_cast<int>(source.primary.nodes[slot] + 1);
    }
    std::vector<int> result(std::max<std::size_t>(1, input.shells.size()), 0);
    rd_coated_roles(static_cast<int>(input.nodes.size()), static_cast<int>(input.solids.size()),
        static_cast<int>(input.shells.size()), positions.data(), solids.data(), shells.data(),
        kinds.data(), offsets.data(), incidences.data(), result.data());
    result.resize(input.shells.size());
    for (const auto role : result)
        Require(role == 3 || role == 7 || role == 4 || role == 8 || role == -4 || role == -8,
                "Native classification produced an unselected role");
    return result;
}

std::vector<std::uint32_t> NativeOrder(const Inputs& input, const std::vector<int>& roles) {
    Require(!input.nodes.empty() && input.nodes.size() <= 1024 && input.shells.size() <= 256 &&
            roles.size() == input.shells.size(), "Native surface order reference extent");
    std::vector<int> nodes, selected_roles;
    std::vector<std::uint32_t> physical;
    for (std::size_t i = 0; i < input.shells.size(); ++i) {
        const auto& shell = input.shells[i];
        if (!shell.contact_selected) continue;
        (void)ShellKind(shell, input.nodes.size());
        const int role = roles[i];
        Require(role == 3 || role == 7 || role == 4 || role == 8 || role == -4 || role == -8,
                "Native surface order reference role");
        for (const auto node : shell.primary.nodes) nodes.push_back(static_cast<int>(node + 1));
        selected_roles.push_back(role);
        physical.push_back(static_cast<std::uint32_t>(i));
    }
    if (physical.empty()) return {};
    std::vector<int> permutation(physical.size(), -1);
    rd_coated_order(static_cast<int>(physical.size()), nodes.data(), selected_roles.data(), permutation.data());
    std::vector<unsigned char> seen(physical.size(), 0);
    std::vector<std::uint32_t> result;
    result.reserve(physical.size());
    for (const auto ordinal : permutation) {
        Require(ordinal >= 0 && std::size_t(ordinal) < physical.size() && !seen[ordinal],
                "Native surface ordering is not a permutation");
        seen[ordinal] = 1;
        result.push_back(physical[ordinal]);
    }
    return result;
}
}
