#pragma once
#include "Internal.h"
namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail {
// Optional mixed-face query index. Legacy selected-shell preparation does not
// allocate it or change its forecast. Rows are representative physical indices;
// native first/last ownership still requires ResolveSupportTies' certificate.
// Inputs and Packed must remain alive and unchanged for the index lifetime.
// Pointer/count binding is source association, not mutation detection.
struct SupportQueryIndex {
    std::vector<std::uint32_t> quad_offsets, quad_rows;
    const coated::Shell* source_shells = nullptr;
    std::size_t shell_count = 0, node_count = 0;
};
std::size_t SupportQueryBytes(std::size_t nodes, std::size_t shells);
SupportQueryIndex PrepareSupportQueries(const coated::Inputs&, const Packed&,
    std::size_t byte_cap = std::size_t{64} << 20);
// Empty winners means native NELTG/NEL are absent. For a triangular face,
// nonempty T3 wins in I25GAPM regardless of a thicker containing Q4.
SupportSelection QuerySupport(const coated::Inputs&, const Packed&, const SupportQueryIndex&,
    const s::Main& post_sides_face, bool grouping_context);
}
