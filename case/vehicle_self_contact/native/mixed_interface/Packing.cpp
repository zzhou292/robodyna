#include "Internal.h"
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::mixed_interface::detail {
namespace {
bool Less(const Lookup& a, const Lookup& b) {
    return a.kind < b.kind || (a.kind == b.kind && a.element < b.element);
}
}
f::Input Packed::Input() const {
    f::Input in;
    in.profile = f::Profile::SingleSurfaceIlev1;
    in.physical = physical.Input(false);
    in.raw_faces = faces.data();
    in.raw_face_count = faces.size();
    in.positions = {positions.data(), std::uint32_t(node_ids.size()), 3, 1};
    in.coordinates = s::Coordinates::Native;
    in.source_generation = 1; // Immutable declared source, not a simulation clock.
    return in;
}
Packed Pack(const coated::Inputs& geometry, const std::vector<std::uint64_t>& parts,
        const std::vector<initial_surfaces::Face>& faces) {
    Packed out;
    out.physical = initial_surfaces::detail::Pack(geometry, parts);
    initial_surfaces::detail::CheckCapacity(geometry, out.physical, faces.size());
    std::vector<Lookup> lookup;
    lookup.reserve(geometry.solids.size() + geometry.shells.size());
    for (std::size_t i = 0; i < geometry.solids.size(); ++i)
        lookup.push_back({source::ParentKind::Solid, geometry.solids[i].source_id, std::uint32_t(i), std::uint32_t(i)});
    const auto add = [&](const auto& rows, const auto& map, auto kind) {
        for (std::size_t i = 0; i < rows.size(); ++i)
            lookup.push_back({kind, rows[i].element_id, std::uint32_t(i), map[i]});
    };
    add(out.physical.quads, out.physical.quad_to_physical, source::ParentKind::ShellQuad);
    add(out.physical.triangles, out.physical.triangle_to_physical, source::ParentKind::ShellTriangle);
    std::sort(lookup.begin(), lookup.end(), Less);
    for (std::size_t i = 1; i < lookup.size(); ++i)
        output::Require(Less(lookup[i-1], lookup[i]), "Duplicate typed physical source identity");
    out.faces.reserve(faces.size());
    out.raw_shell_to_physical.reserve(faces.size());
    for (const auto& face : faces) {
        const Lookup key{face.source.kind, face.source.element_id, 0, 0};
        const auto found = std::lower_bound(lookup.begin(), lookup.end(), key, Less);
        output::Require(found != lookup.end() && !Less(key, *found), "Mixed raw origin is outside complete physical source");
        source::Face raw;
        raw.source = {face.source.kind, face.source.element_id, face.source.part_id, found->row, face.source.solid_face};
        std::copy(face.nodes.begin(), face.nodes.end(), raw.nodes);
        raw.raw_role = face.raw_role;
        // Representative buffer ordinal is unused by this stage and never
        // published as an authentic CREATE/reader order.
        const auto external = initial_surfaces::detail::ExternalFace(raw, out.physical, geometry);
        output::Require(external.source.canonical_row == face.source.canonical_row &&
            external.source.source_line == face.source.source_line,
            "Mixed raw origin source location differs from retained context");
        out.faces.push_back(raw);
        out.raw_shell_to_physical.push_back(face.source.kind == source::ParentKind::Solid ? UINT32_MAX : found->physical);
    }
    out.node_ids.reserve(geometry.nodes.size());
    out.positions.reserve(3*geometry.nodes.size());
    for (const auto& node : geometry.nodes) {
        out.node_ids.push_back(node.source_id);
        out.positions.push_back(node.native_position.x);
        out.positions.push_back(node.native_position.y);
        out.positions.push_back(node.native_position.z);
    }
    if (lookup.capacity() > 2*lookup.size() || out.faces.capacity() > 2*faces.size() ||
        out.raw_shell_to_physical.capacity() > 2*faces.size() ||
        out.node_ids.capacity() > 2*geometry.nodes.size() || out.positions.capacity() > 6*geometry.nodes.size())
        Reject(Status::ResourceLimit, "Mixed source packing exceeds admitted capacity");
    return out;
}
s::Input SideInput(const Packed& packed, const f::Snapshot& classified) {
    s::Input input;
    input.profile = s::Profile::MixedSurface;
    input.topology = s::TopologyPolicy::NativeMixedSurface;
    input.node_source_ids = packed.node_ids.data();
    input.node_count = packed.node_ids.size();
    input.positions = packed.Input().positions;
    input.primary = classified.primary;
    input.primary_count = classified.primary_count;
    input.primary_identities = classified.identities;
    input.primary_identity_count = classified.primary_count;
    input.shell_primary_count = classified.shell_primary_count;
    input.raw_origins = classified.raw_origins;
    input.raw_origin_to_primary = classified.raw_to_primary;
    input.raw_origin_count = classified.raw_face_count;
    input.source_generation = classified.source_generation;
    return input;
}
}
