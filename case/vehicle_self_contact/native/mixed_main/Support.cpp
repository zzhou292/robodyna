#include "Internal.h"
#include "CoefficientValues.h"
#include <algorithm>
#include <cmath>
namespace crash::cases::vehicle_self_contact::native::post_gapm::detail {
namespace {
void Numeric(n::CoefficientStatus status, std::size_t primary) {
    if (status != n::CoefficientStatus::Ok)
        Reject(status == n::CoefficientStatus::NonfiniteResult ? Status::NonfiniteResult : Status::InvalidInput,
            "Original main numerical leaf rejected consumed source operands", primary);
}
n::NativeSolidMainCoefficientInput SolidValues(const Mixed& source, std::size_t row,
    double area, double volume, bool internal) {
    const auto& input = source.initial().geometry();
    const auto& solid = input.solids.at(row);
    const auto& context = source.initial().context();
    const auto query = context.material_slots(solid.part_id);
    Require(query.status == nodal_correction::MaterialSlotStatus::Ready && query.values,
        "Post-UPDMAT main support material is unavailable");
    const auto& material = *query.values;
    Require(tl::math::SameScalarBits(material.units.length_m, input.units.length_m) &&
        tl::math::SameScalarBits(material.units.mass_kg, input.units.mass_kg) &&
        tl::math::SameScalarBits(material.units.time_s, input.units.time_s), "Support geometry/material units differ");
    const auto& parts = context.part_controls();
    const auto at = std::lower_bound(parts.begin(), parts.end(), solid.part_id,
        [](const auto& part, auto id) { return part.part_id < id; });
    Require(at != parts.end() && at->part_id == solid.part_id && at->section_id == material.section_id &&
        at->material_id == material.material_id, "Support effective property association differs");
    n::NativeSolidMainCoefficientInput value;
    value.face = internal ? n::MainFaceKind::Internal : n::MainFaceKind::OrdinaryExterior;
    value.layout = n::SolidLayout::EightSlot;
    value.incompressibility_control = at->effective_control ? 1 : 0;
    value.scale = 1.; value.fill = 1.; // Existing closed direct-source no-FILL certificate.
    value.area = area; value.volume = volume; value.bulk = material.pm32; value.controlled_bulk = material.pm107;
    return value;
}
n::NativeExteriorMainGeometryInput Packet(const coated::Inputs& input, const s::Main& main, std::size_t row) {
    n::NativeExteriorMainGeometryInput packet;
    packet.layout = main.nodes[2] == main.nodes[3] ? n::ShellLayout::Triangle3 : n::ShellLayout::Quad4;
    for (unsigned k=0; k<4; ++k) packet.face[k] = input.nodes.at(main.nodes[k]).native_position;
    const auto& solid = input.solids.at(row);
    Require(solid.phase == coated::PacketPhase::ReaderBeforeInitia, "Main support is not a reader-phase raw8 packet");
    for (unsigned k=0; k<8; ++k) packet.solid_raw[k] = input.nodes.at(solid.nodes[k]).native_position;
    return packet;
}
}
void Resolve(const Mixed& source, const old::Packed& packed, bool grouping, Values& values) {
    const auto& input = source.initial().geometry();
    const auto& side = source.sides();
    const auto shells = old::PrepareSupportQueries(input, packed);
    const auto& flags = source.initial().emitted_solid_flags();
    const auto solids = old::PrepareSolidSupportQueries(input, {flags.data(), flags.size()});
    const auto p = side.primary_count, g = side.main_count;
    values.corners.resize(p); values.before_shell.resize(p); values.final_support.resize(g);
    values.coefficients.resize(g); values.owners.resize(p); values.geometry.resize(p);
    values.counts.primaries = p; values.counts.expanded = g;
    for (std::size_t i=0; i<p; ++i) {
        auto main = side.mains[i];
        auto& geometry = values.geometry[i];
        auto& corner = values.corners[i];
        if (main.nodes[2] == main.nodes[3]) corner.source_corner[3] = 2;
        const auto solid = old::QuerySolidSupport(input, solids, main);
        if (solid.state == old::SolidSupportState::NeedsNativeReaderOrder || solid.matches.size() > 2)
            Reject(Status::NeedsNativeReaderOrder, "More than two solid supports need authentic native first-two order", i);
        ++values.counts.solid_matches[solid.matches.size()];
        auto& before = values.before_shell[i];
        before.unique_match_count = std::uint32_t(solid.matches.size());
        MainPacket coefficient;
        if (solid.state == old::SolidSupportState::Resolved) {
            const bool internal = solid.second != SIZE_MAX;
            before.first_solid_source_id = input.solids.at(solid.first).source_id;
            before.second_solid_source_id = internal ? input.solids.at(solid.second).source_id : 0;
            values.counts.pre_shell_internal += internal;
            const auto packet = Packet(input, main, solid.first);
            if (internal) {
                n::NativeInternalMainGeometryResult raw;
                Numeric(n::EvaluateNativeInternalMainGeometry(packet, &raw), i);
                geometry.area = raw.area; geometry.first_volume = raw.signed_volume;
            } else {
                Require(solid.exterior_orientation, "Effective exterior support has no original orientation phase");
                n::NativeExteriorMainGeometryResult raw;
                Numeric(n::EvaluateNativeExteriorMainGeometry(packet, &raw), i);
                geometry.area = raw.area; geometry.first_volume = raw.signed_volume;
                geometry.exterior_projection = raw.center_projection; geometry.defined |= ExteriorProjection;
                values.counts.negative_exterior_first_volumes += raw.signed_volume < 0.;
                values.counts.primary_reversals += raw.reversed;
                const auto original = main;
                for (unsigned k=0; k<4; ++k) {
                    Require(raw.source_corner[k] < 4, "Invalid native primary corner observation");
                    corner.source_corner[k] = std::uint8_t(raw.source_corner[k]);
                    main.nodes[k] = original.nodes[raw.source_corner[k]];
                }
            }
            geometry.defined |= Area|FirstVolume|SolidLength;
            values.counts.negative_first_volumes += geometry.first_volume < 0.;
            coefficient.has_solid = true;
            coefficient.solid.first = SolidValues(source, solid.first, geometry.area, geometry.first_volume, internal);
            if (internal) {
                const auto second_packet = Packet(input, main, solid.second);
                Numeric(n::EvaluateNativeEightSlotReaderVolume(second_packet.solid_raw, &geometry.second_volume), i);
                geometry.defined |= SecondVolume;
                values.counts.negative_second_volumes += geometry.second_volume < 0.;
                const auto second = SolidValues(source, solid.second, geometry.area, geometry.second_volume, false);
                coefficient.solid.second_fill = second.fill;
                coefficient.solid.second_bulk = second.bulk;
                coefficient.solid.second_volume = second.volume;
            }
            const auto& owner = input.solids[solid.first];
            values.owners[i] = {s::PhysicalSupportKind::EightSlotSolid, owner.source_id, owner.part_id, solid.first};
            values.final_support[i] = {{s::PhysicalSupportKind::EightSlotSolid, owner.source_id}, before.second_solid_source_id};
        }
        const auto shell = old::QuerySupport(input, packed, shells, main, grouping);
        if (!shell.winners.empty()) {
            if (shell.owner == SIZE_MAX)
                Reject(Status::NeedsNativeShellOrder, "Final shell ownership requires native grouping order", i);
            const auto& owner = input.shells.at(shell.owner);
            auto operand = packed.parts.at(packed.shells.at(shell.owner).part).coefficient;
            operand.face = n::MainFaceKind::OrdinaryExterior; operand.layout = owner.primary.layout;
            coefficient.has_shell = true;
            coefficient.shell = operand;
            const auto kind = owner.primary.layout == n::ShellLayout::Triangle3 ?
                s::PhysicalSupportKind::ShellTriangle : s::PhysicalSupportKind::ShellQuad;
            values.owners[i] = {kind, owner.primary.source_id, owner.part_id, shell.owner};
            values.final_support[i] = {{kind, owner.primary.source_id}, 0};
            ++values.counts.shell_owners;
            const auto partner = side.primary_to_partner[i];
            if (partner) {
                Require(partner > p && partner <= g && side.expanded_to_primary[partner-1] == i,
                    "Real shell partner mapping differs from mixed sides");
                coefficient.copy_partner = true;
                values.final_support[partner-1] = values.final_support[i];
            }
        } else {
            if (solid.state != old::SolidSupportState::Resolved)
                Reject(Status::UnsupportedSource, "Mixed primary has no physical main support", i);
            ++values.counts.solid_owners;
        }
        MainResult result;
        Numeric(Evaluate(coefficient, &result), i);
        values.coefficients[i] = result.primary;
        if (coefficient.copy_partner) values.coefficients[side.primary_to_partner[i]-1] = result.partner;
        if (coefficient.has_solid) geometry.solid_length = result.solid_length;
        values.counts.final_internal += values.final_support[i].second_solid_source_id != 0;
    }
    for (std::size_t i=0; i<g; ++i) {
        Require(values.final_support[i].first.source_element_id && std::isfinite(values.coefficients[i]),
            "Expanded main coefficient or final physical support is undefined");
        values.counts.positive_coefficients += values.coefficients[i] > 0.;
        values.counts.zero_coefficients += values.coefficients[i] == 0.;
        values.counts.negative_coefficients += values.coefficients[i] < 0.;
    }
    // Authenticated regular TYPE25 converter writes Idel1. SURFI enables solid
    // erosion when solids exist; GAPM only clears it when pre-shell NSOL_INT0.
    values.incoming_erosion = p > side.shell_primary_count ? s::SolidErosion::Enabled : s::SolidErosion::Disabled;
    values.final_erosion = values.counts.pre_shell_internal ? values.incoming_erosion : s::SolidErosion::Disabled;
}
}
