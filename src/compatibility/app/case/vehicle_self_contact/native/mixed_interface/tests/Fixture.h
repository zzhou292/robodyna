#pragma once
#include "../Internal.h"
#include "../../solid_surfaces/tests/Fixture.h"
#include "lib_utest/qualification/radioss_type25_interface_surface/Assertions.h"
namespace crash::cases::vehicle_self_contact::native::mixed_interface::test {
namespace leaf = ::type25_interface_surface_test;
struct Values {
    coated::Inputs geometry;
    std::vector<initial_surfaces::Face> faces;
    detail::Packed packed;
    coated::Classification roles;
    std::vector<std::uint8_t> flags;
    tl::util::HostArena interface_arena, side_arena, scratch;
    f::Snapshot classified;
    s::MixedSidesSnapshot sides;
    Certificate certificate;
    std::vector<RoleObservation> observations;
    explicit Values(const leaf::Case& input) {
        geometry = initial_surfaces::test::Geometry(input.physical);
        for (std::size_t i = 0; i < geometry.nodes.size(); ++i) geometry.nodes[i].native_position = input.points[i];
        const auto physical = initial_surfaces::detail::Pack(geometry, input.physical.parts);
        for (const auto& face : input.raw) faces.push_back(initial_surfaces::detail::ExternalFace(face, physical, geometry));
        packed = detail::Pack(geometry, input.physical.parts, faces);
        roles = coated::Classify(geometry);
        const leaf::old::Built initial(input.physical);
        flags.assign(initial.result.surface_solid_flags, initial.result.surface_solid_flags+initial.result.solid_count);
        f::Forecast forecast;
        const auto packet = packed.Input();
        if (f::Preflight(packet, {}, forecast).status != f::Status::Ok ||
            !interface_arena.Initialize(forecast.output_bytes) || !scratch.Initialize(forecast.scratch_bytes) ||
            f::Build(packet, {}, interface_arena, scratch, &classified).status != f::Status::Ok)
            throw std::runtime_error("Mixed app fixture classification failed");
    }
    Report Certify() {
        return detail::Certify(geometry, packed, roles, flags, classified, certificate, observations);
    }
    void Expand() {
        const auto input = detail::SideInput(packed, classified);
        const auto forecast = s::PreflightMixedSides(input);
        tl::util::HostArena side_scratch;
        if (forecast.status != s::Status::Ok || !side_arena.Initialize(forecast.output_bytes) ||
            !side_scratch.Initialize(forecast.scratch_bytes) ||
            s::BuildMixedSides(input, {}, side_arena, side_scratch, &sides).status != s::Status::Ok)
            throw std::runtime_error("Mixed app fixture sides failed");
    }
};
}
