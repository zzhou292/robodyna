#pragma once
#include "case/vehicle_wall/Settings.h"
#include "modelio/physical_domain/VehiclePhysicalDomain.h"
#include "modelio/native_spring_ids/ImportContext.h"
#include "lib_src/elements/qeph/QephData.h"
#include "lib_src/elements/ShellBatchStartup.h"
#include <array>
#include <optional>

namespace crash::cases::vehicle_wall::native {
enum class Profile { Unspecified, EnvelopeFixedElasticV1 };
enum class NamespacePhase { CompleteDeclaredInputBeforeNativeGeneratedEntities };
enum class Status { Ready, InvalidInput, UnsupportedSource, ResourceLimit, Unrepresentable, IdentityMismatch };
struct MaterialDeclaration {
    double young_pa=200e9, poisson=.3, density_kg_m3=7860, thickness_m=.001;
};
struct Declaration {
    Profile profile=Profile::Unspecified;
    MaterialDeclaration material;
    double wall_friction=.6; // Separate interface declaration, not material damping.
    double requested_duration_s=.02, leading_gap_m=.02, transverse_margin_m=.01, exposed_clearance_m=1e-6;
    std::uint64_t binding_id=0x56453557414c4c31ULL;
};
struct Limits {
    std::size_t host_bytes=std::size_t{8}<<30;
    std::size_t namespace_entries=2097152, namespace_bytes=std::size_t{256}<<20;
    std::size_t source_metadata_bytes=std::size_t{16}<<20, domain_bytes=std::size_t{64}<<20;
    std::size_t metadata_bytes=1u<<20;
};
struct AllocatedIds {
    std::array<std::uint64_t,4> nodes{};
    std::uint64_t shell=0,part=0,material=0,section=0,node_set=0,surface=0,interface=0;
};
struct NamespaceReport {
    NamespacePhase phase=NamespacePhase::CompleteDeclaredInputBeforeNativeGeneratedEntities;
    std::uint64_t maximum_declared=0, maximum_spring=0, allocation_ceiling=0;
    std::size_t definitions=0, members=0;
    std::string source_digest, digest;
    std::string display_material_keyword, display_material_block_sha256;
    double display_density_native=0, display_young_native=0, display_poisson=0;

    // Generated rigid primaries are allocated after all combined explicit NODE
    // definitions by the pinned fresh PO backend. No observed ID table is used.
    bool generated_node_after_complete_input=false;
};
struct Geometry {
    PlacementValues placement;
    tlfea::contact::PlanarWallBox envelope;
    std::array<tl::math::Vec3,4> reference_m{}, reference_native{};
    std::array<std::uint64_t,4> display_feature_ids{}; // Provenance only, never native IDs.
    tl::fea::qeph::ReferenceInput reference_input;
    tl::fea::qeph::ReferenceData reference;
    double native_half_gap=0, reference_offset_m=0, reference_plane_m=0;
    double component_primary_stiffness_native=0, wall_mass_kg=0;
    double native_working_length_m=0;
};
struct VehiclePrefix {
    std::size_t begin=0, nodes=0;
    std::array<tlfea::contact::Vec3,2> reference_bounds;
    // Prefix is the mass/metric/rendering selection. Source preparation has no
    // vehicle coefficient ledger, so no fabricated vehicle mass is published.
    std::string source_digest;
};
struct Forecast {
    std::size_t retained_vehicle=0, import_context=0, namespace_workspace=0;
    std::size_t common_domain=0, input_packing=0, masks=0, geometry=0, digest=0;
    std::size_t peak_bytes=0;
};
struct Report { Status status=Status::InvalidInput;std::string reason,file;std::size_t row=SIZE_MAX;std::uint64_t source_id=0; };
} // namespace crash::cases::vehicle_wall::native
