#pragma once
#include "TopologyAssessment.h"
#include "coated/Types.h"
#include "case/vehicle_startup/physical_model/VehiclePhysicalModel.h"
namespace crash::cases::vehicle_self_contact::native::coated {
using PhysicalModel = vehicle_startup::physical_model::VehiclePhysicalModel;
enum class Scope { RetainedV5PhysicalShellsAndOriginalContact };
enum class SourceCoordinates { OriginalNativeNodeCards };
enum class NativeOrder { CaseDeclaredAscendingPhysicalNidItab };
enum class SurfaceMembership { SingleSurfaceImbinZero };
struct Config {
    Scope scope = Scope::RetainedV5PhysicalShellsAndOriginalContact;
    SourceCoordinates coordinates = SourceCoordinates::OriginalNativeNodeCards;
    NativeOrder order = NativeOrder::CaseDeclaredAscendingPhysicalNidItab;
    SurfaceMembership membership = SurfaceMembership::SingleSurfaceImbinZero;
};
struct Limits {
    // New assessment envelope. Existing physical-model/source caps are retained;
    // their inclusive backing is charged before this incremental workspace.
    std::size_t host_bytes = std::size_t{8} << 30;
    std::size_t nodes = 524288, shells = 524288, solids = 16384;
    std::size_t metadata_bytes = 1u << 20;
    s::Limits topology;
};
struct Forecast {
    std::size_t retained_model_reservation = 0, selection_reservation = 0, member_reservation = 0;
    std::size_t input_bytes = 0, decode_peak = 0, membership_scratch = 0;
    std::size_t ordering_scratch = 0, role_bytes = 0, result_bytes = 0;
    std::size_t peak_bytes = 0;
    std::size_t node_size = 0, shell_size = 0, solid_size = 0, role_size = 0;
    s::Forecast topology;
    bool admitted = false;
};
struct Result {
    Config config;
    Forecast forecast;
    RoleCounts physical, contact;
    std::size_t physical_nodes = 0, physical_solids = 0, original_solids = 0;
    std::array<std::size_t, 5> solid_families{};
    std::size_t candidate_visits = 0, coordinate_roundtrip_changes = 0;
    bool physical_roles_complete = false, selected_roles_complete = false, topology_attempted = false, topology_complete = false;
    Location first_unready, first_selected_unready, failure, first_warning;
    s::Status role_status = s::Status::Ok, selected_role_status = s::Status::Ok;
    s::Report topology_report;
    std::size_t output_mains = 0, output_references = 0, output_incidences = 0;
    Provenance provenance;
    std::vector<selection::PartDisposition> contact_parts;
    Digest input_digest, output_digest;
};
Forecast Preflight(const PhysicalModel&, const selection::OriginalSelection&, Config = {}, Limits = {});
// Uses actual prepared model/source handles, never omitted original solids or
// captured roles. The original member is authenticated before source scanning.
// Report/digests only: no physical owner, runtime admission or topology borrow.
Result AssessCoatedSource(const PhysicalModel&, const selection::OriginalSelection&,
    const std::string& original_member, Config = {}, Limits = {});
output::Document ResultDocument(const Result&, std::size_t byte_cap = 1u << 20);
output::Document ForecastDocument(const Forecast&, std::size_t byte_cap = 1u << 20);
} // namespace crash::cases::vehicle_self_contact::native::coated
