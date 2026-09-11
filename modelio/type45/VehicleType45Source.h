#pragma once
#include "modelio/physical_domain/VehiclePhysicalDomain.h"
#include "lib_src/elements/type45/Type45Types.h"
#include <optional>

namespace crash::modelio::type45 {
namespace native = tl::fea::type45;
enum class Policy { OriginalDirectSdiType45V1 };
enum class Disposition { Required, OmittedAssemblyBoundary };
enum class NodeUse { Endpoint, InitialAxis, OriginalEvidence };
enum class BodyKind { None, PlainGroup, PartRoot };
enum class RuntimeReadiness { RequiresOwnerTt0Context };
struct SourceBody {
    BodyKind kind = BodyKind::None;
    std::uint64_t source_id = 0;
    std::size_t source_index = SIZE_MAX;
    bool retained = false;
};
struct Node {
    std::uint64_t source_id = 0;
    std::size_t canonical_index = SIZE_MAX, domain_index = SIZE_MAX;
    native::Vec3 position_m{};
    NodeUse use = NodeUse::OriginalEvidence;
    SourceBody body;
};
struct Row {
    std::uint64_t source_id = 0;
    std::size_t source_index = SIZE_MAX, header_line = 0, card_line = 0, property_index = SIZE_MAX;
    unsigned source_node_count = 0, blank_mask = 0;
    std::array<Node, 4> nodes{};
    // Cylindrical CFG columns 5/6 are _BLANK_. Preserve original literal
    // contents there without inventing extra nodes or runtime connections.
    std::array<std::optional<double>, 2> unused_columns{};
    Disposition disposition = Disposition::Required;
    native::GeometryInput Geometry() const noexcept;
};
struct Property {
    native::Property value;
    // Converter shares the first encountered property of each kind. This is
    // its original evidence identity, never a generated native PID.
    std::uint64_t origin_joint_id = 0;
};
struct Limits {
    std::size_t host_bytes = 512u << 20, rows = 64, metadata_bytes = 1u << 20;
};
struct Forecast {
    std::size_t previous_phase = 0, retained_source = 0, decode_bytes = 0;
    std::size_t mapping_bytes = 0, result_bytes = 0, current_phase = 0, total_bytes = 0;
};
struct Data {
    std::vector<Row> rows; // Original declaration order, including all six boundaries.
    std::array<Property, 3> properties{}; // Spherical, revolute, cylindrical.
    std::size_t required = 0, boundaries = 0, owned_payload_bytes = 0;
};
// Original direct-import source/property/geometry only. Native automatic K,
// main-node registration and TT0 coefficients require the actual owner later.
class VehicleType45Source {
  public:
    static Forecast Preflight(const physical_domain::VehiclePhysicalDomain&, Policy, Limits = {});
    static VehicleType45Source Prepare(const physical_domain::VehiclePhysicalDomain&, Policy, Limits = {});
    const physical_domain::VehiclePhysicalDomain& source_domain() const noexcept;
    const Data& data() const noexcept;
    const Forecast& forecast() const noexcept;
    Policy policy() const noexcept { return Policy::OriginalDirectSdiType45V1; }
    RuntimeReadiness readiness() const noexcept { return RuntimeReadiness::RequiresOwnerTt0Context; }
  private:
    struct Storage;
    explicit VehicleType45Source(std::shared_ptr<const Storage> storage) : storage_(std::move(storage)) {}
    std::shared_ptr<const Storage> storage_;
};
} // namespace crash::modelio::type45
