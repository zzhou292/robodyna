#pragma once
#include "../InitialSurfaceSource.h"
#include "../coated/Internal.h"
#include "../TopologyDigestFields.h"

namespace crash::cases::vehicle_self_contact::native::initial_surfaces::detail {
struct Failure { Report report; };
[[noreturn]] inline void Reject(Status status, const char* message) {
    throw Failure{{status, message}};
}
inline Report NumericalFailure(NumericalStage stage, values::Report lower, const char* message) {
    Report result;
    result.status = lower.status == values::Status::ResourceLimit ? Status::ResourceLimit :
        (lower.status == values::Status::UnsupportedProfile || lower.status == values::Status::UnsupportedArithmetic)
            ? Status::UnsupportedSource : Status::InvalidInput;
    result.reason = message;
    result.numerical_stage = stage;
    result.numerical = lower;
    return result;
}
struct Packing {
    std::size_t nodes = 0;
    std::vector<values::Solid> solids;
    std::vector<values::Shell> quads, triangles;
    std::vector<std::uint64_t> selected_parts;
    std::vector<std::uint32_t> selected_solids, quad_to_physical, triangle_to_physical;
    values::Input Input(bool suppression_probe) const;
};
struct ShellKey {
    std::array<std::uint32_t, 4> nodes{};
    unsigned arity = 0;
    std::uint64_t element = 0;
    bool selected = false;
};
struct Controls {
    std::string rule;
};
Controls ResolveSelectionControls(const modelio::self_contact::Data&, std::size_t part_cap);
Controls ResolveControls(const Context&, const Selection&, Limits);
Packing Pack(const coated::Inputs&, const std::vector<std::uint64_t>&);
Report CertifyMembership(const Packing&, const values::Snapshot&, Certificate&);
Report CertifyOrder(const std::vector<Face>&, Certificate&);
Face ExternalFace(const values::Face&, const Packing&, const coated::Inputs&);
Forecast Budget(const Context&, const Selection&, Limits);
std::string InputDigest(const coated::Inputs&, const Packing&, const Provenance&, std::size_t);
std::string OutputDigest(const std::vector<Face>&, const std::vector<std::uint8_t>&,
    const Certificate&, const Provenance&, std::size_t);
void CheckCapacity(const coated::Inputs&, const Packing&, std::size_t faces);
}
