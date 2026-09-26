#pragma once
#include "../MixedInterfaceSource.h"
#include "../solid_surfaces/Internal.h"
#include "../coated/Values.h"
#include "lib_src/collision/RadiossType25FixedMainStartup.h"

namespace crash::cases::vehicle_self_contact::native::mixed_interface::detail {
namespace source = initial_surfaces::values;
struct Failure { Report report; };
[[noreturn]] inline void Reject(Status status, const char* reason) {
    throw Failure{{status, reason}};
}
struct Lookup {
    source::ParentKind kind;
    std::uint64_t element;
    std::uint32_t row, physical;
};
struct Packed {
    initial_surfaces::detail::Packing physical;
    std::vector<source::Face> faces;
    std::vector<std::uint32_t> raw_shell_to_physical; // UINT32_MAX for solid.
    std::vector<std::uint64_t> node_ids;
    std::vector<double> positions; // Native xyz; no round-trip conversion.
    f::Input Input() const;
};
Packed Pack(const coated::Inputs&, const std::vector<std::uint64_t>&,
    const std::vector<initial_surfaces::Face>&);
Report SelectedRoleReport(const coated::Inputs&, const coated::Classification&);
Report Certify(const coated::Inputs&, const Packed&, const coated::Classification&,
    const std::vector<std::uint8_t>& flags, const f::Snapshot&,
    Certificate&, std::vector<RoleObservation>&);
Forecast Budget(const Initial&, Limits);
AdmissionCensus Census(const Initial&);
std::string Digest(const Provenance&, const f::Snapshot&, const s::MixedSidesSnapshot&,
    const std::vector<RoleObservation>&, const Certificate&, std::size_t);
s::Input SideInput(const Packed&, const f::Snapshot&);
}
