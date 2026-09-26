#pragma once
#include "../CorrectedNodalSource.h"
#include "modelio/native_spring_ids/ImportContext.h"
#include "modelio/tied_shell/Internal.h"
#include "modelio/self_contact/PartSets.h"
#include "output/BoundedArrayIO.h"
#include <map>
#include <stdexcept>

namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail {
namespace ids = modelio::native_spring_ids;
namespace reader = modelio::assembly::reader;
namespace source = output::full_shell::source;
namespace c = n::source_nodal::correction;
using output::Require;
struct Failure : std::runtime_error {
    Report report;
    explicit Failure(Report value) : std::runtime_error(value.reason), report(std::move(value)) {}
};
[[noreturn]] inline void Reject(Status status, const char* message,
    const std::string& file = {}, std::size_t line = 0) {
    Report report;
    report.status = status;
    report.reason = message;
    report.source_file = file;
    report.source_line = line;
    throw Failure(std::move(report));
}
struct Part {
    std::uint64_t id = 0, section = 0, material = 0;
    std::string member;
    std::size_t line = 0;
};
struct Section {
    std::uint64_t id = 0;
    std::string keyword;
};
struct Context {
    std::vector<PartControl> parts;
    InterfaceCensus interfaces;
    std::string source_digest, property_digest;
};
// Pure relation helper used by the authenticated context factory. Its input
// tables alone carry no source authority; the public factory authenticates them.
std::vector<PartControl> ResolveSharing(const std::vector<Part>&,
    const std::vector<Section>&, const std::vector<std::uint64_t>& requested);
Context ReadContext(const seed::PreCorrectionNodalSource&,
    const ids::ImportMembers&, const ids::ImportContext&, Limits);
Forecast Budget(const seed::PreCorrectionNodalSource&, const ids::ImportMembers&, Limits);
void PackControls(const seed::PreCorrectionNodalSource&, const Context&,
    std::vector<c::Solid>&, std::vector<std::uint64_t>& source_ids);
std::string MaterialDigest(const std::vector<c::Solid>&, const Context&, std::size_t cap);
std::string CertificateDigest(const c::OrderCertificate&, const std::string& material, std::size_t cap);
} // namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail
