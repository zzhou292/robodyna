#pragma once
#include "EffectiveSource.h"
#include "modelio/tied_shell/Internal.h"
#include "modelio/self_contact/PartSets.h"
#include "output/BoundedArrayIO.h"
#include <map>
#include <stdexcept>
namespace crash::modelio::solid_control::detail {
namespace reader = modelio::assembly::reader;
using output::Require;
struct Failure : std::runtime_error {
    Report report;
    explicit Failure(Report value) : std::runtime_error(value.reason), report(std::move(value)) {}
};
[[noreturn]] inline void Reject(Status status, const char* message,
                               const std::string& file = {}, std::size_t line = 0) {
    throw Failure({status, message, file, line});
}
struct Part {
    std::uint64_t id = 0, section = 0, material = 0;
    std::string member;
    std::size_t line = 0;
};
struct Section { std::uint64_t id = 0; std::string keyword; };
DirectData ReadDirect(const source::CanonicalData&, const ids::ImportMembers&, std::size_t metadata_bytes);
std::vector<PartControl> ResolveSharing(const std::vector<Part>&,
                                      const std::vector<Section>&, const std::vector<std::uint64_t>&);
EffectiveData ReadEffective(const DirectSource&, const ids::ImportContext&,
                            const ids::ImportMembers&, Limits);
} // namespace crash::modelio::solid_control::detail
