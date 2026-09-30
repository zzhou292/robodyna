#pragma once
#include "Resolve.h"
#include "modelio/tied_shell/Internal.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include "output/BoundedArrayIO.h"
#include <stdexcept>
namespace crash::modelio::native_spring_ids::detail {
using output::Require;
namespace reader = assembly::reader;
struct Failure : std::runtime_error {
    Diagnostic diagnostic;
    explicit Failure(Diagnostic value) : std::runtime_error(value.reason), diagnostic(std::move(value)) {}
};
[[noreturn]] inline void Reject(Readiness status, const std::string& reason,
        const std::string& file = {}, std::size_t line = 0, std::uint64_t id = 0) {
    throw Failure({status, reason, file, line, id});
}
void CheckLimits(Limits);
void PopulateElements(const source::CanonicalData&, const output::Document&, ContextData&, Limits);
void PopulateConnections(ContextData&, Limits);
ContextData BuildContext(const source::CanonicalData&, const ImportMembers&, Limits);
Resolution ResolveRows(const ContextData&, std::vector<SourceRow> retained, Limits);
std::string DigestRows(const std::vector<Row>&, const std::string& binding);
} // namespace crash::modelio::native_spring_ids::detail
