#pragma once
#include "modelio/tied_shell/TiedShellSearchGeometry.h"
#include "output/full_shell/static_bundle/tests/ActualMappingSupport.h"
namespace crash::cases::vehicle_startup::test {
inline modelio::tied_shell::TiedShellSearchGeometry OriginalTiedGeometry() {
    namespace tied = modelio::tied_shell;
    auto inputs = tied::source::test::ActualInputs();
    inputs.scope_report.bytes = 13212691;
    inputs.scope_report.sha256 = "fdb51869dfd3f4de265bb4494a2d0f904c5c466bf9962b71ce19e9098a761ff0";
    const auto canonical = tied::source::CanonicalSource::Read(inputs);
    const auto member = output::ReadBounded(std::getenv("ROBO_STATIC_MEMBER"), 42846753);
    const auto declaration = tied::TiedShellDeclaration::Prepare(canonical, member);
    const auto packing = tied::TiedShellPacking::Prepare(declaration);
    return tied::TiedShellSearchGeometry::Prepare(packing, member);
}
}
