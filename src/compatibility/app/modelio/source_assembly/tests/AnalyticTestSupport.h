#pragma once
#include "AssemblyTestSupport.h"

namespace crash::modelio::assembly::test::analytic {
inline ArtifactIdentity Identity(bool mixed) {
    return mixed ? ArtifactIdentity{1130084, "48b3fdf74dcc7dffc53fa91574401ead248dae22912f215924b2e148d8974957"}
                 : ArtifactIdentity{768894, "2c4ce206c1b30a3363035ff748a2a5de293fff9cacdc87cf45fc09c46f5cf8b3"};
}
inline std::filesystem::path Path(bool mixed) {
    const auto* name = mixed ? "ROBO_DYNA_SOURCE_ANALYTIC_MIXED_INVENTORY" : "ROBO_DYNA_SOURCE_ANALYTIC_INVENTORY";
    const auto* path = std::getenv(name);
    output::Require(path && *path, "Explicit analytic source fixture is required");
    return path;
}
inline SourceAssembly Load(bool mixed) { return SourceAssembly::Read(Path(mixed), Identity(mixed)); }
} // namespace crash::modelio::assembly::test::analytic
