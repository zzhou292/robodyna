#pragma once
#include "SourceAssemblyBindingTestSupport.h"

namespace crash::cases::source_assembly::test {
inline SourceAssemblyBindingOptions BracketOptions() {
    auto value=Options();
    value.spotweld=source::SpotweldDeclaration{
        source::SpotweldPolicy::OpenRadiossTonneMillimetreSecondDirectImport,383348001108};
    return value;
}
inline source::SourceAssembly BracketSource() {
    const auto* path=std::getenv("ROBO_DYNA_SOURCE_BRACKET_INVENTORY");
    output::Require(path&&*path,"Explicit bracket source inventory required");
    return source::SourceAssembly::Read(path,source::PinnedYarisSevenPartInventory());
}
inline SourceAssemblyBindings BracketBindings() {return SourceAssemblyBindings::Prepare(BracketSource(),BracketOptions());}
} // namespace crash::cases::source_assembly::test
