#pragma once
#include "SourceAssembly.h"
#include "lib_src/elements/type25/Type25Types.h"

namespace crash::modelio::assembly {
enum class SpotweldPolicy { OpenRadiossTonneMillimetreSecondDirectImport=1 };
struct SpotweldDeclaration {
    SpotweldPolicy policy;
    // Generated TYPE25 property identity, never an original shell section ID.
    std::uint64_t generated_property_id;
};

// Source adapter for literal WID/N1/N2-only internal welds. A mandatory named
// policy resolves pinned converter/starter defaults using the README units.
// TL owns frame construction, validation, mass distribution and forces.
class SourceAssemblySpotweldInput {
  public:
    SourceAssemblySpotweldInput(const SourceAssembly&,std::uint64_t source_instance,SpotweldDeclaration);
    SourceAssemblySpotweldInput(const SourceAssemblySpotweldInput&)=default;
    SourceAssemblySpotweldInput(SourceAssemblySpotweldInput&&)=default;
    SourceAssemblySpotweldInput& operator=(const SourceAssemblySpotweldInput&)=delete;
    SourceAssemblySpotweldInput& operator=(SourceAssemblySpotweldInput&&)=delete;
    tl::fea::type25::ModelInput input() const noexcept;
    SpotweldDeclaration declaration() const noexcept { return declaration_; }
    const SourceAssembly& source() const noexcept { return source_; }
  private:
    SourceAssembly source_;
    std::uint64_t source_instance_;
    SpotweldDeclaration declaration_;
    tl::fea::type25::PropertyInput property_;
    std::vector<tl::fea::type25::ConnectionInput> connections_;
};
} // namespace crash::modelio::assembly
