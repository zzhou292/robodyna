#pragma once
#include "VehicleSectionResolution.h"
#include "modelio/source_assembly/JsonReader.h"

namespace crash::modelio::vehicle::resolution {
struct GlassDeclaration {
    assembly::Part part;
    assembly::Material material;
    assembly::Section section;
    double failure_strain = 0;
    std::optional<double> source_nloc;
    tl::fea::ShellReferencePlacement placement = tl::fea::ShellReferencePlacement::Centered;
};
bool EligibleGlass(const PartDisposition&);
void CheckGlassPolicy(const output::Value&);
GlassDeclaration ReadGlassDeclaration(const output::Value&);
tl::fea::ShellPlasticityMaterialInput NativeGlassMaterial(const assembly::Material&);
// The same literal-card checks gate source eligibility and typed projection.
void CheckGlassMaterialCards(const std::vector<assembly::DeclarationCard>&);
void CheckGlassSectionCards(const std::vector<assembly::DeclarationCard>&);
void CheckGlassCardOrder(const assembly::SourceBlock&, const std::vector<assembly::DeclarationCard>&);
} // namespace crash::modelio::vehicle::resolution
