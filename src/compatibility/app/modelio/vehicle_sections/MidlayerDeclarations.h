#pragma once
#include "VehicleSectionResolution.h"

namespace crash::modelio::vehicle::resolution {
struct MidlayerDeclaration {
    assembly::Material material;
    assembly::Section section;
    std::vector<assembly::DeclarationCard> part_cards;
    double failure_strain = 0;
};
// Source-field interpretation only. Factory separately authenticates the exact
// original source/profile, complete selection and immutable base association.
MidlayerDeclaration ReadMidlayer(const PartDisposition&, const assembly::SourceUnits&);
struct NativeMappingValues {
    std::vector<NativeParentMapping> mapping;
    std::vector<tl::fea::ShellFailureParentInput> parents;
    NativeFormulationCounts counts;
};
NativeMappingValues MapNativeParents(const std::vector<SectionParentResolution>&,
    const std::vector<SectionPartResolution>&, const std::vector<PartDisposition>&);
} // namespace crash::modelio::vehicle::resolution
