#pragma once
#include "modelio/vehicle_source/VehicleSourcePlan.h"
namespace crash::modelio::vehicle::rigid_part {
struct Declaration {
    assembly::Material material;
    assembly::Section section;
    std::vector<assembly::DeclarationCard> part_cards;
};
// Source values only. MAT_RIGID remains in material.source; LayeredLaw1 is
// the explicit converter coefficient role, not independent elastic motion.
Declaration ReadDeclaration(const PartDisposition&, const assembly::SourceUnits&);
}
