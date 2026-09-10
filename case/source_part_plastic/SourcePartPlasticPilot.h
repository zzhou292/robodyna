#pragma once
#include "case/source_part_elastic/SourcePartElasticCase.h"
#include "case/source_part_wall/SourcePartWallSetup.h"

namespace crash::cases::source_part_plastic {
inline constexpr std::uint64_t PlasticPilotMaximumBaseSteps=262144;
// Same original free part, now using its source curve/Cowper-Symonds parameters.
// A separate named experiment preserves the frozen gentle elastic pilot.
source_part_elastic::Config PlasticWallConfig(const SourcePartMaterial&,unsigned refinement,double speed_m_per_s=8);
source_part_wall::SourcePartWallSettings PlasticWallSettings(const source_part_elastic::Config&);
}
