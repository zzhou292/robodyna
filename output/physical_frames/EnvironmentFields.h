#pragma once
#include "RecordFields.h"
#include <array>
namespace crash::output::physical_frames::detail {
// Numeric capture description only. Mapping binds it to the genuine immutable
// DomainEmbedding/source; this helper grants no independent source authority.
struct FixedEnvironmentField {
    std::size_t qeph_index=0;
    std::array<std::size_t,4> nodes{};
    std::array<tl::math::Vec3,4> reference{};
};
void CheckFixedEnvironment(const FixedEnvironmentField&,const double* positions,const double* velocities,
    std::size_t physical_nodes,const tl::fea::ShellBatchLayeredSection*,const std::uint8_t*,std::size_t physical_qeph);
}
