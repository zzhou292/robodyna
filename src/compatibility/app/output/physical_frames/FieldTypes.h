#pragma once
#include "output/full_shell/FullShellVisualizationSchema.h"
#include "lib_src/elements/ShellBatchPlasticityBinding.h"
namespace crash::output::physical_frames {
struct FamilyCounts { std::size_t qeph=0, t3=0, qbat=0; };
namespace records=full_shell;
inline constexpr std::uint32_t QephFamily=1,T3Family=2,QbatFamily=3;
struct ParentField {
    std::uint32_t family=0,index=0;
    tl::fea::ShellSectionLaw law=tl::fea::ShellSectionLaw::Unspecified;
};
std::uint32_t Family(tl::fea::ShellBindingFamily);
records::PlasticField Plasticity(tl::fea::ShellSectionLaw);
}
