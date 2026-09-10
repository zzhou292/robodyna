#pragma once
#include "../t3/mixed/MixedShellFixture.h"
#include "lib_src/elements/ShellBatchPlasticity.h"
#include "lib_src/elements/qeph/QephLayeredJ2.h"
#include "lib_src/elements/t3/T3LayeredJ2.h"

namespace resident_plasticity_test {
using namespace mixed_shell_test;
inline constexpr double CurveX[]{0,.1,.3},CurveY[]{2700,3400,3620};
struct SectionPair { fe::ShellBatchSectionState q{},t{}; };
bool Initialize(Rig&,bool mismatch_curve=false,bool triangle_elastic=false);
bool InitializeRate(Rig&,tl::material::TabulatedShellPlasticityRate,bool mismatch_rate=false);
Loads PlasticSchedule(const Rig&,unsigned interval);
bool Sections(Rig&,SectionPair&,const Staged* prepared=nullptr);
void SameSections(const SectionPair&,const SectionPair&);
void CheckHostAdapters(const Rig&,const Prepared&,const Staged& old_shell,
                       const SectionPair& old_section,const Staged&,const SectionPair&,
                       tl::material::TabulatedShellPlasticityRate rate={});
} // namespace resident_plasticity_test
