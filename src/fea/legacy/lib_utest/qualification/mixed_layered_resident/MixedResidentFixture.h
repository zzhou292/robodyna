#pragma once
#include "../resident_plasticity/ResidentCollectionFixture.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"
#include "lib_src/elements/ShellBatchSectionBinding.h"
#include "lib_src/elements/qeph/QephLayeredLaw1.h"
#include "lib_src/elements/t3/T3LayeredLaw1.h"

namespace mixed_layered_test {
using namespace resident_plasticity_test;
constexpr unsigned Parents=2;
struct Frame {
  std::array<q::ForceTrial,Parents> qforce;
  std::array<t::ForceTrial,Parents> tforce;
  std::array<fe::ShellBatchLayeredSection,Parents> qsection,tsection;
  fe::ShellBatchDiagnostics diagnostics;
};
bool Initialize(Rig&,fe::ShellBatchSectionBinding&,bool other_family_change=false);
bool ReadFrame(Rig&,Frame&,const fe::ShellBatchDiagnostics* candidate=nullptr);
bool Evaluate(Rig&,const Prepared&,Frame&);
bool Commit(Rig&,const Prepared&,const Frame&);
void SameFrame(const Frame&,const Frame&);
void CheckAdapters(const Rig&,const fe::ShellBatchSectionBinding&,const Prepared&,const Frame&,const Frame&);
// Test-only linked CUDA copy seam. Normal forwarding is exact; armed modes
// complete the final private readback then inject one explicit failure.
enum class ReadFault { None,Nonfinite,DeviceError };
void ArmReadFault(ReadFault);
unsigned ReadFaultCopies();
} // namespace mixed_layered_test
