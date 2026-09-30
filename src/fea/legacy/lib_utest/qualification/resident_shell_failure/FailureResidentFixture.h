#pragma once
#include "FailureResidentSource.h"
#include "../mixed_layered_resident/MixedResidentFixture.h"
#include "../shell_failure_force/NativeAgreement.h"
#include "lib_src/elements/failure/ShellFailureValues.h"

namespace resident_failure_test {
namespace mixed=mixed_layered_test;
using Rig=mixed::Rig;
using Prepared=mixed::Prepared;
namespace qe=fe::qeph;
namespace tr=fe::t3;
using Cuda=mixed::MixedShellCuda;
struct Frame {
  mixed::Frame material;
  std::array<fe::ShellBatchFailureState,Parents> qfailure,tfailure;
};
bool Initialize(Rig&,fe::ShellBatchPlasticityBinding&,fe::ShellBatchFailureBinding&,bool enabled=true,bool mismatch=false,double failure_strain=1e-6);
bool Read(Rig&,Frame&,const fe::ShellBatchDiagnostics* candidate=nullptr);
bool Evaluate(Rig&,const Prepared&,Frame&);
void Same(const Frame&,const Frame&);
void CheckForceAdapters(const Rig&,const fe::ShellBatchPlasticityBinding&,const Prepared&,const Frame&,const Frame&);
enum class Fault {None,Nonfinite,InvalidFlag,DeviceError};
void Arm(Fault);
unsigned Copies();
} // namespace resident_failure_test
