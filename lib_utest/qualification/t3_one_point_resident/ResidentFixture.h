#pragma once
#include "ResidentSource.h"
#include "../mixed_layered_resident/MixedResidentFixture.h"
#include "../t3_one_point/NativeOracle.h"

namespace t3_one_point_resident_test {
namespace mixed = mixed_layered_test;
namespace temporal = tl_test::nodal_temporal;
namespace t3 = fe::t3;
using Rig = mixed::Rig;
using Frame = mixed::Frame;
using Prepared = mixed::Prepared;
using Cuda = mixed::MixedShellCuda;
using pure::Bytes;
bool Initialize(Rig&, fe::ShellBatchPlasticityBinding&, fe::ShellBatchFailureBinding&,
    bool one_point = true, double failure_strain = 2.5);
bool Prepare(Rig&, unsigned step, Prepared&);
t3::PrescribedInterval Interval(const Rig&, const Prepared&);
void Same(const Frame&, const Frame&);
void NativeAgreement(const Frame&, const Frame&, const pure::Fixture&, const pure::NativeState&);
enum class ReadFault { None, Nonfinite, InvalidFlag, DeviceError, ShellMismatch };
void Arm(ReadFault);
unsigned FaultCopies();
} // namespace t3_one_point_resident_test
