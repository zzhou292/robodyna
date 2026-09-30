// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Serial.h"
// Complete selected caller at baseline12a957f. Includes unchanged native leaves.
#define TL_CIN_SERIAL_DEVICE __device__
#define TL_CIN_SERIAL_KERNEL __global__
#include "serial/Kernel.inc"
#undef TL_CIN_SERIAL_DEVICE
#undef TL_CIN_SERIAL_KERNEL
namespace cin_parallel_test {
cudaError_t LaunchSerial(const cin_advance::Input& p, cudaStream_t stream) {
  AdvanceCin<<<1,1,0,stream>>>(p.control,p.accepted,p.trial,p.loads,p.fixed,
    p.model,p.tail,p.work,p.patches,p.activity,p.groups,p.durations,p.maximum_angle,
    p.epoch,p.attempt,p.capture,p.rotation_present,p.structural);
  return cudaGetLastError();
}
} // namespace cin_parallel_test
} // namespace tl::fea (opened by exact frozen extract)
