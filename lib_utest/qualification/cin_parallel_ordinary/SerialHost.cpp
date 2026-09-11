// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Serial.h"
#define TL_CIN_SERIAL_DEVICE
#define TL_CIN_SERIAL_KERNEL
#include "serial/Kernel.inc"
#undef TL_CIN_SERIAL_DEVICE
#undef TL_CIN_SERIAL_KERNEL
namespace cin_parallel_test {
void RunSerialHost(const cin_advance::Input& p) {
  AdvanceCin(p.control,p.accepted,p.trial,p.loads,p.fixed,p.model,p.tail,p.work,
    p.patches,p.activity,p.groups,p.durations,p.maximum_angle,p.epoch,p.attempt,
    p.capture,p.rotation_present,p.structural);
}
}
} // namespace tl::fea (opened by exact frozen extract)
