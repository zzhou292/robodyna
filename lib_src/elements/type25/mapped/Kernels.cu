// SPDX-License-Identifier: AGPL-3.0-or-later
#include "AssemblyValues.h"
#include "../../mapped_connector/Kernels.cuh"
namespace tl::fea::type25::batch_detail {
void LaunchMappedAssembly(Storage* storage, const Slab* accepted, NodalAssemblyView view,
    NodalCinAssemblyView cin, bool initial) {
  mapped_connector::Launch<AssemblyFamily>(storage, accepted, view, cin, initial);
}
} // namespace tl::fea::type25::batch_detail
