// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25NodalCorrection.h"
#include "lib_utils/BoundedArena.h"
int main() {
  namespace c = tlfea::contact::radioss_type25::source_nodal::correction;
  double input[]{2, 3}, output[]{0, 0};
  c::Solid solid{{0, 1, 1, 1, 1, 1, 1, 1}, 1, 2, 6};
  c::Input in{input, 2, &solid, 1, nullptr, 0};
  c::Forecast forecast;
  if (c::Preflight(in, {}, forecast).status != c::Status::Ok) return 1;
  tl::util::HostArena arena;
  if (!arena.Initialize(forecast.scratch_bytes)) return 2;
  if (c::Apply(in, {}, arena.data(), arena.bytes(), {output, 2}).status != c::Status::Ok ||
      output[0] != 6 || output[1] != 9) return 3;
  c::FactorResult factor;
  if (c::EvaluateFactor(solid, &factor) != c::Status::Ok || !factor.active || factor.value != 3) return 4;
  const c::OrderInput rows{&solid, 1, 2};
  if (c::PreflightOrderCertificate(rows, {}, forecast).status != c::Status::Ok) return 5;
  tl::util::HostArena proof;
  if (!proof.Initialize(forecast.scratch_bytes)) return 6;
  c::OrderCertificate result;
  return c::CertifyOrder(rows, {}, proof.data(), proof.bytes(), &result).status != c::OrderStatus::Ok ||
      result.controlled_solids != 1 || result.affected_nodes != 2;
}
