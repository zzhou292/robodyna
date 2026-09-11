#pragma once
#include "lib_src/constraints/tied_shell/TiedPostKinChk.h"
#include "native/Packet.h"
#include <array>
#include <stdexcept>
#include <vector>
namespace kinchk_test {
struct NativeControls {
  std::vector<std::int32_t> bodies, members;
  std::int32_t wall = 0, rbe2 = 0, rbe3 = 0, cyclic = 0, print_level = 0;
};
struct NativeResult {
  std::vector<std::int32_t> five, kinet;
  std::array<std::int32_t,8192> decode{};
  std::array<std::int32_t,5> statistics{};
};
inline NativeResult Native(const tl::constraints::tied_shell::KinChkInput& input, NativeControls controls = {}) {
  const auto n = input.slaves.count;
  if (!n || n > 1048576 || controls.bodies.size()%5 || input.interface_decode.count != 8192)
    throw std::runtime_error("Invalid native KINCHK test packet");
  std::vector<std::int32_t> ids(n), five(5*n);
  for (std::size_t i = 0; i < n; ++i) {
    const auto& slave = input.slaves.data[i];
    ids[i] = slave.source_id;
    five[i] = slave.kinematics.conditions;
    five[n+i] = slave.kinematics.translation;
    five[2*n+i] = slave.kinematics.rotation;
    five[3*n+i] = slave.kinematics.duplicate_conditions;
    five[4*n+i] = slave.kinematics.incompatible_conditions;
  }
  NativeResult out;
  out.five.resize(5*n);
  out.kinet.resize(n);
  std::int32_t status = -1;
  native_post_kinchk(n,ids.data(),five.data(),input.interface_decode.data,
    controls.bodies.size()/5,controls.bodies.data(),controls.members.size(),controls.members.data(),
    controls.wall,controls.rbe2,controls.rbe3,controls.cyclic,controls.print_level,
    out.five.data(),out.decode.data(),out.kinet.data(),out.statistics.data(),&status);
  if (status) throw std::runtime_error("Native KINCHK rejected its packet");
  return out;
}
}
