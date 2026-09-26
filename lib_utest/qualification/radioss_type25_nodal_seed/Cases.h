// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25NodalSeed.h"
#include "lib_src/collision/RadiossType25ShellSource.h"
#include "lib_src/collision/RadiossType25Coefficients.h"
#include "lib_src/elements/beam18/Reference.h"
#include <array>
#include <vector>
namespace type25_seed_test {
namespace n = tlfea::contact::radioss_type25;
namespace seed = n::source_nodal;
namespace shell = n::source_shells;
namespace beam = tl::fea::beam18;
struct Solid {
  int eid = 0;
  std::array<std::uint32_t,8> nodes{};
  n::NativeSolidNodalInput input;
};
enum class DirectKind { Truss, Beam18, Type13, Type25, Type45 };
struct Direct {
  int eid = 0;
  std::array<std::uint32_t,2> nodes{};
  DirectKind kind = DirectKind::Type45;
  // Truss is an already prepared STT operand, not a claimed TMASS port.
  double truss_stiffness = 0;
  beam::Input beam_input;
  n::NativeSpringNodalInput spring;
  double joint_kn = 0; // Native TYPE45 contact STR remains+0, distinct from Kn.
};
struct Case {
  std::size_t nodes = 0;
  std::vector<Solid> solids;
  std::vector<Direct> direct;
  std::vector<shell::PhysicalShell> shells;
};
struct Prepared {
  std::vector<n::NativeVolumeOccurrence> volumes;
  std::vector<n::NativeStiffnessOccurrence> stiffness;
  std::vector<shell::PhysicalShell> shells;
  seed::Input Input(std::size_t nodes) const {
    return {nodes,volumes.empty()?nullptr:volumes.data(),volumes.size(),
      stiffness.empty()?nullptr:stiffness.data(),stiffness.size()};
  }
};
// Qualification packet only. Source IDs/order are explicit synthetic inputs;
// this is not an app generated-EID resolver or whole vehicle source authority.
enum class SpringOrder { NativeEid, PropertyFamilyNegativeControl };
Prepared PreparePort(const Case&, SpringOrder = SpringOrder::NativeEid);
Case MixedCase();
shell::Profile ShellProfile(shell::Population);
} // namespace type25_seed_test
