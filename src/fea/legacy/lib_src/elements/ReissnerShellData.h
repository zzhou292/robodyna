#pragma once

#include "ReissnerFrame.h"

namespace tl::fea::reissner {

// Compact immutable input for the prescribed elastic Q4 operation. These are
// values, not views: no Chrono objects, allocation, device pointers or history.
// Natural node order is (+,+), (-,+), (-,-), (+,-). All matrices are row-major.
// The setup adapter copies actual Chrono reference fields once; zero/default
// data is deliberately unprepared and must not be admitted by the force API.
struct ShellPointReference {
  double natural[2]{};
  double shape[4]{};
  double gradient[4][2]{};  // Reference material-coordinate derivatives.
  Matrix3 frame_offset{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  Vec3 strain0[2]{};
  Vec3 curvature0[2]{};  // Used only at Gauss points; ANS does not use curvature.
  double area_weight = 0;  // alpha * w at Gauss points; zero at ANS points.
};

struct ShellReference {
  Quaternion node_frame_offset[4]{};  // Actual iTa converted once to a quaternion.
  ShellPointReference gauss[4]{};
  ShellPointReference ans[4]{};
  Vec3 initial_position[4]{};
  Quaternion initial_rotation[4]{};
  bool prepared = false;  // Validated, one-time planar-rectangle setup only.
};

struct ElasticSection {
  // Resultants = C * strain in order (eps1, eps2, k1, k2), each a 3-vector.
  // This is one centered isotropic elastic layer with fixed positive factors.
  double stiffness[144]{};
  double thickness = 0;
  double density = 0;
  bool prepared = false;
};

struct ShellConfiguration {
  Vec3 position[4]{};
  Quaternion rotation[4]{};  // Nodal frame; physical frame includes right offset.
};

struct ShellResult {
  Vec3 force[4]{};
  Vec3 couple[4]{};  // WORLD spin work-conjugate couples, not nodal-local values.
  double strain[4][12]{};
  double resultant[4][12]{};
  double energy = 0;
  double bending_energy = 0;
};

}  // namespace tl::fea::reissner
