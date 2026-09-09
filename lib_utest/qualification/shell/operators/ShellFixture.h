#pragma once

// Bounded donor qualification fixture, not a constitutive model or FE backend.
#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace shell_spike {

struct Vec3 {
  double x = 0, y = 0, z = 0;
};
using Tensor = std::array<double, 9>;  // Row-major, symmetric, world coordinates.

struct Element {
  std::array<int, 4> nodes{};  // Zero-based, four distinct nodes in perimeter order.
  double thickness = 1;
  // Prescribed physical resultants, already integrated through thickness.
  // Membrane has force/length units, bending has force units, shear force/length.
  // Both tensors and shear must lie in the element tangent plane.
  Tensor membrane{};
  Tensor bending{};
  Vec3 shear{};
};

struct Input {
  std::vector<Vec3> positions;
  std::vector<Vec3> velocity;
  std::vector<Vec3> angular_velocity;
  std::vector<Element> elements;
};

enum class Status {
  Ok,
  EmptyInput,
  InvalidInput,
  UnsupportedGeometry,
  ResourceLimit,
  CudaError,
  InvalidOutput,
  TrialRejected,
};

struct Options {
  std::size_t max_device_bytes = 64 * 1024;
  // Fixture-only fault injection, after GPU execution and finite-output checks.
  bool reject_after_assembly = false;
};

struct Result {
  std::vector<Vec3> forces, moments;
  std::vector<std::array<Vec3, 3>> frames;
  std::vector<double> areas;
  // Donor scratch histories are returned to make zero initialization observable.
  // No material history, timestep, or constitutive update is implemented here.
  std::vector<double> off, reference_coordinates, hourglass, energy;
  std::size_t device_bytes = 0;
};

struct Attempt {
  Status status = Status::InvalidInput;
  std::string message;
  std::size_t device_bytes = 0;
};

// Maximum 16 planar, strictly convex Q4 elements and 64 nodes. Fresh trial
// scratch each call; hourglass coefficients zero, NPT=3, ISMSTR=1, no timestep
// stiffness qualification. General-quad transverse shear uses the donor's
// center-point rotational interpolation. No K2, plasticity, bending stiffness,
// contact, or time integration is involved.
//
// `committed` is replaced only on Ok. Input is const and never uploaded through
// writable aliases. Invalid/empty/budget failures happen before CUDA calls.
Attempt Evaluate(const Input& input, Result* committed,
                 const Options& options = {});

}  // namespace shell_spike
