#pragma once
#include "NodalWallOwnerFixture.h"

namespace nodal_wall_native_test {
using namespace nodal_wall_owner_test;
enum class Layout { SingleT3,PairT3,Mixed };

// Actual qualified native startup masses and TOTAL rotary inertia. Geometry
// is immutable during preparation; subsequent x/v are prescribed contact
// probes, not a native shell force/history or coupled-shell admission.
struct NativeFixture : Fixture {
  explicit NativeFixture(Layout layout,bool edge_on=false,unsigned cyclic=0);
  bool PrepareNative(bool reverse=false);
  void Depth(double depth);
  unsigned first() const { return layout==Layout::Mixed?0:1; }
  Layout layout;
  sc::SurfaceTriangle triangles[2]{};
  sc::T3MaterialMeasure measures[2];
  std::array<double,Capacity> mass{},total_j{};
  std::array<long double,Capacity> truth_mass{},truth_j{},node_area{};
  std::array<long double,2> parent_area{};
 private:
  bool preparation_started_=false; // Test fixture is one-shot, including a failed preparation.
};
void CheckMass(const NativeFixture& f);
void CheckIndependent(const NativeFixture& f,const sc::NodalWallDeviceResults& result,
                      const std::array<double,3*Capacity>& x,
                      const std::array<double,3*Capacity>& v,double stiffness=16);
void CheckFaces(const NativeFixture& f,const sc::NodalWallDeviceResults& result,
                const std::array<double,3*Capacity>& x);
} // namespace nodal_wall_native_test
