#pragma once

#include "lib_src/collision/Q4PrescribedPlanarContact.h"
#include "lib_utest/q4_planar_geometry_fixture.h"

#include <array>
#include <cstring>
#include <vector>

namespace q4_prescribed_test {
namespace sc=tlfea::contact;
// Independent, genuine physical mass data. Existing fixtures are reused only
// for their Q4 geometry/views; their old component-mass inputs are never cast
// or converted into this free-XYZ/fully-fixed contract.
template<std::size_t N> struct PhysicalMass {
  std::array<double,N> inverse{};
  std::array<std::uint8_t,N> fixed{};
  PhysicalMass() { inverse.fill(1); }
  sc::LumpedTranslationMassView view() const {
    return {inverse.data(),fixed.data(),static_cast<std::uint32_t>(N),9,sc::TranslationMassModel::kIsotropicLumped};
  }
};
struct Scratch {
  std::vector<sc::Q4RectangularCell> leaves=std::vector<sc::Q4RectangularCell>(sc::MaxQ4IntegrationLeaves);
  std::vector<std::uint32_t> heap=std::vector<std::uint32_t>(sc::MaxQ4IntegrationLeaves);
  sc::Q4RectangularScratch view() { return {leaves.data(),heap.data(),sc::MaxQ4IntegrationLeaves,sc::MaxQ4IntegrationLeaves}; }
};
inline sc::Q4PrescribedPlanarConfig Config() {
  sc::Q4PrescribedPlanarConfig config;
  config.stiffness_per_area=16; config.maximum_penetration=.125;
  config.integration.force_error=1e-7; config.integration.energy_error=1e-9;
  config.sweep={q4_planar_test::Clearance,1e-6}; return config;
}
struct Prepared {
  sc::PlanarWallGeometry wall;
  sc::Q4PlanarGeometry geometry;
  sc::Q4PrescribedPlanarConfig config=Config();
  Scratch scratch;
  bool Initialize(const q4_planar_test::Wall& input,const sc::Q4SurfaceView& reference) {
    return wall.Initialize(input.view()).status == sc::PlanarContactStatus::Ok &&
           geometry.InitializeReference(wall,reference,config.sweep.exposed_clearance).status == sc::PlanarContactStatus::Ok;
  }
  sc::Q4PrescribedPlanarReport Evaluate(const sc::Q4SurfaceView& base,const sc::Q4SurfaceView& endpoint,
                                      sc::LumpedTranslationMassView mass,sc::Q4PrescribedPlanarResult* result) {
    return sc::IntegrateQ4PrescribedPlanarContact(wall,geometry.view(),base,endpoint,mass,config,7,scratch.view(),result);
  }
};
template<class T> std::array<unsigned char,sizeof(T)> Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> result; std::memcpy(result.data(),&value,sizeof(value)); return result;
}
inline void TransformProjection(q4_planar_test::Single* surface,double yy,double yz,double zy,double zz,
                                double y0=0,double z0=0) {
  for (unsigned n=0;n<4;++n) {
    const double y=surface->position[n+4],z=surface->position[n+8];
    surface->position[n+4]=yy*y+yz*z+y0; surface->position[n+8]=zy*y+zz*z+z0;
  }
}
}  // namespace q4_prescribed_test
