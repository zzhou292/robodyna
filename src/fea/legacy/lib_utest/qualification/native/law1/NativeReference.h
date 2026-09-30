#pragma once
#include <array>
namespace tl::qualification::law1 {
struct PointInput {
  double young=0,nu=0,rho=0,gs=0,layer_thickness=0,reported_thickness=0;
  std::array<double,5> stress{},increment{};
};
struct PointResult {
  std::array<double,5> stress{};
  double thickness=0;
  std::array<double,4> coefficients{}; // native HM_READ_MAT01 G,A11,A12,sound speed.
};
struct SectionInput {
  double young=0,nu=0,rho=0,gs=0,reference_thickness=0,reported_thickness=0;
  std::array<double,8> increment{};
  std::array<double,15> stress{}; // point-major five components, independent native history.
};
struct SectionResult {
  std::array<double,15> stress{};
  std::array<double,5> force{};
  std::array<double,3> moment{};
  double thickness=0;
};
// Test-only serial native context. Pure value inputs and failure-atomic outputs;
// no production material routine is called to validate or evaluate this oracle.
bool Evaluate(const PointInput&,PointResult*) noexcept;
bool Evaluate(const SectionInput&,SectionResult*) noexcept;
} // namespace tl::qualification::law1
