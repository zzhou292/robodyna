// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/elements/publication/physical_activity/Device.h"
namespace physical_activity_test {
namespace { bool armed = false; }
void ArmCopyFailure() { armed = true; }
extern "C" cudaError_t __real_cudaMemcpyAsync(void*, const void*, std::size_t, cudaMemcpyKind, cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* destination, const void* source,
    std::size_t bytes, cudaMemcpyKind kind, cudaStream_t stream) {
  if (armed && kind == cudaMemcpyDeviceToHost && bytes == 2*sizeof(fe::physical_activity::FamilyControl)) {
    armed = false; return cudaErrorInvalidValue;
  }
  return __real_cudaMemcpyAsync(destination, source, bytes, kind, stream);
}
TEST(PhysicalActivityCuda, GenuineInputsRejectInvalidBaseAndReactivationDeterministically) {
  Fixture f; ASSERT_TRUE(f.Initialize()); ASSERT_TRUE(f.Begin());
  namespace a = fe::physical_activity;
  fe::ShellPhysicalDiagnostics diagnostics;
  ASSERT_TRUE(p::Good(f.rig.publication.CopyAcceptedPhysicalDiagnostics(f.rig.owner.accepted(), &diagnostics)));
  a::T3Input input;
  ASSERT_TRUE(Good(a::BatchAccess::Borrow(f.rig.t3, f.rig.owner, f.rig.publication,
      f.rig.fixture.physical, f.token, &f.assembly, nullptr, diagnostics.t3, input)));
  void* device = nullptr; ASSERT_EQ(cudaMalloc(&device, 64), cudaSuccess);
  auto* control = static_cast<a::FamilyControl*>(device);
  auto* bytes = reinterpret_cast<std::uint8_t*>(device) + sizeof(a::FamilyControl);
  fe::ShellSectionLaw law; ASSERT_TRUE(f.rig.fixture.catalog.Law(fe::ShellBindingFamily::T3, 0, &law));
  for (unsigned repeat = 0; repeat < 3; ++repeat) for (std::uint8_t base : {std::uint8_t(0),std::uint8_t(2)}) {
    const std::uint8_t values[]{std::uint8_t(law), base, 77}; a::FamilyControl result;
    ASSERT_EQ(cudaMemcpyAsync(control, &result, sizeof(result), cudaMemcpyHostToDevice, f.assembly.stream), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(bytes, values, sizeof(values), cudaMemcpyHostToDevice, f.assembly.stream), cudaSuccess);
    ASSERT_EQ(a::Capture(input, {bytes, bytes + 1, bytes + 2, control}, f.assembly.stream), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&result, control, sizeof(result), cudaMemcpyDeviceToHost, f.assembly.stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(f.assembly.stream), cudaSuccess);
    const auto r = a::Decode(result, fe::PhysicalActivityFamily::T3);
    EXPECT_EQ(r.status, base == 0 ? Status::Reactivation : Status::InvalidActivity);
    EXPECT_EQ(r.family_index, 0u); EXPECT_EQ(Read(bytes + 2, 1, f.assembly.stream)[0], 77u);
  }
  EXPECT_EQ(cudaFree(device), cudaSuccess); f.Discard();
}
} // namespace physical_activity_test
