#include "FrozenValidation.cuh"
namespace tl::fea::seal_test {
void FrozenLaunch(nodal_detail::Control* control, const double* scratch, std::uint32_t n,
    double safety, double minimum, double h, bool rotations, cudaStream_t stream) {
  seal_frozen::Launch(reinterpret_cast<FrozenControl*>(control), scratch, n,
      0, 7, safety, minimum, h, rotations, stream);
}
} // namespace tl::fea::seal_test
