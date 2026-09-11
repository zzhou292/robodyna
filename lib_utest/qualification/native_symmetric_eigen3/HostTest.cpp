#include "TestSupport.h"
#include <gtest/gtest.h>

namespace spectrum_test {
TEST(NativeSpectrumHost, ZeroHydrostaticAndNativeCoordinateFallback) {
  for (double hydro : {0., .125, -.25}) {
    const auto packet = Diagonal(hydro, hydro, hydro);
    Spectrum result;
    ASSERT_TRUE(tl::math::NativeSymmetricEigen3(packet.tensor, result));
    for (unsigned k = 0; k < 3; ++k) EXPECT_EQ(result.value[k], hydro);
    for (unsigned r = 0; r < 3; ++r)
      for (unsigned c = 0; c < 3; ++c)
        EXPECT_EQ(result.vectors.v[3*r+c], r == c ? 1 : 0);
  }
  Packet near;
  const double values[6]{1e-12, -2e-12, 3e-12, 4e-13, -8e-13, 2e-13};
  std::copy_n(values, 6, near.tensor);
  Spectrum result;
  detail::Work work;
  ASSERT_TRUE(detail::Roots(near.tensor, work, result));
  ASSERT_EQ(detail::Path(work), detail::DirectionPath::NearTriple);
  ASSERT_TRUE(tl::math::NativeSymmetricEigen3(near.tensor, result));
  for (unsigned k = 0; k < 3; ++k) EXPECT_TRUE(SameBits(result.value[k], near.tensor[k]));
  EXPECT_LT(result.value[1], result.value[0]);
  EXPECT_GT(result.value[2], result.value[0]); // Deliberately not sorted.
  EXPECT_EQ(Residual(near, result), 8e-13); // Native ignores this tiny shear.
}

TEST(NativeSpectrumHost, RotatedDistinctReconstructionAndCompressionOrder) {
  bool compression = false;
  for (unsigned n = 0; n < 40; ++n) {
    const auto packet = Rotated(.4, .11, -.63, .07*n, -.51+.03*n);
    Spectrum result;
    detail::Work work;
    ASSERT_TRUE(detail::Roots(packet.tensor, work, result));
    compression = compression || work.compression_swap;
    ASSERT_TRUE(tl::math::NativeSymmetricEigen3(packet.tensor, result));
    EXPECT_NEAR(result.value[0], .4, 2e-15);
    EXPECT_NEAR(result.value[1], .11, 2e-15);
    EXPECT_NEAR(result.value[2], -.63, 2e-15);
    EXPECT_LT(Residual(packet, result), 2e-15);
    for (unsigned c = 0; c < 3; ++c) {
      for (unsigned d = 0; d < 3; ++d) {
        long double dot = 0;
        for (unsigned r = 0; r < 3; ++r)
          dot += static_cast<long double>(result.vectors.v[3*r+c])*result.vectors.v[3*r+d];
        EXPECT_NEAR(static_cast<double>(dot), c == d ? 1 : 0, 2e-15);
      }
    }
  }
  EXPECT_TRUE(compression);
}

TEST(NativeSpectrumHost, StrictThresholdTiesAndRepeatedBasisAreObservable) {
  const double tied[3]{2, 2, 2};
  const double later[3]{1, 2, 2};
  EXPECT_EQ(detail::LargestColumn(tied), 0u);
  EXPECT_EQ(detail::LargestColumn(later), 1u);
  const auto cases = Cases();
  unsigned near = 0, distinct = 0, repeated = 0;
  for (const auto& packet : cases) {
    Spectrum result;
    detail::Work work;
    ASSERT_TRUE(detail::Roots(packet.tensor, work, result));
    switch (detail::Path(work)) {
      case detail::DirectionPath::NearTriple: ++near; break;
      case detail::DirectionPath::Distinct: ++distinct; break;
      case detail::DirectionPath::Repeated: ++repeated; break;
    }
    ASSERT_TRUE(tl::math::NativeSymmetricEigen3(packet.tensor, result));
  }
  EXPECT_GT(near, 0u);
  EXPECT_GT(distinct, 0u);
  EXPECT_GT(repeated, 0u);

  const auto packet = Diagonal(.25, .25, -.5);
  Spectrum native;
  ASSERT_TRUE(tl::math::NativeSymmetricEigen3(packet.tensor, native));
  Spectrum alternative = native;
  const double c = std::sqrt(.5);
  for (unsigned r = 0; r < 3; ++r) {
    alternative.vectors.v[3*r] = c*native.vectors.v[3*r] + c*native.vectors.v[3*r+1];
    alternative.vectors.v[3*r+1] = -c*native.vectors.v[3*r] + c*native.vectors.v[3*r+1];
  }
  EXPECT_LT(Residual(packet, alternative), 2e-8); // Same degenerate eigenspace.
  double original[15], changed[15];
  Pack(native, packet.rate, original);
  Pack(alternative, packet.rate, changed);
  EXPECT_GT(std::abs(original[12]-changed[12]), .1);
  const double original_rate_norm = std::sqrt(original[12]*original[12] +
      original[13]*original[13] + original[14]*original[14]);
  const double changed_rate_norm = std::sqrt(changed[12]*changed[12] +
      changed[13]*changed[13] + changed[14]*changed[14]);
  EXPECT_GT(std::abs(original_rate_norm-changed_rate_norm), .1);
  EXPECT_FALSE(Agree(changed, original, packet));
}

TEST(NativeSpectrumHost, LateInputAndArithmeticFailurePreserveAndRetry) {
  const auto valid = Rotated(.2, -.1, -.6, .43, -.27);
  Spectrum result;
  ASSERT_TRUE(tl::math::NativeSymmetricEigen3(valid.tensor, result));
  const Spectrum before = result;
  auto invalid = valid;
  invalid.tensor[5] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(tl::math::NativeSymmetricEigen3(invalid.tensor, result));
  EXPECT_EQ(std::memcmp(&result, &before, sizeof(result)), 0);
  invalid = Diagonal(1e200, -1e200, 1e200);
  EXPECT_FALSE(tl::math::NativeSymmetricEigen3(invalid.tensor, result));
  EXPECT_EQ(std::memcmp(&result, &before, sizeof(result)), 0);
  ASSERT_TRUE(tl::math::NativeSymmetricEigen3(valid.tensor, result));
  double actual[15], expected[15];
  Pack(result, valid.rate, actual);
  Pack(before, valid.rate, expected);
  for (unsigned k = 0; k < 15; ++k) EXPECT_TRUE(SameBits(actual[k], expected[k]));
}
} // namespace spectrum_test
