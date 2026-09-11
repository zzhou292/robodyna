#include <iomanip>
#include "NativeSupport.h"

namespace spectrum_test {
TEST(NativeSpectrumReference, NativeRealPrecisionAndWorkingConstants) {
  double values[4];
  spectrum_constants(values);
  EXPECT_TRUE(SameBits(values[0], static_cast<double>(2.2e-16f)));
  EXPECT_TRUE(SameBits(values[1], detail::NativePrecisionFactor()));
  EXPECT_TRUE(SameBits(values[2], 1.0/1e10));
  EXPECT_TRUE(SameBits(values[3], 1.0/1e20));
  EXPECT_NE(values[1], 2*std::sqrt(std::numeric_limits<double>::epsilon()));
  EXPECT_NE(values[1], 2*std::sqrt(static_cast<double>(2.2e-16f)));
}

TEST(NativeSpectrumReference, AllOrderedDirectionsAndLaw90ProjectedRates) {
  const auto packets = Cases();
  const auto native = Native(packets);
  ASSERT_EQ(native.size(), packets.size());
  for (std::size_t i = 0; i < packets.size(); ++i) {
    Spectrum result;
    ASSERT_TRUE(tl::math::NativeSymmetricEigen3(packets[i].tensor, result)) << i;
    double values[15];
    Pack(result, packets[i].rate, values);
    ASSERT_TRUE(Compare(values, native[i].data(), packets[i])) << "packet " << i;
    auto changed = native[i];
    changed[3] += 1e-6;
    EXPECT_FALSE(Agree(values, changed.data(), packets[i]));
    changed = native[i];
    changed[12] += 1e-6;
    EXPECT_FALSE(Agree(values, changed.data(), packets[i]));
  }
}

TEST(NativeSpectrumReference, CompleteNativeVectorQueuesMatchScalarCalls) {
  const auto packets = Cases();
  const auto batched = Native(packets);
  ASSERT_EQ(batched.size(), packets.size());
  for (std::size_t i = 0; i < packets.size(); ++i) {
    const auto scalar = Native({packets[i]});
    ASSERT_EQ(scalar.size(), 1u);
    for (unsigned k = 0; k < 15; ++k)
      ASSERT_TRUE(SameBits(scalar[0][k], batched[i][k])) << "packet " << i << " field " << k;
  }
  double input[6]{}, values[15];
  std::fill_n(values, 15, 17.);
  for (int count : {0, 130}) {
    int status = -1;
    spectrum_native(input, input, &count, values, &status);
    ASSERT_EQ(status, 1);
    for (double value : values) EXPECT_EQ(value, 17.);
  }
}
} // namespace spectrum_test
