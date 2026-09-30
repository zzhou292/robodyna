#include <gtest/gtest.h>
#include <openssl/crypto.h>
#include <openssl/evp.h>
#include <openssl/opensslv.h>

#include <array>

TEST(OpenSslSdk, LinkedRuntimeMatchesDeclaredHeaders) {
  EXPECT_EQ(OpenSSL_version_num(), OPENSSL_VERSION_NUMBER);
  EXPECT_STREQ(OpenSSL_version(OPENSSL_VERSION), OPENSSL_VERSION_TEXT);
}

TEST(OpenSslSdk, Sha256RetainsKnownArtifactDigest) {
  constexpr std::array<unsigned char, 32> expected{
      0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea,
      0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
      0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c,
      0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad};
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned length = 0;
  ASSERT_EQ(EVP_Digest("abc", 3, digest.data(), &length, EVP_sha256(), nullptr), 1);
  ASSERT_EQ(length, expected.size());
  for (unsigned i = 0; i < length; ++i) EXPECT_EQ(digest[i], expected[i]);
}
