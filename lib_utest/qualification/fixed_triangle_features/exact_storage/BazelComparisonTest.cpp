// SPDX-License-Identifier: AGPL-3.0-or-later
#include <gtest/gtest.h>
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <sys/wait.h>
#include <unistd.h>

TEST(ExactStorageDiscovery, CompleteAdaptiveAndWidePublicationsMatch) {
  const auto* source = std::getenv("TEST_SRCDIR");
  const auto* workspace = std::getenv("TEST_WORKSPACE");
  const auto* temporary = std::getenv("TEST_TMPDIR");
  const auto* retained = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR");
  const auto* output_root = retained && *retained ? retained : temporary;
  ASSERT_NE(source, nullptr); ASSERT_NE(workspace, nullptr); ASSERT_NE(output_root, nullptr);
  const auto directory = std::filesystem::path(source) / workspace /
      "lib_utest/qualification/fixed_triangle_features";
  const auto script = (directory / "exact_storage/compare_discovery.py").string();
  const auto adaptive = (directory / "exact_storage_adaptive_driver").string();
  const auto wide = (directory / "exact_storage_wide_driver").string();
  const auto output = (std::filesystem::path(output_root) / "exact-storage-parity").string();
  const auto child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    execl("/usr/bin/python3", "python3", "-B", script.c_str(),
        "--adaptive", adaptive.c_str(), "--wide", wide.c_str(),
        "--output-dir", output.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }
  int status = 0;
  pid_t waited;
  do { waited = waitpid(child, &status, 0); } while (waited < 0 && errno == EINTR);
  ASSERT_EQ(waited, child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
}
