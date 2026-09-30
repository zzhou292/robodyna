// SPDX-License-Identifier: AGPL-3.0-or-later
#include <gtest/gtest.h>
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <sys/wait.h>
#include <unistd.h>

TEST(SelfContactFilterBatchSource, QualifiedBodiesAndBoundedOwnerRemainIntact) {
  const auto* root = std::getenv("TEST_SRCDIR");
  const auto* workspace = std::getenv("TEST_WORKSPACE");
  ASSERT_NE(root, nullptr); ASSERT_NE(workspace, nullptr);
  const auto script = std::filesystem::path(root) / workspace /
      "lib_utest/qualification/self_contact_filter_batch/verify_extraction.py";
  ASSERT_TRUE(std::filesystem::is_regular_file(script));
  const auto child = fork();
  ASSERT_GE(child, 0);
  if (!child) {
    execl("/usr/bin/python3", "/usr/bin/python3", "-B", script.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }
  int status = 0;
  pid_t result;
  do { result = waitpid(child, &status, 0); } while (result < 0 && errno == EINTR);
  ASSERT_EQ(result, child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
}
