// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <string>

namespace {

TEST(PhysicalScratchParticipationSource, BazelRunsOwningSourceProof) {
  const char* source_root=std::getenv("TEST_SRCDIR");
  const char* workspace=std::getenv("TEST_WORKSPACE");
  ASSERT_NE(source_root,nullptr);
  ASSERT_NE(workspace,nullptr);
  const auto script=std::filesystem::path(source_root)/workspace/
      "lib_utest/qualification/physical_publication/"
      "verify_participation_sources.py";
  ASSERT_TRUE(std::filesystem::is_regular_file(script));
  const std::string command="/usr/bin/python3 -B \""+script.string()+"\"";
  EXPECT_EQ(std::system(command.c_str()),0);
}

} // namespace
