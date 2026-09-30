// SPDX-License-Identifier: AGPL-3.0-or-later
#include <gtest/gtest.h>
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <sys/wait.h>
#include <unistd.h>
TEST(InitialStateSource, WholeOriginalBodiesAndDeclaredNamesOnlyAreVerified) {
  const auto* root=std::getenv("TEST_SRCDIR");const auto* workspace=std::getenv("TEST_WORKSPACE");
  const auto* tmp=std::getenv("TEST_TMPDIR");
  ASSERT_NE(root,nullptr);
  ASSERT_NE(workspace,nullptr);
  ASSERT_NE(tmp,nullptr);
  const auto tree=std::filesystem::path(root)/workspace;
  const auto script=tree/"lib_utest/qualification/radioss_type25_initial_state/native/prepare.py";
  const auto output=std::filesystem::path(tmp)/"native-initial-state";
  ASSERT_TRUE(std::filesystem::is_regular_file(script));
  const auto child=fork();
  ASSERT_NE(child,-1);
  if(child==0) {
    execl("/usr/bin/python3","python3","-B",script.c_str(),"--tl-root",tree.c_str(),"--output",output.c_str(),static_cast<char*>(nullptr));
    _exit(127);
  }
  int status=0;pid_t waited;
  do{waited=waitpid(child,&status,0);}while(waited<0&&errno==EINTR);
  ASSERT_EQ(waited,child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status),0);
}
