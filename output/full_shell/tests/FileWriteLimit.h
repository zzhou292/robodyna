#pragma once
#include "output/ArtifactIO.h"
#include <csignal>
#include <sys/resource.h>

namespace crash::output::full_shell::test {
// Serial test-only fault injection. Restore both inherited limits and signals
// even when the production create-only write reports an exception.
class FileSizeLimit {
  public:
    explicit FileSizeLimit(rlim_t bytes = 100) {
        Require(getrlimit(RLIMIT_FSIZE, &old_) == 0, "Cannot inspect test file limit");
        struct sigaction ignore{};
        ignore.sa_handler = SIG_IGN;
        sigemptyset(&ignore.sa_mask);
        Require(sigaction(SIGXFSZ, &ignore, &signal_) == 0, "Cannot stage test signal");
        auto next = old_;
        next.rlim_cur = bytes;
        if (setrlimit(RLIMIT_FSIZE, &next) != 0) {
            sigaction(SIGXFSZ, &signal_, nullptr);
            throw std::runtime_error("Cannot stage test file limit");
        }
    }
    ~FileSizeLimit() { setrlimit(RLIMIT_FSIZE, &old_); sigaction(SIGXFSZ, &signal_, nullptr); }
  private:
    struct rlimit old_{};
    struct sigaction signal_{};
};
} // namespace crash::output::full_shell::test
