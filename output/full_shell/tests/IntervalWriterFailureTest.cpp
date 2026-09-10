#include "IntervalTestSupport.h"
#include <csignal>
#include <sys/resource.h>

namespace crash::output::full_shell::test {
namespace {
class FileSizeLimit {
  public:
    FileSizeLimit() {
        Require(getrlimit(RLIMIT_FSIZE,&old_)==0,"Cannot inspect test file limit");
        struct sigaction ignore{};ignore.sa_handler=SIG_IGN;sigemptyset(&ignore.sa_mask);
        Require(sigaction(SIGXFSZ,&ignore,&signal_)==0,"Cannot stage test signal");
        auto next=old_;next.rlim_cur=100;
        if(setrlimit(RLIMIT_FSIZE,&next)!=0) {
            sigaction(SIGXFSZ,&signal_,nullptr);
            throw std::runtime_error("Cannot stage test file limit");
        }
    }
    ~FileSizeLimit(){setrlimit(RLIMIT_FSIZE,&old_);sigaction(SIGXFSZ,&signal_,nullptr);}
  private:
    struct rlimit old_{};
    struct sigaction signal_{};
};
}
TEST(IntervalWriter,PreexistingLateDestinationAndPartialWritePoisonAreCreateOnly) {
    const auto c=Intervals();Directory dir;
    WriteBytes(dir.path/IntervalArrayName("blocked",2,false),"existing");
    EXPECT_THROW(IntervalWriter(dir.path,"blocked",c),std::exception);
    EXPECT_FALSE(std::filesystem::exists(dir.path/IntervalArrayName("blocked",0,true)));
    IntervalWriter late(dir.path,"late",c);Append(late,Row(c,1));
    WriteBytes(dir.path/IntervalArrayName("late",0,false),"new conflict");
    EXPECT_THROW(Append(late,Row(c,2)),std::exception);EXPECT_TRUE(late.failed());
    EXPECT_FALSE(std::filesystem::exists(dir.path/IntervalArrayName("late",0,true)));
    EXPECT_THROW(late.FinishPrefix(),std::exception);
    IntervalWriter partial(dir.path,"partial",c);Append(partial,Row(c,1));
    {
        FileSizeLimit limit;
        EXPECT_THROW(Append(partial,Row(c,2)),std::exception);
    }
    EXPECT_TRUE(partial.failed());
    EXPECT_EQ(std::filesystem::file_size(dir.path/IntervalArrayName("partial",0,true)),64u);
    EXPECT_LT(std::filesystem::file_size(dir.path/IntervalArrayName("partial",0,false)),560u);
    EXPECT_THROW(partial.FinishPrefix(),std::exception);
    EXPECT_THROW(Append(partial,Row(c,3)),std::exception);
    EXPECT_THROW(IntervalWriter(dir.path,"partial",c),std::exception); // Evidence cannot be overwritten.
}
} // namespace crash::output::full_shell::test
