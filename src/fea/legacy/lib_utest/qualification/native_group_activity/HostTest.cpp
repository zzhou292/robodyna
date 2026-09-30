#include "lib_src/collision/RadiossType25Transaction.h"
#include "lib_src/elements/publication/PhysicalActivePrefix.h"
#include <gtest/gtest.h>
#include <type_traits>
namespace {
  template<class T,class=void>struct PublicSession:std::false_type{};
  template<class T>struct PublicSession<T,std::void_t<typename T::PreparedGroupSession>>:std::true_type{};
  template<class T,class=void>struct PublicFastCheck:std::false_type{};
  template<class T>struct PublicFastCheck<T,std::void_t<decltype(&T::CheckPreparedWithinGroup)>>:std::true_type{};
  TEST(NativeGroupedActivityHost, NoConstructiblePublicProofOrFastCheck){
    EXPECT_FALSE(PublicSession<tl::fea::PhysicalActivePrefix>::value);
    EXPECT_FALSE(PublicFastCheck<tl::fea::PhysicalActivePrefix>::value);
    EXPECT_EQ(sizeof(tl::fea::PhysicalActivePrefix),sizeof(void*));
    EXPECT_EQ(tl::fea::MaxNativeContactInterfaces,2u);
  }

  TEST(NativeGroupedActivityHost, InvalidBoundedDescriptorNeedsNoLiveOwner){
    namespace n=tlfea::contact::radioss_type25;
    tl::fea::FENodalState owner;
    tl::fea::ShellBatchPublication publication;
    tl::fea::NodalTrialToken token;
    tl::fea::NodalPreparedView view;
    tl::fea::ShellPhysicalDiagnostics d;
    tl::fea::ShellPhysicalScratchParticipationReceipt receipt;
    n::Transaction* members[]{
      nullptr,nullptr
    };
    for(std::size_t count:{0u,3u}){
      const auto r=n::Transaction::SealCandidateGroup(members,count,publication,owner,token,view,d,&receipt,1);
      EXPECT_EQ(r.report.status,n::TransactionStatus::InvalidInput);
      EXPECT_EQ(r.interface_index,SIZE_MAX);
      EXPECT_FALSE(receipt.valid());
    }
  }
}
