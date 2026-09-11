#pragma once
#include "TestSupport.h"

namespace law44_solid_test {
struct NativeResult {
  law::Result result{};
  std::array<double,26> prepared{};  // Native working units, unmodified UPARAM.
};
NativeResult Native(const law::Parameters&,const law::History&,const law::Input&);
void Compare(const law::Result&,const law::Result&,const law::Parameters&,
             const law::History&,const law::Input&);
}
