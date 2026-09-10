#pragma once
#include "chrono/assets/ChColor.h"
#include <gtest/gtest.h>
#include <vector>
namespace crash::visual::full_shell::test {
inline bool SameColor(const chrono::ChColor& a,const chrono::ChColor& b) {
    return a.R==b.R&&a.G==b.G&&a.B==b.B;
}
inline void ExpectColor(const chrono::ChColor& a,const chrono::ChColor& b) {
    EXPECT_EQ(a.R,b.R);EXPECT_EQ(a.G,b.G);EXPECT_EQ(a.B,b.B);
}
inline void ExpectColors(const std::vector<chrono::ChColor>& a,const std::vector<chrono::ChColor>& b) {
    ASSERT_EQ(a.size(),b.size());
    for(std::size_t i=0;i<a.size();++i)ASSERT_TRUE(SameColor(a[i],b[i]));
}
} // namespace crash::visual::full_shell::test
