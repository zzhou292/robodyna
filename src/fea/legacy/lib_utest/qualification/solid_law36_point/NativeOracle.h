#pragma once
#include "TestSupport.h"
namespace law36_test {
std::array<double,20> Native(const law::Parameters&,const law::History&,
                            const law::Input&,int defaults=1);
std::array<double,26> Native(const law::Parameters&,const law::CallerHistory&,
                            const law::Kinematics&,const law::Measures&,int defaults=1);
law::History NativeHistory(const std::array<double,20>&);
law::CallerHistory NativeHistory(const std::array<double,26>&);
}
