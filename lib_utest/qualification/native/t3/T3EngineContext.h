#pragma once
#include <mutex>

namespace tl::qualification::t3::detail {
// All later T3 engine/history calls share this lock and the same privately
// renamed native modules/COMMON blocks. R1 startup has a separate context.
std::mutex& NativeEngineContext();
inline constexpr unsigned kKinematicsValues=38;
extern "C" void t3_r2_kinematics(const double*,const double*,const double*,
                                  const double*,double*,int*);
}  // namespace tl::qualification::t3::detail
