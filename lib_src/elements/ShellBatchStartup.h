// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../math/Fixed3.h"
#include "../math/Quaternion.h"
#if defined(__CUDACC__)
#define TL_SHELL_STARTUP_HD __host__ __device__
#else
#define TL_SHELL_STARTUP_HD
#endif

namespace tl::fea {
enum class ShellBatchStartupKind { ReferenceRest,ReferenceUniformTranslation };
struct ShellBatchStartup {
  ShellBatchStartupKind kind=ShellBatchStartupKind::ReferenceRest;
  tl::math::Vec3 uniform_velocity{}; // Physical common WORLD velocity, m/s.
};
namespace shell_startup_detail {
TL_SHELL_STARTUP_HD inline bool SameBits(double a,double b) noexcept {
  const auto* x=reinterpret_cast<const unsigned char*>(&a);
  const auto* y=reinterpret_cast<const unsigned char*>(&b);
  for(unsigned i=0;i<sizeof(double);++i) if(x[i]!=y[i]) return false;
  return true;
}
TL_SHELL_STARTUP_HD inline bool SameVector(tl::math::Vec3 a,tl::math::Vec3 b) noexcept {
  return SameBits(a.x,b.x)&&SameBits(a.y,b.y)&&SameBits(a.z,b.z);
}
TL_SHELL_STARTUP_HD inline bool SameStartup(const ShellBatchStartup& a,const ShellBatchStartup& b) noexcept {
  if(a.kind!=b.kind) return false;
  if(a.kind==ShellBatchStartupKind::ReferenceRest) {
    const auto x=a.uniform_velocity,y=b.uniform_velocity;
    return x.x==0&&x.y==0&&x.z==0&&y.x==0&&y.y==0&&y.z==0;
  }
  return a.kind==ShellBatchStartupKind::ReferenceUniformTranslation&&SameVector(a.uniform_velocity,b.uniform_velocity);
}
inline bool ValidStartup(const ShellBatchStartup& s,bool coupled) noexcept {
  const auto v=s.uniform_velocity;
  if(!tl::math::Finite(v.x)||!tl::math::Finite(v.y)||!tl::math::Finite(v.z)) return false;
  if(s.kind==ShellBatchStartupKind::ReferenceRest) return v.x==0&&v.y==0&&v.z==0;
  return s.kind==ShellBatchStartupKind::ReferenceUniformTranslation&&coupled;
}
// Caller checks finite fields/unit quaternion and free native m/J separately.
// Rest retains numeric-zero/unit-q behavior. Explicit translation binds velocity
// bits and identity q, without running a dt0 material or force operation.
TL_SHELL_STARTUP_HD inline bool MatchesInitialNode(const ShellBatchStartup& s,tl::math::Vec3 x,
    tl::math::Vec3 reference,tl::math::Vec3 velocity,tl::math::Vec3 omega,const double* q) noexcept {
  if(!SameVector(x,reference)||omega.x!=0||omega.y!=0||omega.z!=0) return false;
  if(s.kind==ShellBatchStartupKind::ReferenceRest)
    return velocity.x==0&&velocity.y==0&&velocity.z==0;
  return s.kind==ShellBatchStartupKind::ReferenceUniformTranslation&&
    SameVector(velocity,s.uniform_velocity)&&q[0]==1&&q[1]==0&&q[2]==0&&q[3]==0;
}
// Same binary64 node order and scalar arithmetic as native kinetic diagnostics.
// Host metadata preflight admits the operation domain; measured K0 comes from
// actual authenticated owner fields. Failure preserves the partial sum.
TL_SHELL_STARTUP_HD inline bool AddInitialTranslationKinetic(double mass,tl::math::Vec3 velocity,double& kinetic) noexcept {
  const double square=velocity.x*velocity.x+velocity.y*velocity.y+velocity.z*velocity.z;
  const double term=.5*mass*square,next=kinetic+term;
  if(!tl::math::Finite(square)||square<0||!tl::math::Finite(term)||term<0||
     !tl::math::Finite(next)||next<0) return false;
  kinetic=next; return true;
}
} // namespace shell_startup_detail
} // namespace tl::fea
#undef TL_SHELL_STARTUP_HD
