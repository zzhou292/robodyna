#include "UniformTranslationKinetic.h"
#include <cmath>

namespace crash::cases::wall_penalty {
namespace qb=contact::q4_bounds;
bool BeginUniformTranslation(double speed,UniformTranslationKinetic* output) noexcept {
    if(!output||!std::isfinite(speed)||speed<=0)return false;
    UniformTranslationKinetic next;
    if(!qb::MultiplyPositive({speed,speed},{speed,speed},&next.speed_squared))return false;
    next.nominal_speed_squared=speed*speed;
    if(!std::isfinite(next.nominal_speed_squared))return false;
    *output=next;return true;
}
bool AddTranslationMass(double mass,UniformTranslationKinetic* output) noexcept {
    if(!output||!std::isfinite(mass)||mass<=0||output->mass_count==SIZE_MAX||
       !std::isfinite(output->nominal_speed_squared)||output->nominal_speed_squared<=0)return false;
    auto next=*output;qb::Interval term;
    if(!qb::Scale(next.speed_squared,mass,&term)||!qb::Scale(term,.5,&term)||!qb::Add(next.energy,term,&next.energy))return false;
    // Preserve the legacy source-part nominal expression and input order.
    next.nominal_energy+=.5*mass*next.nominal_speed_squared;
    if(!std::isfinite(next.nominal_energy))return false;
    ++next.mass_count;*output=next;return true;
}
bool FinishUniformTranslation(const UniformTranslationKinetic& input,contact::Q4CertifiedIntegral* output) noexcept {
    if(!output||!input.mass_count||input.energy.lower<=0)return false;
    contact::Q4CertifiedIntegral next;
    if(!qb::Certify(input.nominal_energy,input.energy,&next))return false;
    *output=next;return true;
}
} // namespace crash::cases::wall_penalty
