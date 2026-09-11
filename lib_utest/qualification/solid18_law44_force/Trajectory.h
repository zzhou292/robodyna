#pragma once
#include "TestSupport.h"
namespace rear_force_test {
// Prescribed endpoint/midpoint packets; native and host receive identical data.
inline law::PrescribedInterval Path(const law::Reference& reference, unsigned step, bool rotate = false) {
  constexpr double dt = 1e-5, frequency = .12;
  const double end = (step+1)*dt, mid = (step+.5)*dt;
  const double amplitude = .025*std::sin(frequency*(step+1));
  const double rate = .025*frequency/dt*std::cos(frequency*(step+.5));
  const double spin = rotate ? 20 : 0;
  const double c = std::cos(spin*end), s = std::sin(spin*end);
  const double cm = std::cos(spin*mid), sm = std::sin(spin*mid);
  const double am = .025*std::sin(frequency*(step+.5));
  law::PrescribedInterval interval;
  interval.dt_s = dt;
  interval.base_time_s = step*dt;
  interval.sample_index = step+1;
  const auto origin = reference.input().position_m[0];
  for (unsigned n = 0; n < 8; ++n) {
    const auto p = reference.input().position_m[n];
    const s::Vec3 x{p.x-origin.x,p.y-origin.y,p.z-origin.z};
    const s::Vec3 shape{.3*x.x+x.y,-.2*x.y+.2*x.z,-.1*x.z+.1*x.x};
    const s::Vec3 pe{x.x+amplitude*shape.x,x.y+amplitude*shape.y,x.z+amplitude*shape.z};
    const s::Vec3 pm{x.x+am*shape.x,x.y+am*shape.y,x.z+am*shape.z};
    interval.position_endpoint_m[n] = {origin.x+c*pe.x-s*pe.y,
                                      origin.y+s*pe.x+c*pe.y,origin.z+pe.z};
    interval.velocity_midpoint_m_s[n] = {
        cm*rate*shape.x-sm*rate*shape.y-spin*(sm*pm.x+cm*pm.y),
        sm*rate*shape.x+cm*rate*shape.y+spin*(cm*pm.x-sm*pm.y),rate*shape.z};
  }
  return interval;
}
}
