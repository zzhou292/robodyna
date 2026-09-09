#include "ShellPhaseNative.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <mutex>
#include <type_traits>

extern "C" int crash_shell_phase_native_impl(
    int, int, const double*, const double*, const double*, const double*,
    const double*, const double*, double*);

namespace {
std::mutex phase_mutex;

template <std::size_t N> bool Finite(const double (&a)[N]) {
    return std::all_of(a, a + N, [](double x) { return std::isfinite(x); });
}
double Dot(const double* a, const double* b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
bool ProperFrame(const double* f) {
    for (int i=0; i<3; ++i)
        for (int j=0; j<3; ++j)
            if (std::abs(Dot(f+3*i, f+3*j) - (i==j ? 1.0 : 0.0)) > 1e-11)
                return false;
    const double cross[3] = {f[1]*f[5]-f[2]*f[4], f[2]*f[3]-f[0]*f[5],
                             f[0]*f[4]-f[1]*f[3]};
    return Dot(cross, f+6) > 1.0 - 1e-11;
}
// Independent planar polygon admission, not the donor's area floor. The
// quarter-step CDEFO3 branch also requires its two denominators away from zero.
bool Polygon(const double* xy) {
    double scale=0.0;
    for (int i=0; i<8; ++i) scale=std::max(scale,std::abs(xy[i]));
    if (!(scale>1e-12) || !std::isfinite(scale*scale)) return false;
    const double floor=1e-12*scale*scale;
    double area2=0;
    for (int i=0; i<4; ++i) {
        const int j=(i+1)%4, k=(i+2)%4;
        const double cross=(xy[2*j]-xy[2*i])*(xy[2*k+1]-xy[2*j+1])
                          -(xy[2*j+1]-xy[2*i+1])*(xy[2*k]-xy[2*j]);
        if (!(cross>floor) || !std::isfinite(cross)) return false;
        area2+=xy[2*i]*xy[2*j+1]-xy[2*i+1]*xy[2*j];
    }
    return std::isfinite(area2) && area2>floor &&
           std::abs(xy[6]-xy[2]-xy[4])>1e-12*scale &&
           std::abs(xy[5]-xy[3]+xy[7])>1e-12*scale;
}
bool Valid(const ShellPhaseInput& a, const ShellPhaseState& s) {
    if ((a.ismstr!=1 && a.ismstr!=2) || (a.ithk!=0 && a.ithk!=1) ||
        (s.off!=1.0 && s.off!=2.0) || !Finite(a.x) || !Finite(a.v) ||
        !Finite(a.omega) || !Finite(a.frame) || !Finite(a.h) || !Finite(a.srh) ||
        !Finite(s.smstr) || !Finite(s.gstr) || !ProperFrame(a.frame)) return false;
    const double scalars[]={a.dt,a.section_thickness,a.current_thickness,a.young,
                            a.poisson,a.density,a.sound_speed,a.shear_factor};
    if (!Finite(scalars) || !(a.dt>0) || !(a.section_thickness>0) ||
        !(a.current_thickness>0) || !(a.young>0) || !(a.poisson>-1) ||
        !(a.poisson<0.5) || !(a.density>0) || !(a.sound_speed>0) ||
        !(a.shear_factor>0)) return false;
    for (double x:a.h) if (x<0) return false;
    for (double x:a.srh) if (x<0) return false;
    if (!std::isfinite(a.young/(1-a.poisson*a.poisson)) ||
        !std::isfinite(a.young/(2*(1+a.poisson)))) return false;
    double xy[8]={}, distance[4]={}, length=0;
    for (int i=0;i<4;++i) {
        const double d[3]={a.x[3*i]-a.x[0],a.x[3*i+1]-a.x[1],a.x[3*i+2]-a.x[2]};
        xy[2*i]=Dot(d,a.frame); xy[2*i+1]=Dot(d,a.frame+3);
        distance[i]=Dot(d,a.frame+6);
        length=std::max(length,std::hypot(xy[2*i],xy[2*i+1]));
    }
    if (!Finite(xy) || !Finite(distance) || !Polygon(xy)) return false;
    for (double d:distance) if (std::abs(d)>1e-10*length) return false;
    if (s.off==2) {
        const double stored[8]={0,0,s.smstr[0],s.smstr[1],s.smstr[2],
                                 s.smstr[3],s.smstr[4],s.smstr[5]};
        if (!Polygon(stored)) return false;
    }
    return true;
}
} // namespace

static_assert(std::is_standard_layout<ShellPhaseState>::value);
static_assert(std::is_standard_layout<ShellPhaseOutput>::value);
static_assert(sizeof(ShellPhaseState)==15*sizeof(double));
static_assert(sizeof(ShellPhaseOutput)==73*sizeof(double));

extern "C" int crash_shell_phase_native(int n, const ShellPhaseInput* input,
                                         const ShellPhaseState* accepted,
                                         ShellPhaseOutput* trial) {
    if (n<1 || n>2 || !input || !accepted || !trial) return 1;
    for (int i=0;i<n;++i) if (!Valid(input[i],accepted[i])) return 1;
    std::array<ShellPhaseOutput,2> staged{};
    std::lock_guard<std::mutex> lock(phase_mutex);
    for (int i=0;i<n;++i) {
        const auto& a=input[i];
        const double cfg[14]={a.dt,a.section_thickness,a.current_thickness,a.young,
            a.poisson,a.density,a.sound_speed,a.shear_factor,
            a.h[0],a.h[1],a.h[2],a.srh[0],a.srh[1],a.srh[2]};
        double state[15], result[73];
        std::memcpy(state,&accepted[i],sizeof state);
        const int status=crash_shell_phase_native_impl(a.ismstr,a.ithk,a.x,a.v,
            a.omega,a.frame,cfg,state,result);
        if (status!=0 || !Finite(result)) return 2;
        std::memcpy(&staged[i],result,sizeof result);
        const auto& out=staged[i];
        if (!(out.area>0) || !(out.thk0>0) || out.gathered_off!=1.0 ||
            out.state.off!=(a.ismstr==1 ? 2.0 : accepted[i].off)) return 2;
    }
    std::copy_n(staged.data(),n,trial);
    return 0;
}
