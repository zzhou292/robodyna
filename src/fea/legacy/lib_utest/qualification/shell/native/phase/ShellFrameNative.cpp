#include "ShellFrameNative.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <mutex>
#include <type_traits>

extern "C" int crash_shell_frame_native_impl(int, const double*, double*);

namespace {
using Vec = std::array<double,3>;
std::mutex frame_mutex;
Vec Sub(const Vec& a,const Vec& b) { return {a[0]-b[0],a[1]-b[1],a[2]-b[2]}; }
Vec Cross(const Vec& a,const Vec& b) {
    return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
}
double Dot(const Vec& a,const Vec& b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
bool Valid(const ShellFrameInput& a) {
    for(double x:a.x) if(!std::isfinite(x)) return false;
    std::array<Vec,4> p{};
    double scale=0;
    for(int i=1;i<4;++i) for(int j=0;j<3;++j) {
        p[i][j]=a.x[3*i+j]-a.x[j];
        if(!std::isfinite(p[i][j])) return false;
        scale=std::max(scale,std::abs(p[i][j]));
    }
    if(!(scale>0)) return false;
    for(auto& v:p) for(double& x:v) x/=scale;
    Vec normal=Cross(p[1],p[2]);
    const double length=std::hypot(normal[0],normal[1],normal[2]);
    if(!(length>1e-12)) return false;
    for(double& x:normal) x/=length;
    if(std::abs(Dot(normal,p[3]))>1e-10) return false;
    for(int i=0;i<4;++i) {
        const auto u=Sub(p[(i+1)%4],p[i]);
        const auto v=Sub(p[(i+2)%4],p[(i+1)%4]);
        if(!(Dot(Cross(u,v),normal)>1e-12)) return false;
    }
    return true;
}
bool Proper(const double* frame) {
    Vec e[3];
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) {
        e[i][j]=frame[3*i+j];
        if(!std::isfinite(e[i][j])) return false;
    }
    for(int i=0;i<3;++i) for(int j=0;j<3;++j)
        if(std::abs(Dot(e[i],e[j])-(i==j?1.:0.))>1e-11) return false;
    return Dot(Cross(e[0],e[1]),e[2])>1.-1e-11;
}
} // namespace

static_assert(std::is_standard_layout<ShellFrameInput>::value);
static_assert(std::is_standard_layout<ShellFrameOutput>::value);
static_assert(sizeof(ShellFrameInput)==12*sizeof(double));
static_assert(sizeof(ShellFrameOutput)==13*sizeof(double));

extern "C" int crash_shell_frame_native(int n,const ShellFrameInput* input,
                                        ShellFrameOutput* output) {
    if(n<1 || n>2 || !input || !output) return 1;
    for(int i=0;i<n;++i) if(!Valid(input[i])) return 1;
    double packed[24]={},result[26]={};
    std::memcpy(packed,input,n*sizeof(ShellFrameInput));
    std::lock_guard<std::mutex> lock(frame_mutex);
    if(crash_shell_frame_native_impl(n,packed,result)!=0) return 2;
    std::array<ShellFrameOutput,2> staged{};
    for(int i=0;i<n;++i) {
        const double* r=result+13*i;
        if(!Proper(r) || r[9]!=17.25 || r[10]!=17.25 || r[11]!=-9.5 || r[12]!=-9.5)
            return 2;
        std::memcpy(&staged[i],r,sizeof(ShellFrameOutput));
    }
    std::copy_n(staged.data(),n,output);
    return 0;
}
