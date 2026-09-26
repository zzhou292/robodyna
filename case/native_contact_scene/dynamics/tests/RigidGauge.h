#pragma once
#include <array>
#include <algorithm>
#include <cmath>
namespace crash::cases::native_scene::rigid_trajectory_test::gauge {
using Matrix=std::array<double,9>;
using Moments=std::array<double,3>;
// Exactly the original frame comparison budgets; only the physical observable
// changes from arbitrary eigenvectors to relative world rotation.
constexpr double Absolute=2e-10,Relative=2e-8;
inline bool Near(double a,double b){return std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=Absolute+Relative*std::abs(b);}
inline Matrix Transpose(const Matrix& a){Matrix out{};for(unsigned r=0;r<3;++r)for(unsigned c=0;c<3;++c)out[3*r+c]=a[3*c+r];return out;}
inline Matrix Product(const Matrix& a,const Matrix& b){Matrix out{};for(unsigned r=0;r<3;++r)for(unsigned c=0;c<3;++c)for(unsigned k=0;k<3;++k)out[3*r+c]+=a[3*r+k]*b[3*k+c];return out;}
inline bool Proper(const Matrix& a) {
    for(double x:a)if(!std::isfinite(x))return false;
    const auto gram=Product(Transpose(a),a);
    for(unsigned r=0;r<3;++r)for(unsigned c=0;c<3;++c)
        if(!Near(gram[3*r+c],r==c?1.:0.))return false;
    const double det=a[0]*(a[4]*a[8]-a[5]*a[7])-a[1]*(a[3]*a[8]-a[5]*a[6])+a[2]*(a[3]*a[7]-a[4]*a[6]);
    return Near(det,1.);
}
inline Matrix WorldInertia(const Matrix& axes,const Moments& moments) {
    Matrix out{};
    for(unsigned r=0;r<3;++r)for(unsigned c=0;c<3;++c)for(unsigned k=0;k<3;++k)
        out[3*r+c]+=(axes[3*r+k]*moments[k])*axes[3*c+k];
    return out;
}
struct Alignment {Matrix actual_initial{},native_initial{};};
inline bool Bind(const Matrix& a,const Moments& ja,const Matrix& b,const Moments& jb,Alignment* output) {
    if(!output||!Proper(a)||!Proper(b))return false;
    double scale=0.;
    for(unsigned k=0;k<3;++k) {
        if(!std::isfinite(ja[k])||!std::isfinite(jb[k])||ja[k]<=0||jb[k]<=0)return false;
        scale=std::max(scale,std::max(ja[k],jb[k]));
    }
    const auto x=WorldInertia(a,ja),y=WorldInertia(b,jb);
    // Dimensionless normalization avoids a large fixed absolute tolerance on
    // tiny kg*m^2 tensors. It preserves the original frame tolerance budget.
    for(unsigned i=0;i<9;++i)if(!Near(x[i]/scale,y[i]/scale))return false;
    *output={a,b};return true;
}
inline Matrix Rotation(const Matrix& current,const Matrix& initial){return Product(current,Transpose(initial));}
inline bool SameRotation(const Alignment& initial,const Matrix& a,const Matrix& b) {
    if(!Proper(a)||!Proper(b))return false;
    const auto x=Rotation(a,initial.actual_initial),y=Rotation(b,initial.native_initial);
    for(unsigned i=0;i<9;++i)if(!Near(x[i],y[i]))return false;
    return true;
}
}
