#pragma once

#include "ElasticCouponModel.h"
// Chrono installs its Eigen extensions here before Eigen is first included.
// Keep the same matrix definitions in the audit and owning FEA translation units.
#include "chrono/core/ChMatrix.h"

namespace crash::reference::patch_audit {

// Shared host oracle tools for the two declared six-node shell fixtures. The
// only instantiated coordinate counts are 16 and 24; no dynamics or generic
// vehicle model is introduced. Components 0..2 are world x/y/z, 3..5 spins.
struct Coordinate { std::size_t node = 0; unsigned component = 0; };
template<std::size_t N> using Layout = std::array<Coordinate,N>;
template<std::size_t N> using Matrix = Eigen::Matrix<double,N,N>;
template<std::size_t N> using Vector = Eigen::Matrix<double,N,1>;
enum class ForceSource { Chrono, TL };

inline constexpr double TranslationDifference = 1e-6 * ElasticCouponData::length;
inline constexpr double RotationDifference = 1e-6;
inline constexpr double DerivativeTolerance = 1e-5;
inline constexpr double FrequencyRefinementTolerance = .005;

Layout<kCouponFreeDofs> FullCouponLayout();
ElasticCouponStatus Reject(const char* gate,double measured,double limit,std::string& diagnostic);
// Staged publication, including invalid late entries; base/output may alias.
ElasticCouponStatus ApplyIncrement(const ElasticCouponConfiguration& base,const Coordinate* coordinates,
                                   std::size_t count,const double* increment,double scale,
                                   ElasticCouponConfiguration& output,std::string& diagnostic);

template<std::size_t N> Vector<N> InverseRootMass(const ElasticCouponData&,const Layout<N>&);
template<std::size_t N> Matrix<N> MassScale(const Matrix<N>&,const Vector<N>&);
template<std::size_t N> double OperatorNorm(const Matrix<N>&);
template<std::size_t N> ElasticCouponStatus DifferenceJacobian(
    const ElasticCouponModel&,const ElasticCouponConfiguration&,const Layout<N>&,ForceSource,double difference_scale,
    Matrix<N>& output,std::string& diagnostic);
template<std::size_t N> ElasticCouponStatus DirectionalCrossCheck(
    const ElasticCouponModel&,const ElasticCouponConfiguration&,const Layout<N>&,
    const Matrix<N>& chrono_stiffness,const Vector<N>& inverse_root_mass,double& worst,std::string& diagnostic);

template<std::size_t N> struct ReferenceSpectrum {
    Matrix<N> stiffness;
    Vector<N> inverse_root_mass,squared_frequency;
    Matrix<N> mass_modes;
    double symmetry_error=0,derivative_refinement_error=0,tl_derivative_error=0;
    double maximum_frequency_refinement_error=0,eigen_residual=0;
};
// Actual coherent-Chrono and TL force differences, per-column halving/parity,
// stress-free symmetry, positive spectrum above derivative uncertainty and
// eigen residual. Only stress-free spectral extraction symmetrizes its matrix.
template<std::size_t N> ElasticCouponStatus AuditReference(
    const ElasticCouponModel&,const Layout<N>&,ReferenceSpectrum<N>& output,std::string& diagnostic);

}  // namespace crash::reference::patch_audit
