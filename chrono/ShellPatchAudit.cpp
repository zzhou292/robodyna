#include "ShellPatchAudit.h"
#include "math/Quaternion.h"

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace crash::reference::patch_audit {
using Status=ElasticCouponStatus;
namespace tlr=tl::fea::reissner;

Layout<kCouponFreeDofs> FullCouponLayout() {
    Layout<kCouponFreeDofs> result;
    for (std::size_t free=0;free<kCouponFreeNodes.size();++free)
        for (unsigned c=0;c<6;++c) result[6*free+c]={kCouponFreeNodes[free],c};
    return result;
}
Status Reject(const char* gate,double measured,double limit,std::string& diagnostic) {
    std::ostringstream explanation;
    explanation.precision(17);
    explanation<<gate<<": measured "<<measured<<", required limit "<<limit;
    diagnostic=explanation.str();
    return Status::kAuditRejected;
}
Status ApplyIncrement(const ElasticCouponConfiguration& base,const Coordinate* coordinates,std::size_t count,
                      const double* increment,double scale,ElasticCouponConfiguration& output,std::string& diagnostic) {
    if (!coordinates || !increment || !count || count>kCouponFreeDofs || !std::isfinite(scale)) {
        diagnostic="Invalid shell-patch coordinate layout or increment";
        return Status::kInvalidConfiguration;
    }
    for (std::size_t node=0;node<kCouponNodes;++node)
        if (!tlr::detail::Finite(base.position[node]) || !tl::math::UnitQuaternion(base.rotation[node])) {
            diagnostic="Shell-patch increment requires finite positions and unit quaternions";
            return Status::kInvalidConfiguration;
        }
    double values[kCouponNodes][6]{};
    bool used[kCouponNodes][6]{},incident[kCouponNodes]{};
    for (std::size_t i=0;i<count;++i) {
        const auto coordinate=coordinates[i];
        if (coordinate.node>=kCouponNodes || coordinate.component>=6 || used[coordinate.node][coordinate.component]) {
            diagnostic="Repeated or out-of-range shell-patch coordinate";
            return Status::kInvalidConfiguration;
        }
        used[coordinate.node][coordinate.component]=true; incident[coordinate.node]=true;
        values[coordinate.node][coordinate.component]=scale*increment[i];
    }
    auto candidate=base;
    for (std::size_t node=0;node<kCouponNodes;++node) if (incident[node]) {
        candidate.position[node]=tlr::detail::Add(base.position[node],{values[node][0],values[node][1],values[node][2]});
        if (!tlr::detail::Finite(candidate.position[node]) ||
            !tl::math::IncrementWorldRotation(base.rotation[node],values[node]+3,candidate.rotation[node])) {
            diagnostic="Shell-patch increment produced invalid translation or world rotation";
            return Status::kInvalidConfiguration;
        }
    }
    output=candidate; diagnostic.clear(); return Status::kSuccess;
}

template<std::size_t N> Vector<N> InverseRootMass(const ElasticCouponData& data,const Layout<N>& layout) {
    Vector<N> result;
    for (std::size_t i=0;i<N;++i) {
        const auto& coordinate=layout[i];
        if (coordinate.node>=kCouponNodes || coordinate.component>=6) {
            result.setConstant(std::numeric_limits<double>::quiet_NaN()); return result;
        }
        const auto& mass=data.nodal_mass[coordinate.node];
        result(i)=1/std::sqrt(coordinate.component<3 ? mass.mass : mass.physical_tangential_inertia);
    }
    return result;
}
template<std::size_t N> Matrix<N> MassScale(const Matrix<N>& matrix,const Vector<N>& inverse_root_mass) {
    return inverse_root_mass.asDiagonal()*matrix*inverse_root_mass.asDiagonal();
}
template<std::size_t N> double OperatorNorm(const Matrix<N>& matrix) {
    const Eigen::JacobiSVD<Matrix<N>> decomposition(matrix);
    if (decomposition.info()!=Eigen::Success || !decomposition.singularValues().allFinite())
        return std::numeric_limits<double>::quiet_NaN();
    return decomposition.singularValues()(0);
}
namespace {
template<std::size_t N> Status Forces(const ElasticCouponModel& model,const ElasticCouponConfiguration& configuration,
                                     const Layout<N>& layout,ForceSource source,Vector<N>& output,std::string& diagnostic) {
    ElasticCouponEvaluation evaluation;
    const auto status=source==ForceSource::Chrono ? model.EvaluateChrono(configuration,evaluation,diagnostic)
                                                 : model.EvaluateTL(configuration,evaluation,diagnostic);
    if (status!=Status::kSuccess) return status;
    Vector<N> result;
    for (std::size_t i=0;i<N;++i) {
        const auto c=layout[i];
        result(i)=tlr::detail::Component(c.component<3 ? evaluation.force[c.node] : evaluation.couple[c.node],c.component%3);
    }
    output=result; return status;
}
template<std::size_t N> double ColumnRelativeError(const Matrix<N>& a,const Matrix<N>& b) {
    double worst=0;
    for (std::size_t column=0;column<N;++column) {
        const double denominator=std::max(a.col(column).norm(),b.col(column).norm());
        if (!std::isfinite(denominator) || !(denominator>0)) return std::numeric_limits<double>::infinity();
        const double error=(a.col(column)-b.col(column)).norm()/denominator;
        if (!std::isfinite(error)) return std::numeric_limits<double>::infinity();
        worst=std::max(worst,error);
    }
    return worst;
}
template<std::size_t N> double RelativeError(const Matrix<N>& a,const Matrix<N>& b) {
    return (a-b).norm()/std::max(a.norm(),b.norm());
}
}

template<std::size_t N> Status DifferenceJacobian(const ElasticCouponModel& model,const ElasticCouponConfiguration& base,
                                                 const Layout<N>& layout,ForceSource source,double difference_scale,
                                                 Matrix<N>& output,std::string& diagnostic) {
    if (!std::isfinite(difference_scale) || difference_scale<=0) {
        diagnostic="Invalid shell-patch difference scale"; return Status::kInvalidConfiguration;
    }
    Matrix<N> candidate;
    for (std::size_t column=0;column<N;++column) {
        std::array<double,N> increment{};
        const double delta=difference_scale*(layout[column].component<3 ? TranslationDifference : RotationDifference);
        increment[column]=delta;
        ElasticCouponConfiguration plus,minus;
        auto status=ApplyIncrement(base,layout.data(),N,increment.data(),1,plus,diagnostic);
        if (status!=Status::kSuccess) return status;
        status=ApplyIncrement(base,layout.data(),N,increment.data(),-1,minus,diagnostic);
        if (status!=Status::kSuccess) return status;
        Vector<N> force_plus,force_minus;
        status=Forces<N>(model,plus,layout,source,force_plus,diagnostic);
        if (status!=Status::kSuccess) return status;
        status=Forces<N>(model,minus,layout,source,force_minus,diagnostic);
        if (status!=Status::kSuccess) return status;
        candidate.col(column)=-(force_plus-force_minus)/(2*delta);
    }
    if (!candidate.allFinite()) { diagnostic="Shell-patch finite-difference Jacobian is nonfinite"; return Status::kNonfiniteResult; }
    output=candidate; return Status::kSuccess;
}
template<std::size_t N> Status DirectionalCrossCheck(const ElasticCouponModel& model,const ElasticCouponConfiguration& base,
                                                   const Layout<N>& layout,const Matrix<N>& chrono_stiffness,
                                                   const Vector<N>& inverse_root_mass,double& worst,std::string& diagnostic) {
    if (!std::isfinite(worst) || worst<0 || !chrono_stiffness.allFinite() ||
        !inverse_root_mass.allFinite() || (inverse_root_mass.array()<=0).any()) {
        diagnostic="Invalid shell-patch directional audit inputs";
        return Status::kInvalidConfiguration;
    }
    double candidate=worst;
    for (std::size_t direction=0;direction<3;++direction) {
        std::array<double,N> increment{}; Vector<N> delta;
        for (std::size_t c=0;c<N;++c) {
            const double sign=((c*(2*direction+1)+direction)%7<3) ? -1 : 1;
            increment[c]=sign*(1+.1*((c+direction)%3))*
                         (layout[c].component<3 ? TranslationDifference : RotationDifference)*.5;
            delta(c)=increment[c];
        }
        ElasticCouponConfiguration plus,minus;
        auto status=ApplyIncrement(base,layout.data(),N,increment.data(),1,plus,diagnostic);
        if (status!=Status::kSuccess) return status;
        status=ApplyIncrement(base,layout.data(),N,increment.data(),-1,minus,diagnostic);
        if (status!=Status::kSuccess) return status;
        Vector<N> force_plus,force_minus;
        status=Forces<N>(model,plus,layout,ForceSource::TL,force_plus,diagnostic);
        if (status!=Status::kSuccess) return status;
        status=Forces<N>(model,minus,layout,ForceSource::TL,force_minus,diagnostic);
        if (status!=Status::kSuccess) return status;
        const Vector<N> independent=inverse_root_mass.asDiagonal()*(-(force_plus-force_minus)/2);
        const Vector<N> predicted=inverse_root_mass.asDiagonal()*(chrono_stiffness*delta);
        const double error=(independent-predicted).norm()/std::max(independent.norm(),predicted.norm());
        if (!std::isfinite(error) || error>DerivativeTolerance)
            return Reject("TL/Chrono mixed directional derivative",error,DerivativeTolerance,diagnostic);
        candidate=std::max(candidate,error);
    }
    worst=candidate; return Status::kSuccess;
}
template<std::size_t N> Status AuditReference(const ElasticCouponModel& model,const Layout<N>& layout,
                                             ReferenceSpectrum<N>& output,std::string& diagnostic) {
    const auto& neutral=model.data().reference_configuration;
    ReferenceSpectrum<N> result; result.inverse_root_mass=InverseRootMass<N>(model.data(),layout);
    Matrix<N> coarse,fine,tl_fine;
    auto status=DifferenceJacobian<N>(model,neutral,layout,ForceSource::Chrono,1,coarse,diagnostic);
    if (status!=Status::kSuccess) return status;
    status=DifferenceJacobian<N>(model,neutral,layout,ForceSource::Chrono,.5,fine,diagnostic);
    if (status!=Status::kSuccess) return status;
    status=DifferenceJacobian<N>(model,neutral,layout,ForceSource::TL,.5,tl_fine,diagnostic);
    if (status!=Status::kSuccess) return status;
    const Matrix<N> a_coarse=MassScale<N>(coarse,result.inverse_root_mass);
    const Matrix<N> a_fine=MassScale<N>(fine,result.inverse_root_mass);
    const Matrix<N> a_tl=MassScale<N>(tl_fine,result.inverse_root_mass);
    result.symmetry_error=std::max(RelativeError<N>(a_fine,a_fine.transpose()),RelativeError<N>(a_coarse,a_coarse.transpose()));
    if (!std::isfinite(result.symmetry_error) || result.symmetry_error>DerivativeTolerance)
        return Reject("Stress-free mass-scaled symmetry",result.symmetry_error,DerivativeTolerance,diagnostic);
    result.derivative_refinement_error=ColumnRelativeError<N>(a_coarse,a_fine);
    if (result.derivative_refinement_error>DerivativeTolerance)
        return Reject("FD halving column disagreement",result.derivative_refinement_error,DerivativeTolerance,diagnostic);
    result.tl_derivative_error=ColumnRelativeError<N>(a_tl,a_fine);
    if (result.tl_derivative_error>DerivativeTolerance)
        return Reject("TL/Chrono all-DOF derivative disagreement",result.tl_derivative_error,DerivativeTolerance,diagnostic);
    const Matrix<N> symmetric=.5*(a_fine+a_fine.transpose());
    const Eigen::SelfAdjointEigenSolver<Matrix<N>> spectrum(symmetric);
    const Eigen::SelfAdjointEigenSolver<Matrix<N>> coarse_spectrum(.5*(a_coarse+a_coarse.transpose()));
    if (spectrum.info()!=Eigen::Success || coarse_spectrum.info()!=Eigen::Success ||
        !spectrum.eigenvalues().allFinite() || !coarse_spectrum.eigenvalues().allFinite()) {
        diagnostic="Clamped shell-patch eigensolver failed"; return Status::kModalFailure;
    }
    const double largest=spectrum.eigenvalues()(N-1),smallest=spectrum.eigenvalues()(0);
    const double uncertainty=OperatorNorm<N>(a_coarse-a_fine);
    const double positive_floor=std::max(1e-10*largest,10*uncertainty);
    if (!std::isfinite(positive_floor) || !(smallest>positive_floor) || coarse_spectrum.eigenvalues()(0)<=0)
        return Reject("Unresolved zero/negative clamped mode (minimum must exceed FD uncertainty)",smallest,positive_floor,diagnostic);
    result.maximum_frequency_refinement_error=std::abs(std::sqrt(largest/coarse_spectrum.eigenvalues()(N-1))-1);
    if (result.maximum_frequency_refinement_error>FrequencyRefinementTolerance)
        return Reject("Maximum frequency FD-halving change",result.maximum_frequency_refinement_error,FrequencyRefinementTolerance,diagnostic);
    result.eigen_residual=(symmetric*spectrum.eigenvectors()-spectrum.eigenvectors()*spectrum.eigenvalues().asDiagonal()).norm()/symmetric.norm();
    if (!std::isfinite(result.eigen_residual) || result.eigen_residual>1e-10)
        return Reject("Mass-normalized eigen residual",result.eigen_residual,1e-10,diagnostic);
    result.stiffness=fine; result.squared_frequency=spectrum.eigenvalues(); result.mass_modes=spectrum.eigenvectors();
    output=result; diagnostic.clear(); return Status::kSuccess;
}

// Fixed-size Eigen arithmetic preserves B2's existing 24-coordinate operation
// order. D instantiates the same tools at exactly sixteen coordinates.
#define INSTANTIATE_PATCH_AUDIT(N) \
template Vector<N> InverseRootMass<N>(const ElasticCouponData&,const Layout<N>&); \
template Matrix<N> MassScale<N>(const Matrix<N>&,const Vector<N>&); \
template double OperatorNorm<N>(const Matrix<N>&); \
template Status DifferenceJacobian<N>(const ElasticCouponModel&,const ElasticCouponConfiguration&,const Layout<N>&,ForceSource,double,Matrix<N>&,std::string&); \
template Status DirectionalCrossCheck<N>(const ElasticCouponModel&,const ElasticCouponConfiguration&,const Layout<N>&,const Matrix<N>&,const Vector<N>&,double&,std::string&); \
template Status AuditReference<N>(const ElasticCouponModel&,const Layout<N>&,ReferenceSpectrum<N>&,std::string&);
INSTANTIATE_PATCH_AUDIT(16)
INSTANTIATE_PATCH_AUDIT(24)
#undef INSTANTIATE_PATCH_AUDIT

}  // namespace crash::reference::patch_audit
