#include "ThinShellScreenInternal.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <utility>

namespace crash::reference::thin_shell_detail {
namespace {
bool ValidMass(const ThinShellVector& root) { return root.allFinite()&&(root.array()>0).all(); }

bool Pool(const ThinShellSpectrumDiagnostic& source,const ThinShellVector& root,
          bool selected,std::uint32_t label,ShellModeSet& pool,
          std::array<std::size_t,kCouponFreeDofs>& indices) {
    pool.coordinate_count=kShellTranslationCapacity;
    if (!source.available||!source.positive_spectrum||!ValidMass(root)) return false;
    for (std::size_t g=0;g<source.cluster_count;++g) {
        const auto& cluster=source.clusters[g];
        if (!cluster.bending_eligible||(selected&&cluster.label!=label)) continue;
        for (std::size_t member=0;member<cluster.count;++member) {
            const std::size_t mode=cluster.modes[member],entry=pool.mode_count++;
            indices[entry]=mode;
            pool.frequency[entry]=source.modes[mode].angular_frequency;
            pool.cluster[entry]=cluster.label;
            for (std::size_t node=0;node<kCouponFreeNodes.size();++node)
                for (std::size_t c=0;c<3;++c)
                    pool.translation[entry][3*node+c]=root(6*node+c)*source.mass_modes(6*node+c,mode);
        }
    }
    return pool.mode_count>0;
}

bool RefinedFrequencies(ThinShellModeMatchDiagnostic& result) {
    bool refined=true;
    for (std::size_t g=0;g<result.comparison.cluster_count;++g) {
        const auto& match=result.comparison.clusters[g];
        std::array<double,kCouponFreeDofs> a{},b{};
        for (std::size_t i=0;i<match.mode_count;++i) {
            a[i]=result.reference_pool.frequency[match.reference_modes[i]];
            b[i]=result.candidate_pool.frequency[match.candidate_modes[i]];
        }
        std::sort(a.begin(),a.begin()+match.mode_count);
        std::sort(b.begin(),b.begin()+match.mode_count);
        // MatchCluster is called fine -> coarse for FD matching. Preserve the
        // owning audit's coarse-frequency denominator, |fine/coarse - 1|.
        for (std::size_t i=0;i<match.mode_count;++i) {
            const long double difference=std::fabs(static_cast<long double>(b[i])-a[i]);
            result.maximum_fd_relative_frequency_change=std::max(result.maximum_fd_relative_frequency_change,
                                                                  static_cast<double>(difference/b[i]));
            if (200*difference>static_cast<long double>(b[i])) refined=false;
        }
    }
    return refined;
}
} // namespace

bool InspectSpectrum(const ThinShellMatrix& stiffness,const ThinShellVector& root,
                     ThinShellSpectrumDiagnostic& output,std::string& diagnostic) {
    if (!stiffness.allFinite()||!ValidMass(root)) {
        diagnostic="Raw eigendiagnostic requires finite stiffness and positive mass roots"; return false;
    }
    const ThinShellMatrix scaled=patch_audit::MassScale<kCouponFreeDofs>(stiffness,root);
    const ThinShellMatrix symmetric=.5*(scaled+scaled.transpose());
    if (!scaled.allFinite()||!symmetric.allFinite()) {
        diagnostic="Raw mass-scaled matrix is unrepresentable"; return false;
    }
    const Eigen::SelfAdjointEigenSolver<ThinShellMatrix> spectrum(symmetric);
    if (spectrum.info()!=Eigen::Success||!spectrum.eigenvalues().allFinite()||!spectrum.eigenvectors().allFinite()) {
        diagnostic="Raw symmetric eigendiagnostic failed"; return false;
    }
    ThinShellSpectrumDiagnostic result;
    result.squared_frequency=spectrum.eigenvalues(); result.mass_modes=spectrum.eigenvectors();
    const double norm=scaled.norm(),symmetric_norm=symmetric.norm();
    result.symmetry_error=norm>0 ? (scaled-scaled.transpose()).norm()/norm : 0;
    result.eigen_residual=symmetric_norm>0 ?
        (symmetric*result.mass_modes-result.mass_modes*result.squared_frequency.asDiagonal()).norm()/symmetric_norm : 0;
    if (!std::isfinite(norm)||!std::isfinite(symmetric_norm)||!std::isfinite(result.symmetry_error)||
        !std::isfinite(result.eigen_residual)) {
        diagnostic="Raw eigendiagnostic normalization is unrepresentable"; return false;
    }
    result.available=true;
    // Classification can reject a raw spectrum without erasing its matrices,
    // eigenvalues, modes, or reported residual.
    ClassifyModes(result,root,result.diagnostic);
    output=std::move(result); diagnostic.clear(); return true;
}

bool ClassifyModes(ThinShellSpectrumDiagnostic& spectrum,const ThinShellVector& root,std::string& diagnostic) {
    if (!spectrum.available||!spectrum.squared_frequency.allFinite()||!spectrum.mass_modes.allFinite()||!ValidMass(root)) {
        diagnostic="Mode classification requires finite raw eigenvectors and positive mass roots"; return false;
    }
    spectrum.positive_spectrum=true;
    std::array<double,kCouponFreeDofs> frequencies{};
    for (std::size_t mode=0;mode<kCouponFreeDofs;++mode) {
        auto& value=spectrum.modes[mode];
        value=ThinShellModeDiagnostic{};
        value.positive_frequency=spectrum.squared_frequency(mode)>0;
        if (value.positive_frequency) value.angular_frequency=std::sqrt(spectrum.squared_frequency(mode));
        else spectrum.positive_spectrum=false;
        frequencies[mode]=value.angular_frequency;
        for (std::size_t node=0;node<kCouponFreeNodes.size();++node) {
            for (std::size_t c=0;c<6;++c) {
                const double component=spectrum.mass_modes(6*node+c,mode),share=component*component;
                if (c<3) value.translation_mass_fraction+=share;
                if (c==2) value.normal_translation_mass_fraction+=share;
                if (c==3||c==4) value.transverse_rotation_mass_fraction+=share;
                if (c==5) value.drilling_mass_fraction+=share;
            }
        }
        value.out_of_plane_mass_fraction=value.normal_translation_mass_fraction+value.transverse_rotation_mass_fraction;
    }
    spectrum.cluster_count=0;
    spectrum.clusters={};
    if (!spectrum.positive_spectrum) {
        diagnostic="Raw spectrum has a zero/negative eigenvalue; no positive-frequency cluster identity"; return false;
    }
    ShellModeClusters labels;
    if (ClusterShellFrequencies(frequencies,kCouponFreeDofs,labels,diagnostic)!=ShellModeComparisonStatus::kSuccess) return false;
    for (std::size_t mode=0;mode<kCouponFreeDofs;++mode) {
        const auto label=labels[mode];
        auto& group=spectrum.clusters[label];
        group.label=label; group.modes[group.count++]=mode;
        spectrum.cluster_count=std::max(spectrum.cluster_count,static_cast<std::size_t>(label)+1);
    }
    for (std::size_t g=0;g<spectrum.cluster_count;++g) {
        auto& group=spectrum.clusters[g];
        for (std::size_t i=0;i<group.count;++i) {
            const auto mode=group.modes[i];
            group.mean_out_of_plane_fraction+=spectrum.modes[mode].out_of_plane_mass_fraction/group.count;
            group.mean_normal_translation_fraction+=spectrum.modes[mode].normal_translation_mass_fraction/group.count;
            const double a=root(14)*spectrum.mass_modes(14,mode),b=root(20)*spectrum.mass_modes(20,mode);
            group.tip_a_row_norm=std::hypot(group.tip_a_row_norm,a);
            group.tip_b_row_norm=std::hypot(group.tip_b_row_norm,b);
            group.mean_tip_row_norm=std::hypot(group.mean_tip_row_norm,.5*(a+b));
        }
        group.bending_eligible=std::isfinite(group.mean_tip_row_norm)&&
            group.mean_out_of_plane_fraction>=.9&&group.mean_normal_translation_fraction>=.5&&
            group.mean_tip_row_norm>.5*std::max(group.tip_a_row_norm,group.tip_b_row_norm);
        if (!group.bending_eligible) group.diagnostic="Cluster fails frozen bending participation or coherent-tip classification";
    }
    diagnostic.clear(); return true;
}

void MatchCluster(const ThinShellSpectrumDiagnostic& reference,const ThinShellVector& reference_root,
                  std::uint32_t label,const ThinShellSpectrumDiagnostic& candidate,const ThinShellVector& candidate_root,
                  const patch_audit::PatchNodalMass& mass,bool require_refinement,ThinShellModeMatchDiagnostic& output) {
    ThinShellModeMatchDiagnostic result;
    result.attempted=true;
    if (!Pool(reference,reference_root,true,label,result.reference_pool,result.reference_modes)||
        !Pool(candidate,candidate_root,false,0,result.candidate_pool,result.candidate_modes)) {
        result.diagnostic="Selected cluster or candidate bending pool is unresolved/ineligible";
        output=std::move(result); return;
    }
    ShellTranslationMass common{};
    for (std::size_t n=0;n<kCouponFreeNodes.size();++n)
        for (std::size_t c=0;c<3;++c) common[3*n+c]=mass.mass[kCouponFreeNodes[n]];
    result.status=CompareShellModes(result.reference_pool,result.candidate_pool,common,result.comparison,result.diagnostic);
    result.matched=result.status==ShellModeComparisonStatus::kSuccess;
    if (result.matched&&require_refinement) {
        result.frequency_refined=RefinedFrequencies(result);
        if (!result.frequency_refined) {
            result.status=ShellModeComparisonStatus::kFrequencyMismatch;
            result.diagnostic="Tracked cluster exceeds the unchanged 0.5% coarse/fine frequency gate";
        }
    }
    output=std::move(result);
}

} // namespace crash::reference::thin_shell_detail
