#include "ShellModeComparison.h"
// Install the owning Chrono Eigen extensions before Eigen is first included.
#include "chrono/core/ChMatrix.h"
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace crash::reference {
namespace {
using Status = ShellModeComparisonStatus;
using Matrix = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,0,
                             kShellTranslationCapacity,kShellTranslationCapacity>;
struct Group {
    std::uint32_t label = 0;
    std::size_t count = 0;
    std::array<std::size_t,kShellModeCapacity> modes{};
    Matrix basis;
};
struct Groups {
    std::size_t count = 0;
    std::array<Group,kShellModeCapacity> group;
};

Status Reject(Status status,const char* reason,std::string& diagnostic) {
    diagnostic = reason;
    return status;
}
Status RejectValue(Status status,const char* reason,double measured,double limit,std::string& diagnostic) {
    std::ostringstream message;
    message.precision(17);
    message << reason << ": measured " << measured << ", limit " << limit;
    diagnostic = message.str();
    return status;
}

Status Prepare(const ShellModeSet& set,const ShellTranslationMass& mass,Groups& groups,
               std::string& diagnostic) {
    ShellModeClusters expected;
    auto status = ClusterShellFrequencies(set.frequency,set.mode_count,expected,diagnostic);
    if (status != Status::kSuccess) return status;
    for (std::size_t i=0;i<set.mode_count;++i) {
        if (set.cluster[i] >= kShellModeCapacity)
            return Reject(Status::kInvalidInput,"cluster label exceeds the bounded mode capacity",diagnostic);
        for (std::size_t j=0;j<i;++j)
            if ((set.cluster[i]==set.cluster[j]) != (expected[i]==expected[j]))
                return Reject(Status::kInvalidInput,"cluster partition differs from frozen 1% adjacency rule",diagnostic);
        std::size_t g=0;
        while (g<groups.count && groups.group[g].label!=set.cluster[i]) ++g;
        if (g==groups.count) {
            groups.group[g].label=set.cluster[i];
            ++groups.count;
        }
        auto& group=groups.group[g];
        group.modes[group.count++]=i;
    }
    for (std::size_t g=0;g<groups.count;++g) {
        auto& group=groups.group[g];
        if (group.count>set.coordinate_count)
            return Reject(Status::kRankDeficient,"cluster contains more modes than translational coordinates",diagnostic);
        Matrix weighted(set.coordinate_count,group.count);
        for (std::size_t column=0;column<group.count;++column) {
            const auto mode=group.modes[column];
            std::array<long double,kShellTranslationCapacity> values{};
            long double norm=0;
            for (std::size_t row=0;row<set.coordinate_count;++row) {
                const double displacement=set.translation[mode][row];
                if (!std::isfinite(displacement))
                    return Reject(Status::kInvalidInput,"nonfinite physical displacement projection",diagnostic);
                values[row]=static_cast<long double>(displacement)*std::sqrt(static_cast<long double>(mass[row]));
                if (!std::isfinite(values[row]) || (values[row]==0 && displacement!=0))
                    return Reject(Status::kNumericalFailure,"common-mass projection is not representable",diagnostic);
                norm=std::hypot(norm,values[row]);
            }
            if (!std::isfinite(norm))
                return Reject(Status::kNumericalFailure,"common-mass projection normalization overflow",diagnostic);
            if (!(norm>0))
                return Reject(Status::kRankDeficient,"mode has zero translational projection",diagnostic);
            for (std::size_t row=0;row<set.coordinate_count;++row) {
                const double value=static_cast<double>(values[row]/norm);
                if (!std::isfinite(value) || (value==0 && values[row]!=0))
                    return Reject(Status::kNumericalFailure,"normalized projection is not representable",diagnostic);
                weighted(row,column)=value;
            }
        }
        Eigen::JacobiSVD<Matrix> svd(weighted,Eigen::ComputeThinU);
        if (svd.info()!=Eigen::Success || !svd.singularValues().allFinite() || !svd.matrixU().allFinite())
            return Reject(Status::kNumericalFailure,"translational subspace SVD failed",diagnostic);
        const double ratio=svd.singularValues()(group.count-1)/svd.singularValues()(0);
        if (!std::isfinite(ratio) || ratio<kShellModeMinimumRankRatio)
            return RejectValue(Status::kRankDeficient,"translational subspace rank ratio",ratio,
                               kShellModeMinimumRankRatio,diagnostic);
        group.basis=svd.matrixU();
    }
    return Status::kSuccess;
}

Status Similarity(const Group& a,const Group& b,double& squared,std::string& diagnostic) {
    Matrix overlap=a.basis.transpose()*b.basis;
    Eigen::JacobiSVD<Matrix> svd(overlap);
    if (svd.info()!=Eigen::Success || !svd.singularValues().allFinite())
        return Reject(Status::kNumericalFailure,"principal-angle SVD failed",diagnostic);
    const double cosine=svd.singularValues()(a.count-1);
    constexpr double roundoff=64*kShellTranslationCapacity*std::numeric_limits<double>::epsilon();
    if (cosine<0 || cosine>1+roundoff)
        return Reject(Status::kNumericalFailure,"principal-angle cosine is outside numerical unit range",diagnostic);
    squared=std::min(1.0,cosine)*std::min(1.0,cosine);
    return Status::kSuccess;
}

Status Frequencies(const ShellModeSet& reference,const Group& a,const ShellModeSet& candidate,
                   const Group& b,double& maximum,std::string& diagnostic) {
    std::array<double,kShellModeCapacity> original{},changed{};
    for (std::size_t i=0;i<a.count;++i) {
        original[i]=reference.frequency[a.modes[i]];
        changed[i]=candidate.frequency[b.modes[i]];
    }
    std::sort(original.begin(),original.begin()+a.count);
    std::sort(changed.begin(),changed.begin()+b.count);
    maximum=0;
    for (std::size_t i=0;i<a.count;++i) {
        const long double difference=std::fabs(static_cast<long double>(changed[i])-original[i]);
        const long double relative=difference/original[i];
        if (!std::isfinite(relative) || 20* difference>static_cast<long double>(original[i]))
            return RejectValue(Status::kFrequencyMismatch,"matched cluster relative frequency change",
                               static_cast<double>(relative),kShellModeMaximumRelativeFrequencyChange,diagnostic);
        maximum=std::max(maximum,static_cast<double>(relative));
    }
    return Status::kSuccess;
}
}  // namespace

ShellModeComparisonStatus ClusterShellFrequencies(
    const std::array<double,kShellModeCapacity>& frequencies,std::size_t count,
    ShellModeClusters& output,std::string& diagnostic) {
    if (!count || count>kShellModeCapacity)
        return Reject(Status::kInvalidInput,"frequency count is outside [1,24]",diagnostic);
    std::array<std::size_t,kShellModeCapacity> order{};
    for (std::size_t i=0;i<count;++i) {
        if (!std::isfinite(frequencies[i]) || !(frequencies[i]>0))
            return Reject(Status::kInvalidInput,"frequency must be finite and positive",diagnostic);
        order[i]=i;
    }
    std::sort(order.begin(),order.begin()+count,[&](auto a,auto b) {
        return frequencies[a]<frequencies[b] || (frequencies[a]==frequencies[b] && a<b);
    });
    ShellModeClusters staged{};
    std::uint32_t label=0;
    for (std::size_t i=1;i<count;++i) {
        const long double previous=frequencies[order[i-1]],next=frequencies[order[i]];
        if (100*(next-previous)>previous) ++label;
        staged[order[i]]=label;
    }
    output=staged;
    diagnostic.clear();
    return Status::kSuccess;
}

ShellModeComparisonStatus CompareShellModes(
    const ShellModeSet& reference,const ShellModeSet& candidate,
    const ShellTranslationMass& mass,ShellModeComparison& output,std::string& diagnostic) {
    if (!reference.coordinate_count || reference.coordinate_count>kShellTranslationCapacity ||
        !candidate.coordinate_count || candidate.coordinate_count>kShellTranslationCapacity ||
        !reference.mode_count || reference.mode_count>kShellModeCapacity ||
        !candidate.mode_count || candidate.mode_count>kShellModeCapacity)
        return Reject(Status::kInvalidInput,"mode or translation count exceeds the bounded comparison domain",diagnostic);
    if (reference.coordinate_count!=candidate.coordinate_count)
        return Reject(Status::kDimensionMismatch,"translational coordinate counts differ",diagnostic);
    for (std::size_t i=0;i<reference.coordinate_count;++i)
        if (!std::isfinite(mass[i]) || !(mass[i]>0))
            return Reject(Status::kInvalidInput,"common translational mass must be finite and positive",diagnostic);
    Groups a,b;
    auto status=Prepare(reference,mass,a,diagnostic);
    if (status!=Status::kSuccess) return status;
    status=Prepare(candidate,mass,b,diagnostic);
    if (status!=Status::kSuccess) return status;
    ShellModeComparison staged{};
    staged.mode_count=reference.mode_count;
    staged.cluster_count=a.count;
    staged.singleton_candidate.fill(kNoShellMode);
    staged.minimum_squared_cosine=1;
    std::array<bool,kShellModeCapacity> used{};
    std::array<std::size_t,kShellModeCapacity> selection{};
    for (std::size_t i=0;i<a.count;++i) {
        const auto& original=a.group[i];
        std::size_t matches=0,selected=0,compatible=0;
        double selected_similarity=0,best=0;
        for (std::size_t j=0;j<b.count;++j) {
            if (original.count!=b.group[j].count) continue;
            ++compatible;
            double similarity=0;
            status=Similarity(original,b.group[j],similarity,diagnostic);
            if (status!=Status::kSuccess) return status;
            best=std::max(best,similarity);
            if (similarity>=kShellModeMinimumSquaredCosine) {
                ++matches; selected=j; selected_similarity=similarity;
            }
        }
        if (!compatible)
            return Reject(Status::kDimensionMismatch,"no candidate cluster has the selected reference dimension",diagnostic);
        if (!matches)
            return RejectValue(Status::kUnmatched,"best translational subspace squared cosine",best,
                               kShellModeMinimumSquaredCosine,diagnostic);
        if (matches>1 || used[selected])
            return Reject(Status::kAmbiguous,"translational subspaces do not define a unique one-to-one match",diagnostic);
        used[selected]=true;
        selection[i]=selected;
        const auto& changed=b.group[selected];
        auto& match=staged.clusters[i];
        match.reference_cluster=original.label;
        match.candidate_cluster=changed.label;
        match.mode_count=original.count;
        match.reference_modes=original.modes;
        match.candidate_modes=changed.modes;
        match.minimum_squared_cosine=selected_similarity;
        staged.minimum_squared_cosine=std::min(staged.minimum_squared_cosine,selected_similarity);
        if (original.count==1) staged.singleton_candidate[original.modes[0]]=changed.modes[0];
    }
    // Complete geometric identity before any frequency acceptance. In
    // particular, a frequency mismatch cannot conceal a reused shape match.
    for (std::size_t i=0;i<a.count;++i) {
        auto& match=staged.clusters[i];
        status=Frequencies(reference,a.group[i],candidate,b.group[selection[i]],
                           match.maximum_relative_frequency_change,diagnostic);
        if (status!=Status::kSuccess) return status;
        staged.maximum_relative_frequency_change=std::max(staged.maximum_relative_frequency_change,
                                                          match.maximum_relative_frequency_change);
    }
    output=staged;
    diagnostic.clear();
    return Status::kSuccess;
}

}  // namespace crash::reference
