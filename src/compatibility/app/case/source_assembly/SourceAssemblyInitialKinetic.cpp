#include "SourceAssemblyInitialKinetic.h"
#include "output/ArtifactIO.h"
#include <cmath>
#include <new>

namespace crash::cases::source_assembly {
namespace wp=wall_penalty;
namespace qb=tlfea::contact::q4_bounds;
InitialKineticReport EncloseSourceAssemblyInitialKinetic(const SourceAssemblyBindings& assembly,double speed,
    SourceAssemblyInitialKinetic* output) {
    using Code=InitialKineticStatus;
    const auto& shells=assembly.shells();const auto* model=assembly.rigid_groups();const auto count=shells.node_count();
    if(!output||!shells.prepared()||!count||count>tl::fea::MaxShellHostNodes||!model||!model->prepared()||
       model->source_instance_id()!=assembly.source_instance_id()||model->global_node_count()!=count||
       !model->group_count()||model->group_count()>64||model->member_count()>count)
        return {Code::InvalidInput,"Complete native shells and their retained plain rigid groups are required"};
    wp::UniformTranslationKinetic native,aggregate;
    if(!wp::BeginUniformTranslation(speed,&native)||!wp::BeginUniformTranslation(speed,&aggregate))
        return {Code::InvalidInput,"Initial energy requires finite positive uniform +X speed and zero spin"};
    try {
        SourceAssemblyInitialKinetic next;next.groups.reserve(model->group_count());
        std::vector<std::uint8_t> member_nodes(count,0);
        qb::Interval primary_mass;double primary_nominal=0;
        for(std::size_t g=0;g<model->group_count();++g) {
            const auto& properties=model->groups()[g];
            if(properties.member_count<3||properties.member_offset>model->member_count()||
               properties.member_count>model->member_count()-properties.member_offset)
                return {Code::InvalidInput,"Rigid group range is not complete",SIZE_MAX,g};
            GroupInitialKinetic group;group.source_group_id=properties.source_group_id;
            group.source_node_set_id=properties.source_node_set_id;group.member_count=properties.member_count;
            wp::UniformTranslationKinetic members,whole;
            if(!wp::BeginUniformTranslation(speed,&members)||!wp::BeginUniformTranslation(speed,&whole))
                return {Code::CertificateFailure,"Group initial speed cannot be enclosed",SIZE_MAX,g};
            for(std::size_t m=0;m<properties.member_count;++m) {
                const auto& member=model->members()[properties.member_offset+m];const auto n=member.global_node;
                if(n>=count||member_nodes[n])return {Code::InvalidInput,"Incomplete or repeated native group member",n,g};
                const auto& node=shells.nodes()[n];const auto& metric=node.native;
                if(member.source_node_id!=node.source_id||crash::output::Bits(member.position.x)!=crash::output::Bits(node.position.x)||
                   crash::output::Bits(member.position.y)!=crash::output::Bits(node.position.y)||crash::output::Bits(member.position.z)!=crash::output::Bits(node.position.z)||
                   crash::output::Bits(member.mass_kg)!=crash::output::Bits(metric.mass)||
                   crash::output::Bits(member.total_inertia_kg_m2)!=crash::output::Bits(metric.isotropic_inertia)||
                   crash::output::Bits(member.physical_inertia_kg_m2)!=crash::output::Bits(metric.physical_inertia)||
                   crash::output::Bits(member.added_inertia_kg_m2)!=crash::output::Bits(metric.added_inertia))
                    return {Code::InvalidInput,"Group member changed its complete native source or M/J binding",n,g};
                if(!wp::AddTranslationMass(member.mass_kg,&members))
                    return {Code::CertificateFailure,"Native group member kinetic sum cannot be enclosed",n,g};
                member_nodes[n]=1;++next.member_nodes;
            }
            // total_mass_kg already contains the generated primary. The separate
            // regularization ledger is diagnostic and is never added again.
            group.generated_primary_mass_kg=properties.regularization.primary_mass_kg;
            const double primary=group.generated_primary_mass_kg;
            if(!std::isfinite(primary)||primary<=0||!qb::Add(primary_mass,{primary,primary},&primary_mass)||
               !wp::AddTranslationMass(properties.total_mass_kg,&whole)||
               !wp::FinishUniformTranslation(members,&group.native_members)||
               !wp::FinishUniformTranslation(whole,&group.aggregate))
                return {Code::CertificateFailure,"Aggregate or primary regularization cannot be enclosed",SIZE_MAX,g};
            primary_nominal+=primary;if(!std::isfinite(primary_nominal))
                return {Code::CertificateFailure,"Generated-primary mass sum overflows",SIZE_MAX,g};
            ++next.generated_primaries;next.groups.push_back(group);
        }
        if(next.member_nodes!=model->member_count())return {Code::InvalidInput,"Group ledger does not cover every source member"};
        for(std::size_t n=0;n<count;++n) {
            const double mass=assembly.coefficients(n).mass;
            if(!wp::AddTranslationMass(mass,&native))return {Code::CertificateFailure,"Native initial kinetic sum cannot be enclosed",n};
            if(!member_nodes[n]) {
                if(!wp::AddTranslationMass(mass,&aggregate))return {Code::CertificateFailure,"Ordinary-node kinetic sum cannot be enclosed",n};
                ++next.ordinary_nodes;
            }
        }
        for(std::size_t g=0;g<model->group_count();++g)
            if(!wp::AddTranslationMass(model->groups()[g].total_mass_kg,&aggregate))
                return {Code::CertificateFailure,"Complete aggregate initial kinetic sum cannot be enclosed",SIZE_MAX,g};
        if(!wp::FinishUniformTranslation(native,&next.native_nodes)||
           !wp::FinishUniformTranslation(aggregate,&next.with_aggregate_groups)||
           !qb::Certify(primary_nominal,primary_mass,&next.generated_primary_mass_kg))
            return {Code::CertificateFailure,"Initial native/aggregate energy or primary mass certificate failed"};
        *output=std::move(next);return {Code::Ok,"Native and aggregate uniform-startup energy scopes certified separately"};
    } catch(const std::bad_alloc&) { return {Code::ResourceLimit,"Bounded initial energy ledger allocation failed"}; }
}
} // namespace crash::cases::source_assembly
