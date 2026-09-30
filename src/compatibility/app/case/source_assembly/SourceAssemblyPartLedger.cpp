#include "SourceAssemblyStartupInternal.h"
#include "output/ArtifactIO.h"

namespace crash::cases::source_assembly::startup {
namespace {
template<class Reference> void Append(tl::fea::ShellBindingMass& sum,const Reference& reference,unsigned arity) {
    for(unsigned n=0;n<arity;++n) {
        sum.mass+=reference.nodal_mass[n]; sum.isotropic_inertia+=reference.isotropic_inertia[n];
        sum.physical_inertia+=reference.physical_inertia[n]; sum.added_inertia+=reference.added_inertia[n];
    }
}
}
std::vector<PartNativeMassLedger> PartLedger(const source::Data& data,const tl::fea::ShellBatchBinding& binding) {
    std::vector<PartNativeMassLedger> ledgers;
    ledgers.reserve(data.parts.size());
    for(const auto& part:data.parts) {
        PartNativeMassLedger ledger; ledger.source_part_id=part.id;
        for(std::size_t i=part.first_parent;i<part.first_parent+part.parent_count;++i) {
            const auto& parent=data.parents.at(i); ++ledger.parent_count;
            if(parent.family==source::ShellFamily::Qeph) {
                ++ledger.qeph_count; Append(ledger.native,binding.qeph_reference(parent.family_index),4);
            } else {
                ++ledger.t3_count; Append(ledger.native,binding.t3_reference(parent.family_index),3);
            }
        }
        for(double value:{ledger.native.mass,ledger.native.isotropic_inertia,
                          ledger.native.physical_inertia,ledger.native.added_inertia})
            output::Require(tl::math::Finite(value)&&value>0,"Invalid diagnostic native part mass/inertia ledger");
        ledgers.push_back(ledger);
    }
    return ledgers;
}
} // namespace crash::cases::source_assembly::startup
