// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellPhysicalOutputRanges.h"
#include "ShellFormulationOutputRanges.h"

namespace tl::fea::shell_physical_owner {
namespace {
template<class T> bool Range(const void* output,std::size_t bytes,const T* source,
    std::size_t count=1) noexcept {
  return !count || trial_identity::Disjoint(output,bytes,source,count*sizeof(T));
}
bool DomainRange(const NodalNodeDomain& source,const void* output,std::size_t bytes) noexcept {
  return Range(output,bytes,&source) && Range(output,bytes,source.nodes().data(),source.node_count());
}
bool Type13Ranges(const Type13NodeContributions& source,const void* output,std::size_t bytes) noexcept {
  const auto& model=*source.model();
  const auto records=source.records();
  if (!Range(output,bytes,&source) || !DomainRange(*source.domain(),output,bytes) || !Range(output,bytes,&model) ||
      !Range(output,bytes,records.data(),records.size()) ||
      !Range(output,bytes,model.nodes(),model.node_count()) ||
      !Range(output,bytes,model.connections(),model.connection_count())) return false;
  for (std::size_t property=0;property<model.property_count();++property) {
    const auto* declaration=model.property_declaration(property);
    if (!Range(output,bytes,declaration) || !Range(output,bytes,model.property(property))) return false;
    for (const auto& curve:declaration->input.curves) {
      if (!Range(output,bytes,curve.points,curve.count)) return false;
    }
  }
  for (std::size_t element=0;element<model.connection_count();++element) {
    if (!Range(output,bytes,model.startup(element))) return false;
  }
  return true;
}
}
bool OutputDisjoint(const ShellPhysicalBinding& physical,const void* output,std::size_t bytes) noexcept {
  if (!physical.prepared() || !Range(output,bytes,&physical)) return false;
  const auto& ledger=*physical.coefficients();
  const auto& domain=*physical.domain();
  const auto& map=*physical.mapping();
  const ShellFormulationScope scope{physical.shells(),physical.catalog(),physical.failure(),nullptr};
  if (!shell_formulation_detail::OutputDisjoint(scope,output,bytes) ||
      !Range(output,bytes,&ledger) || !Range(output,bytes,&ledger.totals()) ||
      !Range(output,bytes,&ledger.scope()) || !DomainRange(domain,output,bytes) ||
      !Range(output,bytes,&map) ||
      !Range(output,bytes,map.mapping().data(),map.shell_node_count()) ||
      !Range(output,bytes,ledger.nodes().data(),ledger.nodes().size())) return false;
  if (const auto* spring=ledger.type25()) {
    const auto count=spring->connection_count();
    if (!Range(output,bytes,spring) || !Range(output,bytes,spring->connections(),count) ||
        !Range(output,bytes,spring->properties(),spring->property_count()) ||
        !Range(output,bytes,spring->references(),count) ||
        !Range(output,bytes,spring->initial_histories(),count) ||
        !Range(output,bytes,spring->endpoint_mass(),2*count)) return false;
  }
  if (ledger.type13() && !Type13Ranges(*ledger.type13(),output,bytes)) return false;
  if (const auto* point=ledger.element_mass()) {
    if (!Range(output,bytes,point) || !DomainRange(*point->domain(),output,bytes) ||
        !Range(output,bytes,point->records().data(),point->records().size())) return false;
  }
  if (const auto* solids=ledger.solids()) {
    if (!Range(output,bytes,solids) || !DomainRange(*solids->domain(),output,bytes) ||
        !Range(output,bytes,solids->parents().data(),solids->parents().size())) return false;
  }
  return true;
}
} // namespace tl::fea::shell_physical_owner
