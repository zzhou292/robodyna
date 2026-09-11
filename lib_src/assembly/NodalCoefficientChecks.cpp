// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCoefficientInternal.h"
#include "../../lib_utils/SourceIdentityIndex.h"

namespace tl::fea::coefficient_detail {
CoefficientReport Identities(NodalCoefficientSources input) {
  const auto& shells=*input.shells->shells();
  const auto& domain=*input.shells->domain();
  if(input.type13&&!input.type13->domain()->Matches(domain))
    return {S::IdentityMismatch,"TYPE13 contribution domain differs",P::Type13};
  if(input.type25&&(input.type25->source_instance_id()!=domain.source_instance_id()||
      input.type25->global_node_count()!=domain.node_count()))
    return {S::IdentityMismatch,"TYPE25 source instance or node extent differs",P::Type25};
  const auto q=shells.qeph_count(),t=shells.t3_count(),b=shells.qbat_count();
  const auto beams=input.type13?input.type13->model()->connection_count():0;
  auto id=[&](std::size_t i) {
    if(i<q) return shells.qeph_source_id(i);
    i-=q;
    if(i<t) return shells.t3_source_id(i);
    i-=t;
    if(i<b) return shells.qbat_source_id(i);
    return input.type13->model()->connections()[i-b].source_id;
  };
  util::SourceIdentityIndex<0> identities;
  identities.Prepare(q+t+b+beams,id);
  for(std::size_t i=0;i<q+t+b+beams;++i) {
    const auto producer=i<q?P::Qeph:i<q+t?P::T3:i<q+t+b?P::Qbat:P::Type13;
    const auto parent=i<q?i:i<q+t?i-q:i<q+t+b?i-q-t:i-q-t-b;
    if(!id(i)) return {S::IdentityMismatch,"Original structural EID must be positive",producer,parent};
    if(identities.First(id(i))!=i)
      return {S::DuplicateIdentity,"Repeated original structural EID",producer,parent};
  }
  return {};
}

CoefficientReport Totals(NodalCoefficientNode* nodes,std::size_t count,
    NodalCoefficientTotals& totals,NodalCoefficientScope& scope) noexcept {
  for(std::size_t n=0;n<count;++n) {
    const auto& row=nodes[n];
    const auto& c=row.coefficients;
    // All global diagnostic and authoritative totals use domain-node order.
    if(!AddPair(totals.mass,totals.isotropic_inertia,c.mass,c.isotropic_inertia)||
        !AddPair(totals.shell.mass,totals.shell.isotropic_inertia,c.shell.mass,c.shell.isotropic_inertia)||
        !Add(totals.shell.physical_inertia,c.shell.physical_inertia)||
        !Add(totals.shell.added_inertia,c.shell.added_inertia)||
        !AddPair(totals.type25.mass,totals.type25.isotropic_inertia,c.type25.mass,c.type25.isotropic_inertia)||
        !AddPair(totals.type13.mass,totals.type13.isotropic_inertia,c.type13.mass,c.type13.isotropic_inertia)||
        !Add(totals.type13.added_inertia,c.type13.added_inertia))
      return {S::NonfiniteResult,"Global coefficient sum overflow",P::NodeTotals,SIZE_MAX,SIZE_MAX,n};
    const auto& o=row.occurrences;
    scope.occurrences.qeph+=o.qeph;
    scope.occurrences.t3+=o.t3;
    scope.occurrences.qbat+=o.qbat;
    scope.occurrences.type25+=o.type25;
    scope.occurrences.type13+=o.type13;
    if(o.qeph||o.t3||o.qbat||o.type25||o.type13) ++scope.covered_nodes;
    else ++scope.uncovered_nodes;
  }
  if(scope.occurrences.qeph!=4*scope.qeph_parents||
      scope.occurrences.t3!=3*scope.t3_parents||
      scope.occurrences.qbat!=4*scope.qbat_parents||
      scope.occurrences.type25!=2*scope.type25_connections||
      scope.occurrences.type13!=2*scope.type13_connections)
    return {S::IdentityMismatch,"Complete producer occurrence count differs"};
  return {};
}
} // namespace tl::fea::coefficient_detail
