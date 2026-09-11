// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCoefficientInternal.h"
#include "../../lib_utils/SourceIdentityIndex.h"

namespace tl::fea::coefficient_detail {
CoefficientReport Identities(NodalCoefficientSources input,const ElementMassContributions* masses,
    const SolidNodeContributions* solids) {
  const auto& shells=*input.shells->shells();
  const auto& domain=*input.shells->domain();
  if(input.type13&&!input.type13->domain()->Matches(domain))
    return {S::IdentityMismatch,"TYPE13 contribution domain differs",P::Type13};
  if(masses&&!masses->domain()->Matches(domain))
    return {S::IdentityMismatch,"Element mass contribution domain differs",P::ElementMass};
  if(solids&&!solids->domain()->Matches(domain))
    return {S::IdentityMismatch,"Solid contribution domain differs",P::Solid18};
  if(input.type25&&(input.type25->source_instance_id()!=domain.source_instance_id()||
      input.type25->global_node_count()!=domain.node_count()))
    return {S::IdentityMismatch,"TYPE25 source instance or node extent differs",P::Type25};
  const auto q=shells.qeph_count(),t=shells.t3_count(),b=shells.qbat_count();
  const auto beams=input.type13?input.type13->model()->connection_count():0;
  const auto mass_records=masses?masses->records().size():0;
  const auto solid_parents=solids?solids->parents().size():0;
  auto id=[&](std::size_t i) {
    if(i<q) return shells.qeph_source_id(i);
    i-=q;
    if(i<t) return shells.t3_source_id(i);
    i-=t;
    if(i<b) return shells.qbat_source_id(i);
    i-=b;
    if(i<beams) return input.type13->model()->connections()[i].source_id;
    i-=beams;
    if(i<mass_records) return masses->records()[i].source.source_element_id;
    return solids->parents()[i-mass_records].source_element_id;
  };
  util::SourceIdentityIndex<0> identities;
  identities.Prepare(q+t+b+beams+mass_records+solid_parents,id);
  for(std::size_t i=0;i<q+t+b+beams+mass_records+solid_parents;++i) {
    auto producer=i<q?P::Qeph:i<q+t?P::T3:i<q+t+b?P::Qbat:
        i<q+t+b+beams?P::Type13:P::ElementMass;
    auto parent=i<q?i:i<q+t?i-q:i<q+t+b?i-q-t:
        i<q+t+b+beams?i-q-t-b:i-q-t-b-beams;
    if(i>=q+t+b+beams+mass_records) {
      parent=i-q-t-b-beams-mass_records;
      const auto family=solids->parents()[parent].family;
      producer=family==SolidCoefficientFamily::Solid18?P::Solid18:
          family==SolidCoefficientFamily::Solid24?P::Solid24:P::Solid6z;
    }
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
        !Add(totals.type13.added_inertia,c.type13.added_inertia)||
        !Add(totals.element_mass,c.element_mass)||
        !Add(totals.solid18_mass,c.solid18_mass)||!Add(totals.solid24_mass,c.solid24_mass)||
        !Add(totals.solid6z_mass,c.solid6z_mass))
      return {S::NonfiniteResult,"Global coefficient sum overflow",P::NodeTotals,SIZE_MAX,SIZE_MAX,n};
    const auto& o=row.occurrences;
    scope.occurrences.qeph+=o.qeph;
    scope.occurrences.t3+=o.t3;
    scope.occurrences.qbat+=o.qbat;
    scope.occurrences.type25+=o.type25;
    scope.occurrences.type13+=o.type13;
    scope.occurrences.element_mass+=o.element_mass;
    scope.occurrences.solid18+=o.solid18;
    scope.occurrences.solid24+=o.solid24;
    scope.occurrences.solid6z+=o.solid6z;
    if(HasCoefficientProducer(o)) ++scope.covered_nodes;
    else ++scope.uncovered_nodes;
  }
  if(scope.occurrences.qeph!=4*scope.qeph_parents||
      scope.occurrences.t3!=3*scope.t3_parents||
      scope.occurrences.qbat!=4*scope.qbat_parents||
      scope.occurrences.type25!=2*scope.type25_connections||
      scope.occurrences.type13!=2*scope.type13_connections||
      scope.occurrences.element_mass!=scope.element_mass_records||
      scope.occurrences.solid18!=8*scope.solid18_parents||
      scope.occurrences.solid24!=8*scope.solid24_parents||
      scope.occurrences.solid6z!=6*scope.solid6z_parents)
    return {S::IdentityMismatch,"Complete producer occurrence count differs"};
  return {};
}
} // namespace tl::fea::coefficient_detail
