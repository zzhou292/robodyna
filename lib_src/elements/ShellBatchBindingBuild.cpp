// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellBindingInventory.h"
#include "qeph/QephStartup.h"
#include "t3/T3Startup.h"
#include "qbat/QbatReference.h"

namespace tl::fea {
using namespace shell_binding_detail;
ShellBindingReport ShellBatchBinding::Build(const ShellBatchCollectionInput& input,bool legacy,
    const ShellQbatBindingInput* qbats,std::size_t qbat_count) {
  auto at=[](ShellBindingReport report,std::size_t parent) {
    report.parent_index=parent; return report;
  };
  ParentIdentityIndex parent_ids;
  if(!legacy) parent_ids.Prepare(input.qeph_count+input.t3_count+qbat_count,[&](std::size_t i) {
    if(i<input.qeph_count) return input.qeph[i].source_parent_id;
    i-=input.qeph_count;
    if(i<input.t3_count) return input.t3[i].source_parent_id;
    return qbats[i-input.t3_count].source_parent_id;
  });
  auto parent_identity=[&](std::uint64_t id,ShellBindingFamily family,std::size_t occurrence) {
    if(!legacy) {
      if(id==0) return Error(ShellBindingStatus::InvalidParentIdentity,"Collection parent ID is zero",family);
      if(parent_ids.First(id)!=occurrence)
        return Error(ShellBindingStatus::InvalidParentIdentity,"Collection parent ID is repeated",family);
    }
    return ShellBindingReport{};
  };
  // Preserve the pair's ordering: all connectivity checks, all native startup,
  // all identities/coverage, then native mass reduction and final publication.
  for(std::size_t i=0;i<input.qeph_count;++i) {
    auto report=CheckConnectivity(input.qeph[i].nodes,input.node_count,ShellBindingFamily::Qeph);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
    report=parent_identity(input.qeph[i].source_parent_id,ShellBindingFamily::Qeph,i);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
  }
  for(std::size_t i=0;i<input.t3_count;++i) {
    auto report=CheckConnectivity(input.t3[i].nodes,input.node_count,ShellBindingFamily::T3);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
    report=parent_identity(input.t3[i].source_parent_id,ShellBindingFamily::T3,input.qeph_count+i);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
  }
  for(std::size_t i=0;i<qbat_count;++i) {
    auto report=CheckConnectivity(qbats[i].nodes,input.node_count,ShellBindingFamily::Qbat);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
    report=parent_identity(qbats[i].source_parent_id,ShellBindingFamily::Qbat,input.qeph_count+input.t3_count+i);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
  }
  Data next;
  next.qeph.Resize(input.qeph_count); next.t3.Resize(input.t3_count);
  next.qbat.Resize(qbat_count);
  next.nodes.Resize(input.node_count);
  next.inventory.words_.Resize(InventoryWords(input.qeph_count,input.t3_count,qbat_count,legacy));
  next.qeph_count=input.qeph_count; next.t3_count=input.t3_count; next.node_count=input.node_count;
  next.qbat_count=qbat_count;
  for(std::size_t i=0;i<input.qeph_count;++i) {
    auto& parent=next.qeph[i];
    const auto status=qeph::InitializeReference(input.qeph[i].reference,parent.reference);
    if(status!=qeph::Status::kSuccess) {
      auto report=Error(ShellBindingStatus::InvalidQephReference,"QEPH startup rejected its typed input",ShellBindingFamily::Qeph);
      report.qeph_status=status; return at(report,i);
    }
    parent.nodes=input.qeph[i].nodes; parent.source_id=input.qeph[i].source_parent_id;
  }
  for(std::size_t i=0;i<input.t3_count;++i) {
    auto& parent=next.t3[i];
    const auto status=t3::InitializeReference(input.t3[i].reference,parent.reference);
    if(status!=t3::Status::kSuccess) {
      auto report=Error(ShellBindingStatus::InvalidT3Reference,"T3 startup rejected its typed input",ShellBindingFamily::T3);
      report.t3_status=status; return at(report,i);
    }
    parent.nodes=input.t3[i].nodes; parent.source_id=input.t3[i].source_parent_id;
  }
  for(std::size_t i=0;i<qbat_count;++i) {
    auto& parent=next.qbat[i];
    const auto status=qbat::InitializeReference(qbats[i].reference,parent.reference);
    if(status!=qbat::Status::kSuccess) {
      auto report=Error(ShellBindingStatus::InvalidQbatReference,"QBAT startup rejected its typed input",ShellBindingFamily::Qbat);
      report.qbat_status=status;
      return at(report,i);
    }
    parent.nodes=qbats[i].nodes;
    parent.source_id=qbats[i].source_parent_id;
  }
  NodeSeen seen; seen.Resize(next.node_count);
  NodeIdentityIndex node_ids;
  node_ids.Prepare(4*next.qeph_count+3*next.t3_count+4*next.qbat_count,[&](std::size_t i)->std::uint64_t {
    if(i<4*next.qeph_count) return next.qeph[i/4].reference.input.node_ids[i%4];
    const auto t=i-4*next.qeph_count;
    if(t<3*next.t3_count) return next.t3[t/3].reference.input.node_ids[t%3];
    const auto b=t-3*next.t3_count;
    return next.qbat[b/4].reference.input().quadrilateral.node_ids[b%4];
  });
  for(std::size_t i=0;i<next.qeph_count;++i) {
    const auto& parent=next.qeph[i];
    const auto report=RegisterNodes(parent.reference.input,parent.nodes,ShellBindingFamily::Qeph,
        next.nodes.data(),seen.data(),node_ids,4*i);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
  }
  for(std::size_t i=0;i<next.t3_count;++i) {
    const auto& parent=next.t3[i];
    const auto report=RegisterNodes(parent.reference.input,parent.nodes,ShellBindingFamily::T3,
        next.nodes.data(),seen.data(),node_ids,4*next.qeph_count+3*i);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
  }
  for(std::size_t i=0;i<next.qbat_count;++i) {
    const auto& parent=next.qbat[i];
    const auto report=RegisterNodes(parent.reference.input().quadrilateral,parent.nodes,ShellBindingFamily::Qbat,
        next.nodes.data(),seen.data(),node_ids,4*next.qeph_count+3*next.t3_count+4*i);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
  }
  for(std::size_t n=0;n<next.node_count;++n) if(!seen[n])
    return Error(ShellBindingStatus::InvalidConnectivity,"Declared global node is uncovered",
                 ShellBindingFamily::None,NoShellBindingNode,n);
  for(std::size_t i=0;i<next.qeph_count;++i) {
    const auto& parent=next.qeph[i];
    const auto report=Accumulate(parent.reference,parent.nodes,ShellBindingFamily::Qeph,next.nodes.data(),next.totals);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
  }
  for(std::size_t i=0;i<next.t3_count;++i) {
    const auto& parent=next.t3[i];
    const auto report=Accumulate(parent.reference,parent.nodes,ShellBindingFamily::T3,next.nodes.data(),next.totals);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
  }
  for(std::size_t i=0;i<next.qbat_count;++i) {
    const auto& parent=next.qbat[i];
    const auto& reference=parent.reference.quadrilateral();
    const auto report=Accumulate(reference,parent.nodes,ShellBindingFamily::Qbat,next.nodes.data(),next.totals);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
    // Diagnostic subtotal only. Each term was already added to the complete
    // node/collection ledger above; never reconstruct authoritative TOTAL J.
    for(std::size_t j=0;j<4;++j) {
      const ShellBindingMass term{reference.nodal_mass[j],reference.isotropic_inertia[j],
          reference.physical_inertia[j],reference.added_inertia[j]};
      if(!AddMass(next.qbat_totals,term))
        return at(Error(ShellBindingStatus::NonfiniteMass,"QBAT subtotal is not representable",
            ShellBindingFamily::Qbat,j,parent.nodes[j]),i);
    }
  }
  static_assert(ShellBatchInventory::WordCount==2+(2+5*4+5)+(2+5*3+5),"Complete placed pair inventory");
  static_assert(ShellBatchInventory::Capacity>=4+MaxShellCollectionParents*(3+5*4+5),"Complete collection inventory");
  auto& words=next.inventory.words_;
  std::size_t cursor=0;
  words[cursor++]=legacy?3:(qbat_count?5:4); // In-process encoding only, never a file schema.
  words[cursor++]=next.node_count;
  if(!legacy) { words[cursor++]=next.qeph_count; words[cursor++]=next.t3_count; }
  if(qbat_count) words[cursor++]=next.qbat_count;
  for(std::size_t i=0;i<next.qeph_count;++i) {
    const auto& parent=next.qeph[i];
    AppendInventory(words,cursor,4,parent.reference.input,parent.nodes,parent.source_id,legacy);
  }
  for(std::size_t i=0;i<next.t3_count;++i) {
    const auto& parent=next.t3[i];
    AppendInventory(words,cursor,3,parent.reference.input,parent.nodes,parent.source_id,legacy);
  }
  for(std::size_t i=0;i<next.qbat_count;++i) {
    const auto& parent=next.qbat[i];
    AppendQbatInventory(words,cursor,parent.reference.input(),parent.nodes,parent.source_id);
  }
  next.inventory.word_count_=cursor;
  data_=next; prepared_=true;
  return {};
}
} // namespace tl::fea
