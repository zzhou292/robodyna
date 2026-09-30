#pragma once
#include "lib_src/constraints/tied_shell/TiedCinAttachmentModel.h"
#include "lib_utest/qualification/tied_shell_patch/Fixture.h"
namespace cin_test {
namespace tied = tl::constraints::tied_shell;
struct Fixture {
  std::vector<tl::fea::NodalDomainNode> nodes;
  std::vector<tied::CinAttachmentDeclaration> declarations;
  std::vector<tied::KinChkSlave> slaves;
  std::array<std::int32_t,8192> decode{};
  tl::fea::NodalNodeDomain domain;
  tied::PostKinChkResult post;
  Fixture() {
    for (unsigned shape = 0; shape < 2; ++shape) {
      const auto patch = tied_patch_test::Geometry(shape);
      tied::CinAttachmentDeclaration d;
      d.original_nsv_row = 3+6*shape;
      d.ordered_master_rank = 19+shape;
      d.master_source = {tied::CinMasterSourceKind::DeclaredShellElement,400+shape,700+shape};
      d.topology = shape ? tied::CinMasterTopology::TriangleRepeatedThird : tied::CinMasterTopology::Quad;
      const std::uint64_t base = 100+100*shape;
      const unsigned unique = shape ? 3 : 4;
      d.secondary_source_id = base+unique;
      d.reference_positions[0] = patch.secondary_position;
      nodes.push_back({d.secondary_source_id,patch.secondary_position});
      for (unsigned slot = 0; slot < 4; ++slot) {
        const auto local = shape && slot == 3 ? 2 : slot;
        d.master_source_ids[slot] = base+local;
        d.reference_positions[slot+1] = patch.master_position[slot];
        if (slot < unique) nodes.push_back({base+slot,patch.master_position[slot]});
      }
      declarations.push_back(d);
      slaves.push_back({static_cast<std::uint32_t>(d.secondary_source_id),0,{2,7,7,0,0}});
    }
    std::reverse(nodes.begin(),nodes.end());
    if (!domain.Initialize({73,nodes.data(),nodes.size()})) throw std::runtime_error("Tiny domain rejected");
    for (std::size_t code = 0; code < decode.size(); ++code) decode[code] = (code&2) != 0;
    if (!tied::PostKinChk(PostInput(),&post)) throw std::runtime_error("Tiny post-KINCHK rejected");
  }
  tied::KinChkInput PostInput() const {
    return {tied::KinChkProfile::NoWallRbeOrCyclic,tied::ClassificationPhase::InterfaceTaggedBeforeKinChk,
      73,881,{slaves.data(),slaves.size()},{decode.data(),decode.size()}};
  }
  tied::ClassificationView<tied::CinAttachmentDeclaration> Input() const {
    return {declarations.data(),declarations.size()};
  }
};
}
