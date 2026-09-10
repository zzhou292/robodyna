#pragma once
#include "lib_src/elements/qeph/QephBatchStorage.h"
#include "lib_src/elements/t3/T3BatchStorage.h"
#include "lib_src/elements/ShellBatchPublicationStorage.h"
#include <gtest/gtest.h>
namespace active_shell_test {
namespace fe=tl::fea;
inline std::size_t PlasticBytes(std::size_t parents,std::size_t points) {
  if(!points) return 0;
  fe::shell_batch_plasticity_detail::Layout layout;
  EXPECT_TRUE(layout.Initialize(parents,points,fe::MaxShellResidentDeviceBytes)); return layout.bytes;
}
template<class Layout> std::size_t FamilyBytes(std::size_t parents,std::size_t nodes,std::size_t points=0) {
  Layout layout;
  EXPECT_TRUE(layout.Initialize(parents,nodes,fe::MaxShellResidentDeviceBytes));
  return layout.bytes+PlasticBytes(parents,points);
}
inline std::size_t QBytes(std::size_t parents,std::size_t nodes,std::size_t points=0) {
  return FamilyBytes<fe::qeph::batch_detail::Layout>(parents,nodes,points);
}
inline std::size_t TBytes(std::size_t parents,std::size_t nodes,std::size_t points=0) {
  return FamilyBytes<fe::t3::batch_detail::Layout>(parents,nodes,points);
}
inline std::size_t PublicationBytes(std::size_t nodes) {
  fe::shell_publication_detail::Layout layout;
  EXPECT_TRUE(layout.Initialize(nodes,fe::MaxShellResidentDeviceBytes)); return layout.bytes;
}
}
