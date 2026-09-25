// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25FixedMainStartup.h"
int main() {
  namespace n=tlfea::contact::radioss_type25;
  namespace s=n::startup;
  const std::uint64_t ids[]{9,100000000000ull,3,7};
  const double x[]{0,0,0,1,0,0,1,1,0,0,1,0};
  const s::PrimaryFace primary{99,n::ShellLayout::Quad4,{0,1,2,3}};
  s::Input input; input.profile=s::Profile::OrdinaryExteriorFixedMain;
  input.node_source_ids=ids; input.node_count=4; input.positions={x,4,3,1};
  input.primary=&primary; input.primary_count=1; input.source_generation=1;
  const auto plan=s::Preflight(4,1);
  tl::util::HostArena output,scratch,ready_output,ready_scratch;
  if(plan.status!=s::Status::Ok || !output.Initialize(plan.output_bytes) ||
      !scratch.Initialize(plan.scratch_bytes) || !ready_output.Initialize(plan.ready_output_bytes) ||
      !ready_scratch.Initialize(plan.ready_scratch_bytes))return 1;
  s::Snapshot snapshot; s::FixedMainView ready;
  if(s::BuildStarter(input,{},output,scratch,&snapshot).status!=s::Status::Ok)return 2;
  const double coefficient[]{2,3};
  if(s::BuildFixedMain(input,snapshot,{coefficient,2},{},ready_output,ready_scratch,&ready).status!=s::Status::Ok)return 3;
  if(snapshot.main_count!=2 || snapshot.primary_to_partner[0]!=2 ||
      snapshot.expanded_to_primary[1]!=0 || snapshot.mains[1].nodes[0]!=1 ||
      snapshot.starter.references[0].boundary!=1 || ready.normals.references[0].boundary!=2)return 4;
  return 0;
}
