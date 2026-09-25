// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../../RadiossType25FixedMainStartup.h"
#include "Internal.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::startup {
Report BuildFixedMain(const Input& input,const Snapshot& starter,const FixedMainInput& active,
    Limits limits,tl::util::HostArena& output,tl::util::HostArena& scratch,FixedMainView* published) noexcept {
  detail::Layout layout;auto report=detail::MakeLayout(input.node_count,input.primary_count,limits,layout);
  if(report.status!=Status::Ok)return report;
  if(output.bytes()<layout.forecast.ready_output_bytes || scratch.bytes()<layout.forecast.ready_scratch_bytes)
    return {Status::ResourceLimit};
  report=detail::CheckInput(input,layout,output,scratch,published,sizeof(FixedMainView));
  if(report.status!=Status::Ok)return report;
  report=detail::CheckSnapshot(input,starter,active,output,scratch,published);
  if(report.status!=Status::Ok)return report;
  auto data=detail::Construct(scratch,layout.output);
  auto* points=scratch.Construct<Vector>(layout.points);
  auto* identities=scratch.Construct<detail::Identity>(layout.identities);
  auto* faces=scratch.Construct<detail::FaceKey>(layout.face_keys);
  report=detail::Expand(input,data,points,identities,faces);
  if(report.status!=Status::Ok)return report;
  std::copy_n(starter.mains,starter.main_count,data.mains);
  std::copy_n(starter.starter.face_normals,4*starter.main_count,data.normals);
  std::copy_n(starter.starter.references,starter.starter.reference_count,data.references);
  const auto count=4*starter.main_count;
  const tl::util::ArenaRegion previous_region{layout.edges.offset,count,count*sizeof(StoredNormal)};
  auto* previous=scratch.Construct<StoredNormal>(previous_region);
  auto* first_slot=scratch.Construct<int>(layout.parents);
  auto* second_slot=scratch.Construct<int>(layout.tags);
  report=detail::FixedNormals(points,data,input.primary_count,starter.main_count,
      starter.starter.reference_count,previous,first_slot,second_slot);
  if(report.status!=Status::Ok)return report;
  tl::util::BoundedArenaLayout target(limits.max_output_bytes);
  tl::util::ArenaRegion normal_region,reference_region;
  if(!target.Append<StoredNormal>(count,normal_region) || !target.Append<NormalReference>(count,reference_region))
    return {Status::ResourceLimit};
  auto* normals=output.Construct<StoredNormal>(normal_region);
  auto* references=output.Construct<NormalReference>(reference_region);
  std::copy_n(data.normals,count,normals);
  std::copy_n(data.references,count,references);
  *published={{normals,references,starter.starter.reference_count},input.source_generation};
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::startup
