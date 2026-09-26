#include "lib_src/collision/RadiossType25SurfaceSource.h"
int main() {
  namespace s=tlfea::contact::radioss_type25::source_surfaces;
  const s::Solid solid{101,10,s::SolidTopology::Hex8,{0,1,2,3,4,5,6,7}};
  const std::uint64_t part=10;
  s::Input input;input.phase=s::ReaderPhase::BeforeGroupingAndInitia;input.node_count=8;
  input.solids=&solid;input.solid_count=1;input.clause.part_ids=&part;input.clause.part_count=1;
  s::Forecast forecast;
  if(s::Preflight(input,{},forecast).status!=s::Status::Ok)return 1;
  tl::util::HostArena output,scratch;
  if(!output.Initialize(forecast.output_bytes)||!scratch.Initialize(forecast.scratch_bytes))return 2;
  s::Snapshot result;
  if(s::Build(input,{},output,scratch,&result).status!=s::Status::Ok)return 3;
  return result.face_count==6&&result.surface_solid_flags[0]==1?0:4;
}
