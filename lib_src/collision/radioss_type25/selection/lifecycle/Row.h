// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Sliding.h"
#include "Keep.h"
namespace tlfea::contact::radioss_type25::selection::lifecycle {
struct RowScratch {
  Occurrence* occurrences=nullptr;
  NativeRawGeometryResult* geometry=nullptr; // Same provisional slot; only enabled entries valid.
  std::size_t occurrence_capacity=0;
  int* sliding_mains=nullptr;std::size_t sliding_capacity=0;
  std::size_t provisional_base=0;
};
struct RowStageResult {
  Report report;
  RowResult value;
  std::size_t occurrence_count=0;
};
struct PreparedRow {
  RowStageResult stage;
  CandidateCache retained_cache;
  int retained_main=0,optimization_main=0,optimization_leave=0;
  std::size_t required_sliding=0;
};
namespace detail {
TL_MATH_HOST_DEVICE inline RowStageResult Failure(RowStageResult result,Status status,
    Stage stage,std::size_t row,std::size_t occurrence=SIZE_MAX) {
  result.report.status=status;result.report.stage=stage;result.report.secondary=row;
  result.report.occurrence=occurrence;return result;
}
TL_MATH_HOST_DEVICE inline Occurrence NewOccurrence(std::size_t row,int main,
    Origin origin,std::size_t source_ordinal) {
  Occurrence result;result.secondary=int(row+1);result.local_main=main;
  result.origin=origin;result.source_ordinal=source_ordinal;return result;
}
TL_MATH_HOST_DEVICE inline PreparedRow PrepareFailure(PreparedRow result,Status status,
    Stage stage,std::size_t row,std::size_t occurrence=SIZE_MAX) {
  result.stage=Failure(result.stage,status,stage,row,occurrence);return result;
}
// Run once into private trial scratch. The batch scans these exact counts and
// admits total CAND_OPT capacity BEFORE continuation/new-impact/force geometry.
TL_MATH_HOST_DEVICE inline PreparedRow PrepareRow(const Input& input,std::size_t row,
    RowScratch scratch,const units_detail::Factors& units) {
  PreparedRow prepared;auto& out=prepared.stage;auto status=BeginRow(input,row,out.value);
  if(status!=Status::Ok)return PrepareFailure(prepared,status,Stage::Begin,row);
  prepared.optimization_main=out.value.history.row.irtlm[0];
  prepared.optimization_leave=out.value.history.row.irtlm[2];
  const auto& csr=input.spatial_by_secondary;
  for(std::size_t i=csr.offsets[row];i<csr.offsets[row+1];++i) {
    const auto raw=csr.entries[i];const auto& candidate=input.spatial[raw];
    if(OptimizedCandidate(input,row,candidate.local_main,prepared.optimization_main,prepared.optimization_leave,units))
      ++out.value.optimized_count;
  }
  ReleaseDeletedMain(out.value);
  NativeRetainedResult retained;
  int& retained_main=prepared.retained_main;
  if(out.value.retained_count) {
    retained_main=out.value.history.row.irtlm[2];
    auto pair=Pair(input,row,retained_main,scratch.provisional_base,units);
    pair.initial_contact_flag=out.value.initial_contact_flag;
    status=EvaluateNativeRetained(input.profile.selection,pair,out.value.history,&retained);
    if(status!=Status::Ok)return PrepareFailure(prepared,status,Stage::Retained,row);
    out.value.history=retained.history;prepared.retained_cache=retained.cache;
    status=PrepareSliding(input.source.mains[retained_main-1],retained.cache,out.value);
    if(status!=Status::Ok)return PrepareFailure(prepared,status,Stage::Sliding,row);
  }
  const auto& normal_csr=input.source.normal_to_main;
  for(int reference:out.value.sliding_reference)if(reference) {
    const auto required=std::size_t(normal_csr.offsets[reference]-normal_csr.offsets[reference-1]);
    if(required>SIZE_MAX-prepared.required_sliding)
      return PrepareFailure(prepared,Status::CapacityExceeded,Stage::Sliding,row);
    prepared.required_sliding+=required;
  }
  status=SlidingMains(input,row,out.value,scratch.sliding_mains,scratch.sliding_capacity);
  if(status!=Status::Ok)return PrepareFailure(prepared,status,Stage::Sliding,row);
  if(out.value.retained_count>SIZE_MAX-out.value.optimized_count||
     out.value.sliding_count>SIZE_MAX-out.value.retained_count-out.value.optimized_count)
    return PrepareFailure(prepared,Status::CapacityExceeded,Stage::Admission,row);
  const auto count=out.value.retained_count+out.value.optimized_count+out.value.sliding_count;
  out.report.required_candidates=count;out.report.count_complete=true;
  out.report.status=Status::Ok;return prepared;
}
TL_MATH_HOST_DEVICE inline RowStageResult CompleteRow(const Input& input,std::size_t row,
    RowScratch scratch,const units_detail::Factors& units,const PreparedRow& prepared) {
  auto out=prepared.stage;if(out.report.status!=Status::Ok)return out;
  const auto count=out.report.required_candidates;
  if(count>scratch.occurrence_capacity||count&&(!scratch.occurrences||!scratch.geometry))
    return Failure(out,Status::CapacityExceeded,Stage::Admission,row);
  const auto& csr=input.spatial_by_secondary;
  auto status=Status::Ok;
  std::size_t used=0;
  if(out.value.retained_count) {
    auto occurrence=NewOccurrence(row,prepared.retained_main,Origin::Retained,row);
    occurrence.cache=prepared.retained_cache;occurrence.cache_initialized=true;
    scratch.occurrences[used++]=occurrence;
  }
  // Re-evaluation uses the SAME immutable pre-release OPTCD snapshot.
  for(std::size_t i=csr.offsets[row];i<csr.offsets[row+1];++i) {
    const auto raw=csr.entries[i];const auto& candidate=input.spatial[raw];
    if(OptimizedCandidate(input,row,candidate.local_main,prepared.optimization_main,prepared.optimization_leave,units))
      scratch.occurrences[used++]=NewOccurrence(row,candidate.local_main,Origin::Spatial,raw);
  }
  for(std::size_t i=0;i<out.value.sliding_count;++i)
    scratch.occurrences[used++]=NewOccurrence(row,scratch.sliding_mains[i],Origin::Sliding,i);
  const int continuation_marker=out.value.history.row.irtlm[0];
  for(std::size_t i=0;i<count;++i) {
    auto& occurrence=scratch.occurrences[i];
    if(continuation_marker>0&&
       continuation_marker!=input.source.mains[occurrence.local_main-1].global_id) {
      auto pair=Continuation(input,row,occurrence.local_main,scratch.provisional_base+i,
          out.value.sliding_reference,units);
      pair.pair.initial_contact_flag=out.value.initial_contact_flag;
      NativeContinuationResult result;
      status=EvaluateNativeContinuation(input.profile.selection,pair,out.value.history,&result);
      if(status!=Status::Ok)return Failure(out,status,Stage::Continuation,row,i);
      out.value.history=result.history;occurrence.cache=result.cache;
      occurrence.cache_initialized=true;++out.value.continuation_count;
    }
  }
  const bool new_impact=out.value.history.row.irtlm[0]<=0;
  if(new_impact)for(std::size_t i=0;i<count;++i) {
    auto& occurrence=scratch.occurrences[i];
    auto pair=NewImpact(input,row,occurrence.local_main,scratch.provisional_base+i,units);
    pair.pair.initial_contact_flag=out.value.initial_contact_flag;
    NativeNewImpactResult result;
    status=EvaluateNativeNewImpact(input.profile.selection,pair,out.value.history,&result);
    if(status!=Status::Ok)return Failure(out,status,Stage::NewImpact,row,i);
    out.value.history=result.history;occurrence.cache=result.cache;
    occurrence.local_main=result.cache.local_main;occurrence.cache_initialized=true;
    ++out.value.new_impact_count;
  }
  for(std::size_t i=0;i<count;++i)if(!scratch.occurrences[i].cache_initialized)
    return Failure(out,Status::UndefinedNativeInput,Stage::Keep,row,i);
  NativeContactRow ended;
  if(EndNativeContact(out.value.history.row,&ended)!=NormalStatus::Ok)
    return Failure(out,Status::InvalidInput,Stage::Keep,row);
  out.value.history.row=ended;
  for(std::size_t i=0;i<count;++i) {
    status=Keep(input.source,scratch.occurrences[i],out.value);
    if(status!=Status::Ok)return Failure(out,status,Stage::Keep,row,i);
  }
  // All KEEPF resets precede ANY selected binding, exactly as MAINF does.
  for(std::size_t i=0;i<count;++i) {
    auto& occurrence=scratch.occurrences[i];scratch.geometry[i]={};
    status=SelectGeometry(input,row,out.value.history,occurrence);
    if(status!=Status::Ok)return Failure(out,status,Stage::GeometryBinding,row,i);
    if(occurrence.selected.enabled) {
      const auto input_geometry=Geometry(input,occurrence,units);
      const auto geometry_status=EvaluateNativeGeometry(input.profile.geometry,input_geometry,&scratch.geometry[i]);
      if(geometry_status!=GeometryStatus::Ok)
        return Failure(out,geometry_status==GeometryStatus::NonfiniteResult?
            Status::NonfiniteResult:Status::UndefinedNativeInput,Stage::GeometryBinding,row,i);
    }
  }
  out.occurrence_count=count;out.report.status=Status::Ok;out.report.stage=Stage::GeometryBinding;return out;
}
// Convenience for bounded single-row coupons; multi-row callers first prepare
// EVERY row and admit the complete count before invoking CompleteRow.
TL_MATH_HOST_DEVICE inline RowStageResult StageRow(const Input& input,std::size_t row,
    RowScratch scratch,const units_detail::Factors& units) {
  const auto prepared=PrepareRow(input,row,scratch,units);
  return CompleteRow(input,row,scratch,units,prepared);
}

} // namespace detail
} // namespace tlfea::contact::radioss_type25::selection::lifecycle
