// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Admission.h"
#include <algorithm>
#include <new>
#include <stdexcept>
#include <vector>
namespace tlfea::contact::radioss_type25::selection::lifecycle {
struct HostResult {
  std::vector<RowResult> rows;
  std::vector<Occurrence> occurrences;
  std::vector<NativeRawGeometryResult> geometry; // Enabled selected entries only.
};
namespace detail {
struct OrderedSlot {
  std::size_t row=0,slot=0;
};
inline bool Before(const Occurrence& a,std::size_t arow,
    const Occurrence& b,std::size_t brow) {
  if(a.origin!=b.origin)return static_cast<unsigned>(a.origin)<static_cast<unsigned>(b.origin);
  if(a.origin==Origin::Sliding&&arow!=brow)return arow<brow;
  return a.source_ordinal<b.source_ordinal;
}
inline bool AddBytes(std::size_t count,std::size_t size,std::size_t& total) {
  if(count>SIZE_MAX/size||count*size>SIZE_MAX-total)return false;
  total+=count*size;return true;
}
} // namespace detail
// Bounded host adapter of the same one-writer numerical row operations used by
// CUDA. It never publishes physical state; the caller's prior result is replaced
// only after every row, selected binding and raw-geometry evaluation succeeds.
inline Report EvaluateNativeLifecycleHost(const Input& input,Limits limits,HostResult* output) {
  Report report;
  if(!output)return report;
  if(!detail::CurrentNormalsDisjoint(input,output,std::size_t{1}))return report;
  const auto count=input.source.secondary_count;
  std::size_t minimum_bytes=0;
  if(count>limits.rows||count==SIZE_MAX||
     !detail::AddBytes(count,sizeof(detail::RowRequirements)+sizeof(PreparedRow)+sizeof(RowResult),minimum_bytes)||
     !detail::AddBytes(count+1,2*sizeof(std::size_t),minimum_bytes)||minimum_bytes>limits.scratch_bytes) {
    report.status=Status::CapacityExceeded;return report;
  }
  report.status=detail::Validate(input);if(report.status!=Status::Ok)return report;
  try {
    std::vector<detail::RowRequirements> needs(count);
    std::vector<std::size_t> offsets(count+1),sliding_offsets(count+1);
    std::size_t occurrence_upper=0,sliding_upper=0;
    for(std::size_t row=0;row<count;++row) {
      needs[row]=detail::Requirements(input,row);
      if(needs[row].status!=Status::Ok){report.status=needs[row].status;report.secondary=row;return report;}
      if(needs[row].occurrences>SIZE_MAX-occurrence_upper||needs[row].sliding>SIZE_MAX-sliding_upper) {
        report.status=Status::CapacityExceeded;return report;
      }
      occurrence_upper+=needs[row].occurrences;sliding_upper+=needs[row].sliding;
      offsets[row+1]=occurrence_upper;sliding_offsets[row+1]=sliding_upper;
    }
    std::size_t bytes=0;
    if(sliding_upper>limits.sliding_scratch||
       !detail::AddBytes(count,sizeof(detail::RowRequirements)+sizeof(PreparedRow)+sizeof(RowResult),bytes)||
       !detail::AddBytes(count+1,2*sizeof(std::size_t),bytes)||
       !detail::AddBytes(occurrence_upper,sizeof(Occurrence)+sizeof(NativeRawGeometryResult)+sizeof(detail::OrderedSlot),bytes)||
       !detail::AddBytes(sliding_upper,sizeof(int),bytes)||
       !detail::AddBytes(limits.candidates,sizeof(Occurrence)+sizeof(NativeRawGeometryResult),bytes)||
       bytes>limits.scratch_bytes) {report.status=Status::CapacityExceeded;return report;}
    std::vector<PreparedRow> prepared(count);
    std::vector<Occurrence> occurrences(occurrence_upper);
    std::vector<NativeRawGeometryResult> geometry(occurrence_upper);
    std::vector<int> sliding(sliding_upper);
    units_detail::Factors units;detail::Factors(input.current,units);
    std::size_t actual=0;
    for(std::size_t row=0;row<count;++row) {
      RowScratch scratch{
        needs[row].occurrences?occurrences.data()+offsets[row]:nullptr,
        needs[row].occurrences?geometry.data()+offsets[row]:nullptr,needs[row].occurrences,
        needs[row].sliding?sliding.data()+sliding_offsets[row]:nullptr,needs[row].sliding,offsets[row]};
      prepared[row]=detail::PrepareRow(input,row,scratch,units);
      if(prepared[row].stage.report.status!=Status::Ok)return prepared[row].stage.report;
      const auto required=prepared[row].stage.report.required_candidates;
      if(required>SIZE_MAX-actual){report.status=Status::CapacityExceeded;return report;}
      actual+=required;
    }
    report.required_candidates=actual;report.count_complete=true;
    if(actual>limits.candidates){report.status=Status::CapacityExceeded;return report;}
    for(std::size_t row=0;row<count;++row) {
      RowScratch scratch{
        needs[row].occurrences?occurrences.data()+offsets[row]:nullptr,
        needs[row].occurrences?geometry.data()+offsets[row]:nullptr,needs[row].occurrences,
        needs[row].sliding?sliding.data()+sliding_offsets[row]:nullptr,needs[row].sliding,offsets[row]};
      prepared[row].stage=detail::CompleteRow(input,row,scratch,units,prepared[row]);
      if(prepared[row].stage.report.status!=Status::Ok) {
        auto failed=prepared[row].stage.report;
        failed.required_candidates=actual;failed.count_complete=true;return failed;
      }
    }
    std::vector<detail::OrderedSlot> order;order.reserve(actual);
    for(std::size_t row=0;row<count;++row)
      for(std::size_t i=0;i<prepared[row].stage.occurrence_count;++i)order.push_back({row,offsets[row]+i});
    std::sort(order.begin(),order.end(),[&](const auto& a,const auto& b) {
      return detail::Before(occurrences[a.slot],a.row,occurrences[b.slot],b.row);
    });
    HostResult result;result.rows.resize(count);result.occurrences.resize(actual);result.geometry.resize(actual);
    for(std::size_t row=0;row<count;++row)result.rows[row]=prepared[row].stage.value;
    for(std::size_t i=0;i<actual;++i) {
      result.occurrences[i]=occurrences[order[i].slot];
      result.occurrences[i].cache.occurrence=i; // Final dense native occurrence order.
      result.geometry[i]=geometry[order[i].slot];
    }
    *output=std::move(result);report.status=Status::Ok;report.stage=Stage::GeometryBinding;
    return report;
  } catch(const std::bad_alloc&) {
    report.status=Status::CapacityExceeded;return report;
  } catch(const std::length_error&) {
    report.status=Status::CapacityExceeded;return report;
  }
}
} // namespace tlfea::contact::radioss_type25::selection::lifecycle
