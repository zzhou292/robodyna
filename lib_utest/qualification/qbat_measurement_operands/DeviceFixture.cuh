// SPDX-License-Identifier: MIT
#pragma once
#include "Fixture.h"
#include "../qbat_mapped_gather/SerialCandidate.cuh"
#include "../qbat_resident/ResultValues.h"
#include <cuda_runtime.h>

namespace qbat_measurement_test {
struct DeviceFixture {
  Fixture source;
  b::Storage* storage=nullptr;
  g::Inputs* input=nullptr;
  explicit DeviceFixture(std::size_t parents=4) : source(parents) {
    EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&storage),source.layout.bytes),cudaSuccess);
    EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&input),sizeof(g::Inputs)),cudaSuccess);
  }
  ~DeviceFixture() { cudaFree(input); cudaFree(storage); }
  void Upload(unsigned epoch=2) {
    source.Reset(epoch);
    Restore();
  }
  void Restore() {
    std::memcpy(storage,source.arena.data(),source.layout.bytes);
    *storage=source.layout.Rebase(*source.host,storage);
    *input=source.input;
  }
  void Evaluate(bool reference,unsigned epoch=2,bool candidate=false,bool mapped=true) {
    const auto view=input->Prepared(epoch);
    const auto identity=g::Identity(epoch);
    std::vector<unsigned char> before[2];
    if (!candidate) {
      for (unsigned slab=0; slab<2; ++slab) {
        before[slab].resize(source.count*sizeof(q::BatchResult));
        std::memcpy(before[slab].data(),storage->slab[slab].element,before[slab].size());
      }
    }
    if (candidate) {
      if (reference) g::serial_candidate::LaunchCandidate(storage,&storage->slab[0],
          &storage->slab[1],view,identity,source.count);
      else b::LaunchCandidate(storage,&storage->slab[0],&storage->slab[1],view,
          identity,source.count,mapped?g::Nodes:0);
    } else if (reference) {
      g::serial_candidate::FinalizeCandidate<<<1,1>>>(storage,&storage->slab[0],
          &storage->slab[1],view,identity);
    } else {
      b::LaunchMappedMeasurements(storage,&storage->slab[0],&storage->slab[1],
          view,identity,g::Nodes);
    }
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    if (!candidate) {
      for (unsigned slab=0; slab<2; ++slab)
        EXPECT_EQ(std::memcmp(before[slab].data(),storage->slab[slab].element,before[slab].size()),0);
    }
  }
};
inline void Same(const DeviceFixture& actual,const DeviceFixture& expected) {
  const auto& a=actual.storage->control;
  const auto& b=expected.storage->control;
  EXPECT_EQ(a.status,b.status);
  EXPECT_EQ(a.element_status,b.element_status);
  EXPECT_EQ(a.element,b.element);
  EXPECT_EQ(a.node,b.node);
  EXPECT_TRUE(q::batch_detail::SameDiagnostics(a.diagnostics,b.diagnostics));
  for (unsigned slab=0; slab<2; ++slab) {
    for (std::size_t parent=0; parent<actual.source.count; ++parent) {
      const auto& ax=actual.storage->slab[slab].element[parent];
      const auto& bx=expected.storage->slab[slab].element[parent];
      if (!q::batch_detail::ValidBool(ax.history.element_active) ||
          !q::batch_detail::ValidBool(bx.history.element_active)) {
        // The injected invalid encoding must not be decoded as a C++ bool by
        // this comparison. Each complete slab was checked unchanged above.
        EXPECT_EQ(*reinterpret_cast<const unsigned char*>(&ax.history.element_active),
            *reinterpret_cast<const unsigned char*>(&bx.history.element_active));
        continue;
      }
      const auto x=qbat_resident_test::ResultValues(actual.storage->slab[slab].element[parent]);
      const auto y=qbat_resident_test::ResultValues(expected.storage->slab[slab].element[parent]);
      ASSERT_EQ(x.size(),y.size());
      for (std::size_t channel=0; channel<x.size(); ++channel)
        EXPECT_EQ(Bits(x[channel]),Bits(y[channel]))<<slab<<":"<<parent<<":"<<channel;
    }
  }
}
} // namespace qbat_measurement_test
