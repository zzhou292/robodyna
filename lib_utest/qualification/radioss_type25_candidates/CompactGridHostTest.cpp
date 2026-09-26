// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/radioss_type25/candidates/Layout.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <limits>
#include <string>
namespace candidate_grid_host {
namespace c=tlfea::contact::radioss_type25::candidates;
namespace d=c::detail;
TEST(CompactGridHost, MonotoneQuantizationAndUniqueKeysPreserveInclusiveBounds) {
  for(std::uint64_t active:{0ull,1ull,2ull,8ull,9ull,512ull,359721ull,524288ull}) {
    const auto n=d::GridResolution(active);EXPECT_GE(n,1u);EXPECT_LE(n,128u);EXPECT_EQ(n&(n-1),0u);
    EXPECT_GE(std::uint64_t(n)*n*n,active);
  }
  for(double shift:{0.,-4.,0x1p20}) {
    const auto low=shift-1.5,high=shift+1.5,span=high-low;constexpr unsigned cells=4;
    EXPECT_EQ(d::GridCell(-std::numeric_limits<double>::max(),low,high,span,cells),0u);
    EXPECT_EQ(d::GridCell(std::numeric_limits<double>::max(),low,high,span,cells),cells-1);
    unsigned previous=0;
    for(unsigned i=0;i<=cells;++i) {
      const double edge=low+.75*i;
      for(double point:{std::nextafter(edge,-INFINITY),edge,std::nextafter(edge,INFINITY)}) {
        const auto cell=d::GridCell(point,low,high,span,cells);EXPECT_GE(cell,previous);EXPECT_LT(cell,cells);previous=cell;
      }
    }
  }
  const std::array<unsigned,3> shapes[]{{1,1,1},{4,2,8},{128,128,128}};
  for(const auto& shape:shapes) {
    double previous=-1;
    for(unsigned z=0;z<shape[2];++z)for(unsigned y=0;y<shape[1];++y) {
      EXPECT_EQ(d::GridKey(shape.data(),0,y,z),previous+1);
      previous=d::GridKey(shape.data(),shape[0]-1,y,z);
    }
    EXPECT_EQ(previous,double(std::uint64_t(shape[0])*shape[1]*shape[2]-1));
  }
  for(double point:{-std::numeric_limits<double>::max(),0.,std::numeric_limits<double>::max()}) {
    EXPECT_EQ(d::GridCell(point,0.,0.,0.,1),0u);
    EXPECT_EQ(d::GridCell(point,-std::numeric_limits<double>::max(),std::numeric_limits<double>::max(),INFINITY,1),0u);
  }
}
TEST(CompactGridHost, ExactSourceSizedStorageForecastAndStrategyCapsAreFailureAtomic) {
  // Actual-size arithmetic shape only, not an invented source admission.
  d::StorageShape shape{376934,359721,341504,0};c::Limits limits;
  d::Layout legacy,compact;ASSERT_EQ(d::MakeStorageLayout(shape,limits,4096,512,legacy),c::Status::Ok);
  EXPECT_EQ(legacy.forecast.index_device_bytes,0u);
  limits.strategy=c::EnumerationStrategy::CompactGrid;limits.max_encounters=std::size_t{16}<<20;
  ASSERT_EQ(d::MakeStorageLayout(shape,limits,4096,512,compact),c::Status::Ok);
  const auto expected=16*(shape.mains+1)+4*limits.max_encounters+sizeof(d::GridControl);
  EXPECT_EQ(compact.forecast.index_device_bytes,expected);
  EXPECT_EQ(compact.forecast.device_bytes,legacy.forecast.device_bytes+expected);
  EXPECT_EQ(compact.forecast.cub_bytes,legacy.forecast.cub_bytes);
  RecordProperty("self_paired_index_device_bytes",std::to_string(2*expected));
  limits.max_device_bytes=compact.forecast.device_bytes;d::Layout exact;
  ASSERT_EQ(d::MakeStorageLayout(shape,limits,4096,512,exact),c::Status::Ok);
  --limits.max_device_bytes;d::Layout sentinel;sentinel.forecast.device_bytes=987;
  EXPECT_EQ(d::MakeStorageLayout(shape,limits,4096,512,sentinel),c::Status::ResourceLimit);
  EXPECT_EQ(sentinel.forecast.device_bytes,987u);
  limits.max_device_bytes=c::Limits{}.max_device_bytes;limits.max_encounters=0;
  EXPECT_EQ(d::MakeStorageLayout(shape,limits,4096,512,sentinel),c::Status::ResourceLimit);
  limits.max_encounters=d::MaximumCompactEncounters+1;
  EXPECT_EQ(d::MakeStorageLayout(shape,limits,4096,512,sentinel),c::Status::ResourceLimit);
  limits.strategy=c::EnumerationStrategy::LegacyAxisSweep;limits.max_encounters=1;
  EXPECT_EQ(d::MakeStorageLayout(shape,limits,4096,512,sentinel),c::Status::InvalidInput);
  limits.strategy=static_cast<c::EnumerationStrategy>(99);limits.max_encounters=0;
  EXPECT_EQ(d::MakeStorageLayout(shape,limits,4096,512,sentinel),c::Status::UnsupportedProfile);
  EXPECT_EQ(sentinel.forecast.device_bytes,987u);
}
}
