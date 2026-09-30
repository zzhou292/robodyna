#include "lib_src/collision/RadiossType25Search.h"
#include "lib_src/collision/radioss_type25/search/Values.h"
#include <gtest/gtest.h>
TEST(Type25SearchHeader, PureOZeroForecastAndValuesNeedNoGpuExecutionLibrary) {
  namespace s=tlfea::contact::radioss_type25::search;
  const std::uint32_t secondary[]{0},main[]{1};
  s::Source source;source.stamp={1,2,3};source.units={.001,1000,1};source.margin=1;
  source.physical_nodes=2;source.secondary_nodes=secondary;source.secondaries=1;source.main_nodes=main;source.mains=1;
  s::Forecast forecast;EXPECT_EQ(s::Maintenance::Preflight(source,{},forecast),s::Status::Ok);
  EXPECT_GT(forecast.device_bytes,0u);EXPECT_EQ(forecast.reference_positions,2u);
  s::Extrema empty;s::Budget result;EXPECT_EQ(s::EvaluateBudget(empty,1,0,false,result),s::Status::UnsupportedLifecycle);
}
