#include "NativeSupport.h"
#include <cuda_runtime.h>

namespace law90_point_test {
constexpr unsigned MaximumSteps = 321;
struct DeviceSeries {
  law::PreparationInput material_input{};
  double x[28]{}, y[28]{};
  unsigned curve_count = 0, step_count = 0, fault_step = 0;
  law::PointKinematics input[MaximumSteps]{};
  law::PointResult accepted{};
  double values[MaximumSteps][21]{};
  std::uint32_t cursors[MaximumSteps][3]{};
  int error = 0;
  bool preserved = false;
};
__device__ bool SamePoint(const law::PointResult& result, const double* before,
                         const std::uint32_t* cursors) {
  double after[21];
  Pack(result, after);
  for (unsigned k = 0; k < 21; ++k)
    if (__double_as_longlong(after[k]) != __double_as_longlong(before[k])) return false;
  for (unsigned k = 0; k < 3; ++k)
    if (result.history.cursor[k] != cursors[k]) return false;
  return true;
}
__global__ void Evaluate(DeviceSeries* series) {
  law::PreparedMaterial material;
  const law::CurveView curve{series->x, series->y, series->curve_count};
  if (law::PrepareSI(series->material_input, curve, material) != law::Status::Ok) {
    series->error = 1;
    return;
  }
  for (unsigned step = 0; step < series->step_count; ++step) {
    const double time = step*1e-4;
    if (step == series->fault_step) {
      double before[21];
      Pack(series->accepted, before);
      std::uint32_t cursors[3];
      for (unsigned k = 0; k < 3; ++k) cursors[k] = series->accepted.history.cursor[k];
      auto invalid = series->input[step];
      invalid.engineering_rate_s_inverse[5] = HUGE_VAL;
      const auto input_status = law::UpdatePointSI(material, series->accepted.history, invalid, time, series->accepted);
      bool unchanged = input_status == law::PointStatus::InvalidInput && SamePoint(series->accepted, before, cursors);
      auto history = series->accepted.history;
      history.cursor[2] = 0xffffffffu;
      const auto cursor_status = law::UpdatePointSI(material, history, series->input[step], time, series->accepted);
      unchanged = unchanged && cursor_status == law::PointStatus::InvalidCursor && SamePoint(series->accepted, before, cursors);
      const double ordinate = series->y[series->curve_count-1];
      series->y[series->curve_count-1] = HUGE_VAL;
      const auto curve_status = law::UpdatePointSI(material, series->accepted.history, series->input[step], time, series->accepted);
      unchanged = unchanged && curve_status == law::PointStatus::InvalidCurve && SamePoint(series->accepted, before, cursors);
      series->y[series->curve_count-1] = ordinate;
      series->preserved = unchanged;
    }
    const auto status = step == 0
        ? law::InitializePointSI(material, series->input[step], series->accepted)
        : law::UpdatePointSI(material, series->accepted.history, series->input[step], time, series->accepted);
    if (status != law::PointStatus::Ok) {
      series->error = 100 + static_cast<int>(step);
      return;
    }
    Pack(series->accepted, series->values[step]);
    for (unsigned k = 0; k < 3; ++k) series->cursors[step][k] = series->accepted.history.cursor[k];
  }
}

void CheckDevice(DeviceSeries& series) {
  const law::CurveView curve{series.x, series.y, series.curve_count};
  const auto native_prepared = NativePrepared(series.material_input, curve);
  DeviceSeries* device = nullptr;
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device), sizeof(DeviceSeries)), cudaSuccess);
  struct Release { DeviceSeries* p; ~Release() { cudaFree(p); } } release{device};
  ASSERT_EQ(cudaMemcpy(device, &series, sizeof(series), cudaMemcpyHostToDevice), cudaSuccess);
  Evaluate<<<1, 1>>>(device);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&series, device, sizeof(series), cudaMemcpyDeviceToHost), cudaSuccess);
  ASSERT_EQ(series.error, 0);
  ASSERT_TRUE(series.preserved);
  NativeState native;
  for (unsigned step = 0; step < series.step_count; ++step) {
    AdvanceNative(native_prepared.data(), curve, series.input[step], step*1e-4, native);
    ASSERT_TRUE(Compare(series.values[step], series.cursors[step], native)) << "step " << step;
  }
}
TEST(Law90PointCuda, OriginalRotatedNativeHistoryAndDeviceOwnedLateRetry) {
  DeviceSeries series;
  series.material_input = OriginalInput();
  series.curve_count = 28;
  std::copy_n(OriginalCurve().compression_strain, 28, series.x);
  std::copy_n(OriginalCurve().stress_pa, 28, series.y);
  series.step_count = MaximumSteps;
  series.fault_step = 43;
  for (unsigned step = 0; step < series.step_count; ++step) series.input[step] = Path(step, true);
  CheckDevice(series);
}
TEST(Law90PointCuda, NativePlateauZeroEtTensionCapAndCursorRetry) {
  ToyCurve curve;
  curve.y[2] = curve.y[1];
  DeviceSeries series;
  series.material_input = ToyInput();
  series.curve_count = 4;
  std::copy_n(curve.x, 4, series.x);
  std::copy_n(curve.y, 4, series.y);
  const double compression[]{0, .1, .2, .4, .6, .8, .6, .4, .2, .1, 0, -.7};
  series.step_count = sizeof(compression)/sizeof(double);
  series.fault_step = 7;
  for (unsigned step = 0; step < series.step_count; ++step)
    series.input[step] = Stretch(1-compression[step], 1, 1);
  CheckDevice(series);
}
} // namespace law90_point_test
