# Included after the standalone owning targets are defined.
target_sources(represented_interval_crossing_gpu_cuda PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/CompoundCudaTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../represented_interval_crossing/BatchAssertions.h"
  "${CMAKE_CURRENT_LIST_DIR}/../represented_interval_crossing/BatchFixture.h")
add_test(NAME represented_interval_crossing_gpu_compound_source
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/verify_compound_sources.py")
set_tests_properties(represented_interval_crossing_gpu_compound_source PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30 LABELS "unit;source;cuda-native-compound")
add_test(NAME represented_interval_crossing_gpu_scene_authority
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/check_scene_authority.py"
          --compiler "${CMAKE_CXX_COMPILER}")
set_tests_properties(represented_interval_crossing_gpu_scene_authority PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 45 LABELS "unit;syntax;cuda-native-authority")
