find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
add_library(robo_dyna_elastic_coupon_reference STATIC ElasticCouponModel.cpp ElasticCouponModal.cpp)
target_link_libraries(robo_dyna_elastic_coupon_reference PUBLIC robo_dyna_reissner_setup Eigen3::Eigen)
target_compile_options(robo_dyna_elastic_coupon_reference PRIVATE -fno-fast-math -ffp-contract=off)

# This separate CPU audit configures the CUDA case; it never advances dynamics.
add_executable(robo_dyna_elastic_coupon_reference_check elastic_coupon_reference_check.cpp)
target_link_libraries(robo_dyna_elastic_coupon_reference_check PRIVATE
  robo_dyna_elastic_coupon_reference GTest::gtest_main)
add_test(NAME elastic_coupon_reference COMMAND robo_dyna_elastic_coupon_reference_check)
set_tests_properties(elastic_coupon_reference PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1
  ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
