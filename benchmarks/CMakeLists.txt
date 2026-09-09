if(NOT TARGET robo_dyna_elastic_coupon_case)
  message(FATAL_ERROR "The coupon profile requires ROBO_DYNA_ENABLE_ELASTIC_COUPON")
endif()
find_package(CUDAToolkit REQUIRED)
add_executable(robo-dyna-coupon-profile
  CouponProfile.cpp CouponProfileReport.cpp coupon_profile_main.cpp)
target_compile_features(robo-dyna-coupon-profile PRIVATE cxx_std_17)
target_compile_options(robo-dyna-coupon-profile PRIVATE -fno-fast-math -ffp-contract=off)
target_link_libraries(robo-dyna-coupon-profile PRIVATE
  robo_dyna_elastic_coupon_case robo_dyna_artifact_io CUDA::cudart)
