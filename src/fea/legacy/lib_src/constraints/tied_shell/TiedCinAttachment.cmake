include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/TiedPatch.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/TiedPostKinChk.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../assembly/NodalNodeDomain.cmake")
add_library(tl_tied_cin_attachment STATIC
  "${CMAKE_CURRENT_LIST_DIR}/TiedCinAttachmentModel.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/TiedCinAttachmentMap.cpp")
target_link_libraries(tl_tied_cin_attachment PUBLIC tl_tied_shell_patch tl_tied_post_kinchk tl_nodal_node_domain)
target_compile_features(tl_tied_cin_attachment PUBLIC cxx_std_17)
target_compile_options(tl_tied_cin_attachment PRIVATE -fno-fast-math -ffp-contract=off)
