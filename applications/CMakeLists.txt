add_library(altro_test_utils
  ../test/test_utils.cpp
  finitediff.hpp
  finitediff.cpp
)


target_link_libraries(altro_test_utils
  PUBLIC
  gtest_main
  altro
  fmt::fmt
  Eigen3::Eigen
  altro_impl

  PRIVATE
  nlohmann_json::nlohmann_json
)

# FetchContent_Declare(
#     finite-diff
#     GIT_REPOSITORY https://github.com/zfergus/finite-diff.git
#     GIT_TAG 60f9fee0928922f1e159d4659aa98a466c24fae4
#     GIT_SHALLOW TRUE
# )
# FetchContent_MakeAvailable(finite-diff)
# target_include_directories(altro_test_utils
#   INTERFACE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
# )
target_compile_definitions(altro_test_utils
  PUBLIC
  ALTRO_TEST_DIR="/home/yifei/Documents/optimal_ctrl/altro/test"
)

add_executable(main
  main.cpp
)
target_link_libraries(main
  PRIVATE
  altro
  altro_test_utils
  fmt::fmt
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
)

# add_executable(main_bicycle
#   main_bicycle.cpp
# )
# target_link_libraries(main_bicycle
#   PRIVATE
#   altro
#   altro::test_utils
#   fmt::fmt
#   Eigen3::Eigen
#   nlohmann_json::nlohmann_json
# )


