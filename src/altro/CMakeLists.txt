add_subdirectory(solver)

add_library(altro

  # altro.hpp
  # altro_solver.hpp
  altro_solver.cpp


  utils/quaternion_utils.cpp
)
target_include_directories(altro
  PUBLIC ../../include 
  PRIVATE utils)
target_link_libraries(altro
  PRIVATE
  fmt::fmt
  altro_impl
)

# set_target_properties(altro PROPERTIES EXPORT_NAME altro)
# add_library(altro ALIAS altro)


# target_sources(altro
#   PUBLIC FILE_SET HEADERS
#   BASE_DIRS ..
#   FILES
#   altro.hpp
#   altro_solver.hpp
#   solver/solver_options.hpp
#   solver/exceptions.hpp
#   solver/typedefs.hpp
# )