find_path(GLFW3_INCLUDE_DIRS "GLFW/glfw3.h" HINTS "${GLFW3_ROOT}/include")
find_library(GLFW3_LIBRARIES NAMES glfw3 PATHS "${GLFW3_ROOT}/lib-vc2015")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLFW3
                                  REQUIRED_VARS GLFW3_INCLUDE_DIRS GLFW3_LIBRARIES)
                                  
mark_as_advanced(GLFW3_INCLUDE_DIRS GLFW3_LIBRARIES)

