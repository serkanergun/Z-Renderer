# locate header
find_path(OGLPLUS_INCLUDE_DIRS "oglplus/fwd.hpp" HINTS ${OGLPLUS_SEARCH_DIRS})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OGLPLUS DEFAULT_MSG OGLPLUS_INCLUDE_DIRS)

mark_as_advanced(OGLPLUS_INCLUDE_DIRS OGLPLUS_SEARCH_DIRS)