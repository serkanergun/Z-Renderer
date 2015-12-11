# Copyright (c) ZombieRendering 2015-2016 serkan.ergun@gmail.com
include(StringifyShaders)

set(ZRBVH_SHADERS shaders/vertRaster.glsl
                  shaders/fragRaster.glsl )

stringify_shaders(${ZRBVH_SHADERS})

set(ZRBVH_PUBLIC_HEADERS)

set(ZRBVH_HEADERS 
  rbvhtree.h 
  types.h)

set(ZRBVH_SOURCES 
  ${SHADER_SOURCES}
  rbvhtree.cpp)

set(ZRBVH_LINK_LIBRARIES 
  PUBLIC ${Boost_SYSTEM_LIBRARY} FreeImage ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}
  PRIVATE zcommon)

set(ZRBVH_INCLUDE_NAME zrenderer/geometry/zrbvh)
set(ZRBVH_NAMESPACE ZRBVH)

common_library(zrbvh)