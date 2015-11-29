# Copyright (c) ZombieRendering 2015-2016 serkan.ergun@gmail.com

include_directories(${Eigen3_INCLUDE_DIR})

set(COMMON_PUBLIC_HEADERS
  mathtypes.h
  mesh.h
  types.h
)

set(COMMON_HEADERS)

set(COMMON_SOURCES 
  mesh.cpp)

set(COMMON_LINK_LIBRARIES
  PUBLIC ${Boost_SYSTEM_LIBRARY})

set(COMMON_INCLUDE_NAME zrenderer/common)
set(COMMON_NAMESPACE zcommon)

common_library(Common)