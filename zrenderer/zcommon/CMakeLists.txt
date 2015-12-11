# Copyright (c) ZombieRendering 2015-2016 serkan.ergun@gmail.com

set(ZCOMMON_PUBLIC_HEADERS
  geometry.h
  mathtypes.h
  mesh.h
  types.h
  utils.h
  half.hpp
)

set(ZCOMMON_HEADERS)

set(ZCOMMON_SOURCES 
  mesh.cpp
  utils.cpp)

set(ZCOMMON_LINK_LIBRARIES
  PUBLIC ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} AssImp)

set(ZCOMMON_INCLUDE_NAME zrenderer/zcommon)
set(ZCOMMON_NAMESPACE zcommon)

common_library(zcommon)