# Copyright (c) ZombieRendering 2015-2016 serkan.ergun@gmail.com

set(RBVH_PUBLIC_HEADERS)

set(RBVH_HEADERS 
  rbvhnode.h 
  types.h)

set(RBVH_SOURCES 
  rbvhnode.cpp)

set(RBVH_LINK_LIBRARIES 
  PUBLIC ${Boost_SYSTEM_LIBRARY} FreeImage AssImp
  PRIVATE Common)

set(RBVH_INCLUDE_NAME zrenderer/geometry/rbvh)
set(RBVH_NAMESPACE ZRBVH)

common_library(RBVH)