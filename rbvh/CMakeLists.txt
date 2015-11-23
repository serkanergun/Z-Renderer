# Copyright (c) ZombieRendering 2015-2016 serkan.ergun@gmail.com

set(RBVH_PUBLIC_HEADERS)
set(RBVH_HEADERS rbvhnode.h)
set(RBVH_SOURCES rbvhnode.cpp)
set(RBVH_LINK_LIBRARIES PRIVATE ${Boost_PROGRAM_OPTIONS_LIBRARY} FreeImage)

common_library(RBVH)