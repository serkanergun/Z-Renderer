# Copyright (c) ZombieRendering 2015-2016 ahmetbilgili@gmail.com
#                                         serkan.ergun@gmail.com

set(RBVH_PUBLIC_HEADERS rbvhnode.h)
set(RBVH_HEADERS)
set(RBVH_SOURCES rbvhnode.cpp)
set(RBVH_LINK_LIBRARIES PRIVATE ${Boost_PROGRAM_OPTIONS_LIBRARY})

common_library(RBVH)