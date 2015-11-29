# Copyright (c) ZombieRendering 2015-2016 serkan.ergun@gmail.com

add_definitions(-DBOOST_PROGRAM_OPTIONS_DYN_LINK) # Fix for windows and shared boost.
set(RBVHCONVERTER_HEADERS)
set(RBVHCONVERTER_SOURCES main.cpp)
set(RBVHCONVERTER_LINK_LIBRARIES RBVH ${Boost_PROGRAM_OPTIONS_LIBRARY} Common)

common_application(rbvhconverter)