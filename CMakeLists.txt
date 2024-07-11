cmake_minimum_required(VERSION 3.20)

project(mx_algo)

# mx_algo sources
SET(mx_algo ${CMAKE_CURRENT_SOURCE_DIR})


#####################
# mx_algo library
#####################

add_library(mx_algo STATIC
    source/mx_algo.cpp)

# include directories
target_include_directories(mx_algo PUBLIC ${rpmath}/include)
