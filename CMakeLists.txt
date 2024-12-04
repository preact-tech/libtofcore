
###############################################################################
# CMakeList.txt file for the libtofcore library
cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(libToF VERSION 1.2.0 LANGUAGES CXX)

add_subdirectory(tofcore)
add_subdirectory(tofcrust)
