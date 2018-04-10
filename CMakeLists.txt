cmake_minimum_required(VERSION 2.8)
project(scanline)

find_package(SDL REQUIRED)

file(GLOB CSOURCE source/*.cpp)
file(GLOB HSOURCE source/*.h)

include_directories(${SDL_INCLUDE_DIR})

add_executable(scanline ${CSOURCE} ${HSOURCE})
target_link_libraries(scanline ${SDL_LIBRARY})
