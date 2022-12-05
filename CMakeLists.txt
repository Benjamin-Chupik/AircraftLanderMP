cmake_minimum_required(VERSION 3.10)
project(AircraftLanderMP)


find_package(ompl)

add_subdirectory(Tempest)


ADD_EXECUTABLE(planePlan main.cpp)

target_link_libraries(planePlan PUBLIC ompl PUBLIC tempest)

target_include_directories(planePlan PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          "${PROJECT_SOURCE_DIR}/Tempest"
                          )