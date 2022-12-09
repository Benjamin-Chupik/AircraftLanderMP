add_library(tempest tempest.cpp)

# Works without all the stuff bellow
target_include_directories(tempest PUBLIC
${OMPL_INCLUDE_DIRS} 
# ${PROJECT_SOURCE_DIR}/stl_rrt_planner
# ${PROJECT_SOURCE_DIR}/PrSTL_Monitor
# ${PROJECT_SOURCE_DIR}/Spaces
)
target_link_libraries(tempest
    ${OMPL_LIBRARIES}
)