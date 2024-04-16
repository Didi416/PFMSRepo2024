#######################################################################
# Now we enable gtests - this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

enable_testing()
###################################

# Just in case someone had the include files in seperate directory
include_directories(../include)
include_directories(..)

# This uses the complete student's library
aux_source_directory(.. SRC_LIST)
list(REMOVE_ITEM SRC_LIST "../main.cpp" "../mission.cpp")
message ( STATUS "Compiling student_lib with following files  ${SRC_LIST}" )
add_library(student_lib ${SRC_LIST})
target_link_libraries(student_lib pfmsconnector audi)

add_dependencies(student_lib pipes::pfmsconnector)
link_directories(/usr/local/lib)

# add_library(mission "../mission.cpp")
# target_link_libraries(mission pfmsconnector)

# ACKERMAN TEST
add_executable(testAudiReachGoals test_audi_reach_goals.cpp)
target_link_libraries(testAudiReachGoals ${GTEST_LIBRARIES} pfmshog student_lib pthread)

# QUADCOPTER TEST
add_executable(testQuadcopterReachGoals test_quad_reach_goals.cpp)
target_link_libraries(testQuadcopterReachGoals ${GTEST_LIBRARIES} student_lib pfmshog pthread)

# MISSION TEST
add_executable(testMissionReachGoals test_mission_reach_goals.cpp)
target_link_libraries(testMissionReachGoals ${GTEST_LIBRARIES} student_lib mission pfmshog pthread)

# TSP Ackerman
add_executable(testTspReachGoalsAck tsp_reach_goals_ack.cpp)
target_link_libraries(testTspReachGoalsAck ${GTEST_LIBRARIES} student_lib mission pfmshog pthread)

# TSP Quadcopter
add_executable(testTspReachGoalsQuad tsp_reach_goals_quad.cpp)
target_link_libraries(testTspReachGoalsQuad ${GTEST_LIBRARIES} student_lib mission pfmshog pthread)

