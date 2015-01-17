# Evolve gaits for a robot
add_executable( evolve-gaits evolve_gaits.cpp ModularRobotEvalOp.cpp )
set_target_properties(evolve-gaits PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS}")
set_target_properties(evolve-gaits PROPERTIES LINK_FLAGS "${ECF_LINK_FLAGS}")
target_link_libraries(evolve-gaits ${ECF_LINK_FLAGS})

add_executable( dummy-answer dummy_answer.cpp)
set_target_properties(dummy-answer PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS}")
