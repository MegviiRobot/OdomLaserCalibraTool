cmake_minimum_required(VERSION 2.6)

# Detection of CSM library using CMake routines
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSL REQUIRED gsl)
pkg_check_modules(CSM REQUIRED csm)

IF(${CSM_FOUND})
	MESSAGE("CSM_LIBRARY_DIRS: ${CSM_LIBRARY_DIRS}")
	MESSAGE("CSM_LIBRARIES: ${CSM_LIBRARIES}")
	MESSAGE("CSM_INCLUDE_DIRS: ${CSM_INCLUDE_DIRS}")

	INCLUDE_DIRECTORIES(${CSM_INCLUDE_DIRS})
	LINK_DIRECTORIES(${CSM_LIBRARY_DIRS})

ELSE(${CSM_FOUND})	
	MESSAGE(SEND_ERROR "CSM not found. Make sure PKG_CONFIG_PATH contains the directory where the file 'csm.pc' can be found.")
ENDIF(${CSM_FOUND})		


# Various flags

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ggdb")
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -O2 ")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2") 

#build dirs
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)

# A test for JSON
ADD_EXECUTABLE(test_json test_json.c)
TARGET_LINK_LIBRARIES(test_json ${CSM_LIBRARIES})


ADD_EXECUTABLE(synchronizer synchronizer.cpp synchronizer_options.cpp  aux_functions.cpp)
TARGET_LINK_LIBRARIES(synchronizer ${CSM_LIBRARIES})

ADD_EXECUTABLE(solver solver.cpp solver_options.cpp solver_utils.cpp aux_functions.cpp)
TARGET_LINK_LIBRARIES(solver ${CSM_LIBRARIES})

ADD_EXECUTABLE(solver2 solver2.cpp solver2_meat.cpp solver_options.cpp solver_utils.cpp aux_functions.cpp calib_tuple.cpp gsl_jacobian.cpp)
TARGET_LINK_LIBRARIES(solver2 ${CSM_LIBRARIES})


ADD_EXECUTABLE(verifier verifier.cpp verifier_options.cpp verifier_utils.cpp aux_functions.cpp analyzer.cpp)
TARGET_LINK_LIBRARIES(verifier ${CSM_LIBRARIES})

ADD_EXECUTABLE(gsl_jacobian_test gsl_jacobian_test.cpp gsl_jacobian.cpp )
TARGET_LINK_LIBRARIES(gsl_jacobian_test ${CSM_LIBRARIES})
