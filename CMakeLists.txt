cmake_minimum_required(VERSION 3.8)
project(play)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # #使用release o2
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)
add_executable(play
    main.cpp
    src/benchmark/benchmark.cpp
    ./src/YAstar/yastar.cpp
    ./src/YAstar/kinodynamicAstar.cpp
    ./src/MinimumSnapOsqp/minimumSnap.cpp
    ./src/MinimumSnapOsqp/sfcSquare.cpp
)
target_compile_definitions(play PRIVATE BENCHMARK_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

target_link_libraries(play
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
    OsqpEigen::OsqpEigen
)

include(CTest)
if(BUILD_TESTING)
    add_executable(minimum_snap_regression
        test/minimum_snap_regression.cpp
        src/MinimumSnapOsqp/minimumSnap.cpp
        src/MinimumSnapOsqp/sfcSquare.cpp
    )
    target_include_directories(minimum_snap_regression PRIVATE ${PROJECT_SOURCE_DIR})
    target_link_libraries(minimum_snap_regression PRIVATE ${OpenCV_LIBS} OsqpEigen::OsqpEigen)
    add_test(NAME minimum_snap_regression COMMAND minimum_snap_regression)
    find_package(PythonInterp 3 QUIET)
    if(PYTHONINTERP_FOUND)
        add_test(NAME benchmark_integration COMMAND ${PYTHON_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/test/test_benchmark.py --executable $<TARGET_FILE:play>)
        set_tests_properties(benchmark_integration PROPERTIES ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
    endif()
endif()
