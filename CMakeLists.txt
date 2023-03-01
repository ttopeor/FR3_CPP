cmake_minimum_required(VERSION 3.10)

project(FR3_CPP
    VERSION 1.0
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB SOURCE_FILE src/*.cpp )


include_directories(include)
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED )
find_package(matplotlib_cpp REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)
find_package(GSL REQUIRED)

add_executable(fr3 ${SOURCE_FILE})


target_link_libraries(fr3 PRIVATE Franka::Franka Eigen3::Eigen matplotlib_cpp::matplotlib_cpp GSL::gsl) 
