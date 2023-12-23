cmake_minimum_required(VERSION 3.0)
project(ComputedTorqueControl)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)
include(modules/CMakeFunctions.cmake)
set(BUILD_SHARED_LIBS FALSE)
set(CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/modules)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()


find_package(SimRobot REQUIRED)
find_package(pinocchio REQUIRED)
find_package(LR REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Bullet REQUIRED)
find_package(urdfdom REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)

add_executable(${PROJECT_NAME} main.cpp ${RobotSimulator_SRCS})
target_link_libraries(${PROJECT_NAME} 
 		Eigen3::Eigen 
		SimRobot::SimRobot 
		LR::LR 
		Threads::Threads
		spdlog::spdlog
		${BULLET_LIBRARIES} 
		${urdfdom_LIBRARIES}
		)
target_include_directories(${PROJECT_NAME} PUBLIC 
		${RobotSimulator_INCLUDE_DIR}	
)
target_compile_definitions(${PROJECT_NAME} PUBLIC -DBT_USE_DOUBLE_PRECISION )

