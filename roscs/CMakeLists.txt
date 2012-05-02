cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

rosbuild_include(roscs cncs)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_library(roscs src/cpp/roscs.cpp)

#FILE(GLOB SRCFILES_CSLIB src/*.cs)
#CSHARP_ADD_LIBRARY(RosSharp ${SRCFILES_CSLIB})


#FILE(GLOB SRCFILES_CODEGEN )
#CSHARP_ADD_LIBRARY(GenerateCode ${SRCFILES_CS})
CSHARP_ADD_EXE(GenerateCode
	src/codegen/Constant.cs
	src/codegen/Message.cs
	src/codegen/MessageField.cs
	src/codegen/GenerateCXCode.cs
)
CSHARP_ADD_EXE(QueryDependencies src/codegen/QueryDependencies.cs)

CSHARP_ADD_EXE(GenerateServices
	src/codegen/Service.cs
	src/codegen/Constant.cs
	src/codegen/Message.cs
	src/codegen/MessageField.cs
	src/codegen/GenerateCXServices.cs
)

#CSHARP_ADD_TARGET_DEPENDENCY(TestMe RosSharp)
#CSHARP_ADD_EXE(TestMe src/Test/Test.cs)
