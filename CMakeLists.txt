cmake_minimum_required(VERSION 3.10)
project(openloong-dyn-control)

IF (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release) 
ENDIF ()
add_compile_options(-std=c++17)

#系统eigen版本可能老旧，使用自供eigen
set(dirEigen "third_party/eigen3")
set(dirGlfw "third_party/glfw")
set(dirPino "third_party/pinocchio")
set(dirJson "third_party/jsoncpp")
set(dirQuill "third_party/quill")
set(dirQP "third_party/qpOASES")

set(libURDF "third_party/urdfdom")

set(incFmt "third_party/quill/quill/bundled")
set(incBoost "third_party/boost")
set(incMujoco "third_party/mujoco")


set(allDir ${dirEigen} ${dirGlfw} ${dirPino} ${dirJson} ${dirQuill} ${dirQP} )
set(allLib ${allDir} ${libURDF})
set(allInc ${allDir} ${incFmt} ${incBoost} ${incMujoco})

include_directories(${allInc})
include_directories("algorithm")
include_directories("common")
include_directories("math")
include_directories("sim_interface")

message(${CMAKE_SYSTEM_PROCESSOR})
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
	list(APPEND allLib "third_party/mujoco/lin_arm64")
else()
	list(APPEND allLib "third_party/mujoco/lin_x64")
endif()
link_directories(${allLib})

file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp algorithm/*.cpp common/*.cpp math/*.cpp sim_interface/*.cpp)
file(GLOB HEADER_FILES *.h algorithm/*.h common/*.h math/*.h sim_interface/*.h)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES} ${HEADER_FILES})

#链接库
set(corLibs pinocchio urdfdom_model tinyxml console_bridge jsoncpp quill qpOASES)
set(simLibs glfw3)#mujoco区分x86/arm的方式不一样，在上面按link目录区分

#根据架构，处理连接库
set(sysCoreLibs)
set(sysSimLibs)
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
	message("linux arm64架构")
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_arm64)
	endforeach()
	foreach(lib ${simLibs})
		list(APPEND sysSimLibs ${lib}_lin_arm64)
	endforeach()
	list(APPEND sysSimLibs dl)#arm 需要 -ldl
else()
	message("linux x64架构")
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_x64)
	endforeach()
	foreach(lib ${simLibs})
		list(APPEND sysSimLibs ${lib}_lin_x64)
	endforeach()
endif()
message(${sysCoreLibs})

#生成控制核心库
add_library(core ${SOURCES})
target_link_libraries(core ${sysCoreLibs} pthread)

#生成仿真可执行文件
add_executable(walk_wbc_joystick demo/walk_wbc_joystick.cpp)
target_link_libraries(walk_wbc_joystick core mujoco ${sysSimLibs} dl)

add_executable(walk_mpc_wbc_joystick demo/walk_mpc_wbc_joystick.cpp)
target_link_libraries(walk_mpc_wbc_joystick core mujoco ${sysSimLibs} dl)
