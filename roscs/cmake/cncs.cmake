
#SET(CMAKE_CSHARP_COMPILER gmcs)

FIND_PROGRAM(CMAKE_CSHARP_COMPILER NAMES dmcs gmcs PATHS ENV PATH)
MESSAGE("CSharp Compiler is ${CMAKE_CSHARP_COMPILER}")


SET(DOC_OUTPUT_PATH "")

macro(CSHARP_ADD_BUILD_FLAGS target)
	foreach(arg ${ARGN})
		set(CSHARP_BUILD_FLAGS_${target} "${CSHARP_BUILD_FLAGS_${target}} ${arg}")
	endforeach(arg)
endmacro(CSHARP_ADD_BUILD_FLAGS target)

macro(CSHARP_ADD_TARGET_DEPENDENCY target)
	#set(csharp_ref_${target})
	#set(csharp_dep_${target})
	set(tmp)
	foreach(_ref ${ARGN})
		set(tmp "${tmp} -r:${_ref}.dll")
		if(csharp_is_building_lib_${_ref})
			#set(csharp_dep_${target} ${csharp_dep_${target}}  CSharp_${_ref}_dll)
			set(csharp_dep_${target} ${csharp_dep_${target}}  ${LIBRARY_OUTPUT_PATH}/${_ref}.dll)
		endif(csharp_is_building_lib_${_ref})
	endforeach(_ref)
	set(csharp_ref_${target} "${csharp_ref_${target}} ${tmp}")
endmacro(CSHARP_ADD_TARGET_DEPENDENCY target)


macro(CSHARP_ADD_PKG_DEPENDENCY target)
	set(tmp)
	foreach(_ref ${ARGN})
		set(tmp "${tmp} -pkg:${_ref}")
	endforeach(_ref)
	set(csharp_ref_${target} "${csharp_ref_${target}} ${tmp}")
endmacro(CSHARP_ADD_PKG_DEPENDENCY target)


macro(CSHARP_ADD_RESOURCE target)
	set(tmp)
	foreach(_ref ${ARGN})
		set(tmp "${tmp} -resource:${_ref}")
	endforeach(_ref)
	set(csharp_ref_${target} "${csharp_ref_${target}} ${tmp}")
endmacro(CSHARP_ADD_RESOURCE target)


MACRO(CSHARP_ADD_LIBRARY name)
  SET(csharp_cs_sources)
  SET(csharp_is_building_lib_${name} 1)
  SEPARATE_ARGUMENTS(${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES)
  CSHARP_ADD_TARGET_DEPENDENCY(${name} ${${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES})
  FOREACH(it ${ARGN})
    IF(EXISTS ${it})
      SET(csharp_cs_sources "${csharp_cs_sources} ${it}")
    ELSE(EXISTS ${it})
      IF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${it})
        SET(csharp_cs_sources "${csharp_cs_sources} ${CMAKE_CURRENT_SOURCE_DIR}/${it}")
      ELSE(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${it})
        MESSAGE("Could not find: ${it}")
        SET(csharp_cs_sources "${csharp_cs_sources} ${it}")
      ENDIF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${it})
    ENDIF(EXISTS ${it})
  ENDFOREACH(it)
  #cs_add_msg_dependencies(${name} "${${PROJECT_NAME}_MESSAGE_DEPENDENCIES}")
  #Get Paths:
	SET(cs_deps)
	SET(cs_curdeps)
	rosbuild_invoke_rospack(${PROJECT_NAME} "cs" deps depends1)
	string(REGEX REPLACE "\n" ";" cs_deps "${cs_deps}")
	FOREACH(it ${cs_deps})
		rosbuild_invoke_rospack(${it} "cs" lpath export --lang=cs --attrib=lpath)
		 IF(NOT cs_lpath STREQUAL "")		
		 	STRING(REGEX REPLACE ";" "," cs_lpath "${cs_lpath}")		
		 	SET(cs_curdeps "${cs_curdeps},${cs_lpath}")
		ENDIF(NOT cs_lpath STREQUAL "")
	ENDFOREACH(it)
	#STRING(REGEX REPLACE " " "," cs_curdeps "${cs_curdeps}")
	IF(NOT cs_curdeps STREQUAL "")
		SET(cs_curdeps ",${cs_curdeps}")
	ENDIF(NOT cs_curdeps STREQUAL "")
  #SET(SHARP #)
  SEPARATE_ARGUMENTS(csharp_cs_sources)
  IF(DOC_OUTPUT_PATH STREQUAL "")
	SET(DOC_GEN "")
  ELSE(DOC_OUTPUT_PATH STREQUAL "")
	file(MAKE_DIRECTORY ${DOC_OUTPUT_PATH}/${name})
	SET(DOC_GEN "-doc:${DOC_OUTPUT_PATH}/${name}/${name}.xml")
  ENDIF(DOC_OUTPUT_PATH STREQUAL "")

  SET(cs_args "${csharp_ref_${name}} -unsafe -debug+ -optimize+ ${DOC_GEN} ${CSHARP_BUILD_FLAGS_${name}} -target:library -lib:${LIBRARY_OUTPUT_PATH}${cs_curdeps} -out:${LIBRARY_OUTPUT_PATH}/${name}.dll")
  SEPARATE_ARGUMENTS(cs_args)
  ADD_CUSTOM_COMMAND(
    OUTPUT ${LIBRARY_OUTPUT_PATH}/${name}.dll ${LIBRARY_OUTPUT_PATH}/${name}.dll.mdb
    COMMAND ${CMAKE_CSHARP_COMPILER}    
    ARGS ${cs_args} ${csharp_cs_sources}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${csharp_cs_sources} ${csharp_dep_${name}}
    COMMENT "Creating Csharp library ${name}.dll"
  )
#AOT STUFF:
#  ADD_CUSTOM_COMMAND(
#    OUTPUT ${LIBRARY_OUTPUT_PATH}/${name}.dll.so
#    COMMAND mono   
#    ARGS --aot -O=all ${LIBRARY_OUTPUT_PATH}/${name}.dll
#    WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH}
#    DEPENDS ${LIBRARY_OUTPUT_PATH}/${name}.dll
#    COMMENT "Creating AOT form of library ${name}.dll"
#  )
#END OF AOT STUFF
  ADD_CUSTOM_TARGET(CSharp_${name}_dll ALL
    DEPENDS ${LIBRARY_OUTPUT_PATH}/${name}.dll #${LIBRARY_OUTPUT_PATH}/${name}.dll.so
  )
  file(MAKE_DIRECTORY ${LIBRARY_OUTPUT_PATH})
ENDMACRO(CSHARP_ADD_LIBRARY)

MACRO(CSHARP_ADD_EXE name)
 # SET(tmp "csharp_ref_${name}")
  SET(${name}_is_exe 1)
  SET(csharp_cs_sources)
  SEPARATE_ARGUMENTS(${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES)
  CSHARP_ADD_TARGET_DEPENDENCY(${name} ${${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES})
  FOREACH(it ${ARGN})
    IF(EXISTS ${it})
      SET(csharp_cs_sources "${csharp_cs_sources} ${it}")
    ELSE(EXISTS ${it})
      IF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${it})
        SET(csharp_cs_sources "${csharp_cs_sources} ${CMAKE_CURRENT_SOURCE_DIR}/${it}")
      ELSE(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${it})
        MESSAGE("Could not find: ${it}")
        SET(csharp_cs_sources "${csharp_cs_sources} ${it}")
      ENDIF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${it})
    ENDIF(EXISTS ${it})
  ENDFOREACH(it)
  #cs_add_msg_dependencies(${name} "${${PROJECT_NAME}_MESSAGE_DEPENDENCIES}")

  #Get Paths:
	SET(cs_deps)
	SET(cs_curdeps)
	rosbuild_invoke_rospack(${PROJECT_NAME} "cs" deps depends1)
	string(REGEX REPLACE "\n" ";" cs_deps "${cs_deps}")
	FOREACH(it ${cs_deps})
		rosbuild_invoke_rospack(${it} "cs" lpath export --lang=cs --attrib=lpath)
#MESSAGE("STR is ${cs_lpath}")
		 IF(NOT cs_lpath STREQUAL "")
			STRING(REGEX REPLACE ";" "," cs_lpath "${cs_lpath}")		
		 	SET(cs_curdeps "${cs_curdeps},${cs_lpath}")
		ENDIF(NOT cs_lpath STREQUAL "")
#MESSAGE("STR be ${cs_lpath}")
	ENDFOREACH(it)
	#STRING(REGEX REPLACE " " "," cs_curdeps "${cs_curdeps}")
	IF(NOT cs_curdeps STREQUAL "")
		SET(cs_curdeps ",${cs_curdeps}")
	ENDIF(NOT cs_curdeps STREQUAL "")

  #SET(SHARP #)

  SEPARATE_ARGUMENTS(csharp_cs_sources)
  IF(DOC_OUTPUT_PATH STREQUAL "")
	SET(DOC_GEN "")
  ELSE(DOC_OUTPUT_PATH STREQUAL "")
	file(MAKE_DIRECTORY ${DOC_OUTPUT_PATH}/${name})
	SET(DOC_GEN "-doc:${DOC_OUTPUT_PATH}/${name}/${name}.xml")
  ENDIF(DOC_OUTPUT_PATH STREQUAL "")
  SET(cs_args "${csharp_ref_${name}} -unsafe -debug+ -optimize+ -target:exe ${DOC_GEN} ${CSHARP_BUILD_FLAGS_${name}} -lib:${LIBRARY_OUTPUT_PATH}${cs_curdeps} -out:${EXECUTABLE_OUTPUT_PATH}/${name}.exe")
  SEPARATE_ARGUMENTS(cs_args)
  ADD_CUSTOM_COMMAND(
    OUTPUT ${EXECUTABLE_OUTPUT_PATH}/${name}.exe
    COMMAND ${CMAKE_CSHARP_COMPILER}
    #ARGS "-debug+" "-optimize+" "-target:library" ${tmp1} "-out:${LIBRARY_OUTPUT_PATH}/${name}.dll"  ${csharp_cs_sources}
    ARGS ${cs_args} ${csharp_cs_sources}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${csharp_cs_sources} ${csharp_dep_${name}}
    COMMENT "Creating Csharp executable ${name}.exe"
  )
#AOT STUFF:
#  ADD_CUSTOM_COMMAND(
#    OUTPUT ${EXECUTABLE_OUTPUT_PATH}/${name}.exe.so
#    COMMAND mono   
#    ARGS --aot -O=all ${EXECUTABLE_OUTPUT_PATH}/${name}.exe
#    WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
#    DEPENDS ${EXECUTABLE_OUTPUT_PATH}/${name}.exe
#    COMMENT "Creating AOT form of executable ${name}.exe"
#  )
#END OF AOT STUFF
  ADD_CUSTOM_TARGET(CSharp_${name}_exe ALL
    DEPENDS ${EXECUTABLE_OUTPUT_PATH}/${name}.exe #${EXECUTABLE_OUTPUT_PATH}/${name}.exe.so
  )
  file(MAKE_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

ENDMACRO(CSHARP_ADD_EXE)

# genmsg processes srv/*.srv files into cs
macro(rosbuild_gen_cs_srv)
	SET(TMP)
	FOREACH(it ${ARGN})
	SET(TMP "${TMP} ${it}")
	ENDFOREACH(it)
	rosbuild_invoke_rospack(roscs "csgenerate" bpath export --lang=cs --attrib=bpath)

	execute_process(
    		COMMAND ${csgenerate_bpath}/QueryDependencies.exe -s ${PROJECT_NAME}
		OUTPUT_VARIABLE _srvFiles
    		ERROR_VARIABLE _err
		RESULT_VARIABLE _failed
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)

	SEPARATE_ARGUMENTS(_srvfiles)
	rosbuild_invoke_rospack(${PROJECT_NAME} "cssrvbuild" deps depends1)
	string(REGEX REPLACE "\n" ";" cssrvbuild_deps "${cssrvbuild_deps}")
	SET(codeGenArgs)
	FOREACH(it ${cssrvbuild_deps})
		SET(codeGenArgs "${codeGenArgs} ${it}/*")
	ENDFOREACH(it)
	SEPARATE_ARGUMENTS(codeGenArgs)

	ADD_CUSTOM_COMMAND(
	    OUTPUT ${PROJECT_SOURCE_DIR}/srv_cs_gen/cpp/roscs.cpp ${PROJECT_SOURCE_DIR}/srv_cs_gen/csharp/Service.cs ${PROJECT_SOURCE_DIR}/srv_cs_gen/csharp/ServiceClient.cs ${PROJECT_SOURCE_DIR}/srv_cs_gen/csharp/AbstractService.cs
	    COMMAND ${csgenerate_bpath}/GenerateServices.exe
	    ARGS  ${PROJECT_NAME} ${codeGenArgs}
	    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	    DEPENDS  ${_srvFiles} ${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.Communication.dll
	    COMMENT "Creating Csharp Wrapper Code"
	)

	rosbuild_add_library(${PROJECT_NAME}.services ${PROJECT_SOURCE_DIR}/srv_cs_gen/cpp/roscs.cpp)

	ADD_CUSTOM_COMMAND(
		OUTPUT ${LIBRARY_OUTPUT_PATH}/RosCS.Services.dll ${LIBRARY_OUTPUT_PATH}/RosCS.Services.dll.mdb
		COMMAND ${CMAKE_CSHARP_COMPILER}
		ARGS -unsafe -debug+ -optimize+ -target:library -r:Mono.Posix.dll -r:${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll -out:${LIBRARY_OUTPUT_PATH}/RosCS.Services.dll ${PROJECT_SOURCE_DIR}/srv_cs_gen/csharp/AbstractService.cs
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}		    
		DEPENDS ${PROJECT_SOURCE_DIR}/srv_cs_gen/csharp/AbstractService.cs ${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll
		COMMENT "Creating Csharp RosCS.Services.dll"
	)
	SET("${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES" "${${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES} RosCS.Services")

	#build service lib for each dependency
	SET(TMPSRVLINK "")
	SET(TMPSRVDLLDEP "")
	FOREACH(it ${cssrvbuild_deps})
		FILE(GLOB "${it}_srvfiles" ${${it}_PACKAGE_PATH}/srv/*.srv)
		MESSAGE("CHECKING BUILD ${it}.Services")
# MESSAGE("var : ${${it}_srvfiles}")
		IF(NOT ${it}_srvfiles STREQUAL "")

			#build dependency to messages
			rosbuild_invoke_rospack(${it} "csmsgdep" deps depends1)
			string(REGEX REPLACE "\n" ";" csmsgdep_deps "${csmsgdep_deps}")
			SET(TMPLINK "")
			FOREACH(dd ${csmsgdep_deps})
				SET(XFILEEXISTS)			
				FILE(GLOB XFILEEXISTS ${${dd}_PACKAGE_PATH}/msg/*.msg)			
				IF(NOT XFILEEXISTS STREQUAL "")
				SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/${dd}.Messages.dll")
				ENDIF(NOT XFILEEXISTS STREQUAL "")
			ENDFOREACH(dd)

			#hack because rosdep is broken in fuerte version (see https://code.ros.org/trac/ros/ticket/3956)
			#link std_msgs
			IF(NOT ${it} STREQUAL "std_msgs")
				SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/std_msgs.Messages.dll")
				SET(TMPDLLDEP "${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/std_msgs.Messages.dll")
			ENDIF(NOT ${it} STREQUAL "std_msgs")
			#link geometry_msgs
			IF(${it} STREQUAL "sensor_msgs")
				SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
				SET(TMPDLLDEP "${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
			ENDIF(${it} STREQUAL "sensor_msgs")
			#link nav_msgs
			IF(${it} STREQUAL "nav_msgs")
				SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
				SET(TMPDLLDEP "${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
			ENDIF(${it} STREQUAL "nav_msgs")
			#link visualization_msgs
			IF(${it} STREQUAL "visualization_msgs")
				SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
				SET(TMPDLLDEP "${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
			ENDIF(${it} STREQUAL "visualization_msgs")

			MESSAGE("SHOULD BUILD ${it}.Services")
			SEPARATE_ARGUMENTS(TMPLINK)
			ADD_CUSTOM_COMMAND(
				OUTPUT ${LIBRARY_OUTPUT_PATH}/${it}.Services.dll ${LIBRARY_OUTPUT_PATH}/${it}.Services.dll.mdb
				COMMAND ${CMAKE_CSHARP_COMPILER}
				ARGS -unsafe -debug+ -optimize+ -target:library -r:Mono.Posix.dll -r:${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll -r:${LIBRARY_OUTPUT_PATH}/RosCS.Services.dll -r:${LIBRARY_OUTPUT_PATH}/${it}.Messages.dll ${TMPLINK} -out:${LIBRARY_OUTPUT_PATH}/${it}.Services.dll ${PROJECT_SOURCE_DIR}/srv_cs_gen/csharp/${it}/*.cs
 				WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
				DEPENDS ${LIBRARY_OUTPUT_PATH}/RosCS.Services.dll ${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll ${LIBRARY_OUTPUT_PATH}/${it}.Messages.dll
				COMMENT "Creating Csharp ${it}.Services.dll"
			)
			SET(TMPSRVLINK "${TMPSRVLINK} -r:${LIBRARY_OUTPUT_PATH}/${it}.Services.dll")
			SET(TMPSRVDLLDEP "${TMPSRVDLLDEP} ${LIBRARY_OUTPUT_PATH}/${it}.Services.dll")
			SET(csharp_is_building_lib_${it}.Services 1)
			SET("${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES" "${${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES} ${it}.Services")
		ENDIF(NOT ${it}_srvfiles STREQUAL "")
	ENDFOREACH(it)	

# MESSAGE("var : ${${PROJECT_NAME}_srvfiles}")
	FILE(GLOB "${PROJECT_NAME}_srvfiles" ${${it}_PACKAGE_PATH}/srv/*.srv)
	IF(NOT ${PROJECT_NAME}_srvfiles STREQUAL "")
#build dependency to messages
		rosbuild_invoke_rospack(${PROJECT_NAME} "csmsgdep" deps depends1)
		string(REGEX REPLACE "\n" ";" csmsgdep_deps "${csmsgdep_deps}")
		SET(TMPLINK "")
		SET(csmsgdep_deps "${csmsgdep_deps} ${PROJECT_NAME}")
		FOREACH(dd ${csmsgdep_deps})
			SET(XFILEEXISTS)			
			FILE(GLOB XFILEEXISTS ${${dd}_PACKAGE_PATH}/msg/*.msg)			
			IF(NOT XFILEEXISTS STREQUAL "")
			SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/${dd}.Messages.dll")
			ENDIF(NOT XFILEEXISTS STREQUAL "")
		ENDFOREACH(dd)

		#hack
		#link std_msgs
		IF(NOT ${PROJECT_NAME}_srvfiles STREQUAL "std_msgs")
			SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/std_msgs.Messages.dll")
			SET(TMPDLLDEP "${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/std_msgs.Messages.dll")
		ENDIF(NOT ${PROJECT_NAME}_srvfiles STREQUAL "std_msgs")
		#link geometry_msgs
		IF(${PROJECT_NAME}_srvfiles STREQUAL "sensor_msgs")
			SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
			SET(TMPDLLDEP "${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
		ENDIF(${PROJECT_NAME}_srvfiles STREQUAL "sensor_msgs")
		#link nav_msgs
		IF(${PROJECT_NAME}_srvfiles STREQUAL "nav_msgs")
			SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
			SET(TMPDLLDEP "${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
		ENDIF(${PROJECT_NAME}_srvfiles STREQUAL "nav_msgs")
		#link visualization_msgs
		IF(${PROJECT_NAME}_srvfiles STREQUAL "visualization_msgs")
			SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
			SET(TMPDLLDEP "${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
		ENDIF(${PROJECT_NAME}_srvfiles STREQUAL "visualization_msgs")
		MESSAGE("SHOULD BUILD ${PROJECT_NAME}.Services")
		SEPARATE_ARGUMENTS(TMPLINK)
		ADD_CUSTOM_COMMAND(
			OUTPUT ${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.Services.dll ${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.Services.dll.mdb
			COMMAND ${CMAKE_CSHARP_COMPILER}
			ARGS -unsafe -debug+ -optimize+ -target:library -r:Mono.Posix.dll -r:${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll -r:${LIBRARY_OUTPUT_PATH}/RosCS.Services.dll ${TMPLINK} -out:${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.Services.dll ${PROJECT_SOURCE_DIR}/srv_cs_gen/csharp/${PROJECT_NAME}/*.cs
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}		    
			DEPENDS ${LIBRARY_OUTPUT_PATH}/RosCS.Services.dll ${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll ${PROJECT_SOURCE_DIR}/srv_cs_gen/csharp/AbstractService.cs
			COMMENT "Creating Csharp ${PROJECT_NAME}.Services.dll"
		)
		SET(TMPSRVLINK "${TMPSRVLINK} -r:${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.Services.dll")
		SET(TMPSRVDLLDEP "${TMPSRVDLLDEP} ${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.Services.dll")
		SET(csharp_is_building_lib_${PROJECT_NAME}.Services 1)
		SET("${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES" "${${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES} ${PROJECT_NAME}.Services")
	ENDIF(NOT ${PROJECT_NAME}_srvfiles STREQUAL "")

	SEPARATE_ARGUMENTS(TMPSRVDLLDEP)
	SEPARATE_ARGUMENTS(TMPSRVLINK)
	ADD_CUSTOM_COMMAND(
	    OUTPUT ${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.CommunicationServices.dll ${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.CommunicationServices.dll.mdb
	    COMMAND ${CMAKE_CSHARP_COMPILER}
	    ARGS -unsafe -debug+ -optimize+ -target:library -r:Mono.Posix.dll -r:${LIBRARY_OUTPUT_PATH}/RosCS.Services.dll -r:${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll ${TMPSRVLINK} -out:${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.CommunicationServices.dll  ${PROJECT_SOURCE_DIR}/srv_cs_gen/csharp/Service.cs ${PROJECT_SOURCE_DIR}/srv_cs_gen/csharp/ServiceClient.cs
	    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	    DEPENDS ${LIBRARY_OUTPUT_PATH}/RosCS.Services.dll ${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll ${PROJECT_SOURCE_DIR}/srv_cs_gen/csharp/ServiceClient.cs ${PROJECT_SOURCE_DIR}/srv_cs_gen/csharp/Service.cs ${PROJECT_SOURCE_DIR}/srv_cs_gen/csharp/AbstractService.cs ${TMPSRVDLLDEP}
	    COMMENT "Creating RosSharp wrapper library"
	  )
	SET(csharp_is_building_lib_${PROJECT_NAME}.CommunicationServices 1)
	SET("${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES" "${${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES} ${PROJECT_NAME}.CommunicationServices")
endmacro(rosbuild_gen_cs_srv)

# genmsg processes msg/*.msg files into cs
macro(rosbuild_gen_cs_msg)
	SET(TMP)
	FOREACH(it ${ARGN})
	SET(TMP "${TMP} ${it}")
	ENDFOREACH(it)
	rosbuild_invoke_rospack(roscs "csgenerate" bpath export --lang=cs --attrib=bpath)

	execute_process(
    		COMMAND ${csgenerate_bpath}/QueryDependencies.exe ${PROJECT_NAME}
		OUTPUT_VARIABLE _msgFiles
    		ERROR_VARIABLE _err
		RESULT_VARIABLE _failed
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
	SEPARATE_ARGUMENTS(_msgfiles)
	rosbuild_invoke_rospack(${PROJECT_NAME} "csmsgbuild" deps depends)
	string(REGEX REPLACE "\n" ";" csmsgbuild_deps "${csmsgbuild_deps}")
	SET(codeGenArgs)
	FOREACH(it ${csmsgbuild_deps})
		SET(codeGenArgs "${codeGenArgs} ${it}/*")
	ENDFOREACH(it)
	SEPARATE_ARGUMENTS(codeGenArgs)

	ADD_CUSTOM_COMMAND(
	    OUTPUT ${PROJECT_SOURCE_DIR}/msg_cs_gen/cpp/roscs.cpp ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/Message.cs ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/Timer.cs ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/RosSharp.cs ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/Node.cs ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/Publisher.cs
	    COMMAND ${csgenerate_bpath}/GenerateCode.exe
	    ARGS  ${PROJECT_NAME} ${codeGenArgs}
	    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	    DEPENDS ${_msgFiles}
	    COMMENT "Creating Csharp Wrapper Code"
	)
#	    ARGS  ${PROJECT_NAME} ${ARGN}
MESSAGE("Depends on packages: ${csmsgbuild_deps}")
	rosbuild_add_library(${PROJECT_NAME}.messages ${PROJECT_SOURCE_DIR}/msg_cs_gen/cpp/roscs.cpp)

	FOREACH(it ${csmsgbuild_deps})
		rosbuild_find_ros_package(${it})
	ENDFOREACH(it)
	#SET("${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES" "")

	ADD_CUSTOM_COMMAND(
		OUTPUT ${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll ${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll.mdb
		COMMAND ${CMAKE_CSHARP_COMPILER}
		ARGS -unsafe -debug+ -optimize+ -target:library -r:Mono.Posix.dll -out:${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll  ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/Message.cs
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}		    
		DEPENDS ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/Message.cs
		COMMENT "Creating Csharp RosCS.Messages.dll"
	)
	SET("${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES" "RosCS.Messages")

	FOREACH(it ${csmsgbuild_deps})
		rosbuild_invoke_rospack(${it} "csmsgdep" deps depends)
		string(REGEX REPLACE "\n" ";" csmsgdep_deps "${csmsgdep_deps}")
		SET(TMPLINK "")
		SET(TMPDLLDEP "")
		FOREACH(dd ${csmsgdep_deps})
			SET(XFILEEXISTS)			
			FILE(GLOB XFILEEXISTS ${${dd}_PACKAGE_PATH}/msg/*.msg)			
			IF(NOT XFILEEXISTS STREQUAL "")
			SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/${dd}.Messages.dll")
			SET(TMPDLLDEP "${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/${dd}.Messages.dll")
			ENDIF(NOT XFILEEXISTS STREQUAL "")
		ENDFOREACH(dd)

		#hack because rosdep is broken in fuerte version (see https://code.ros.org/trac/ros/ticket/3956)
		#link std_msgs
		IF(NOT ${it} STREQUAL "std_msgs")
			SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/std_msgs.Messages.dll")
			SET(TMPDLLDEP "${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/std_msgs.Messages.dll")
		ENDIF(NOT ${it} STREQUAL "std_msgs")
		#link geometry_msgs
		IF(${it} STREQUAL "sensor_msgs")
			SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
			SET(TMPDLLDEP "${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
		ENDIF(${it} STREQUAL "sensor_msgs")
		#link nav_msgs
		IF(${it} STREQUAL "nav_msgs")
			SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
			SET(TMPDLLDEP "${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
		ENDIF(${it} STREQUAL "nav_msgs")
		#link visualization_msgs
		IF(${it} STREQUAL "visualization_msgs")
			SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
			SET(TMPDLLDEP "${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/geometry_msgs.Messages.dll")
		ENDIF(${it} STREQUAL "visualization_msgs")
		SEPARATE_ARGUMENTS(TMPLINK)
		SEPARATE_ARGUMENTS(TMPDLLDEP)
MESSAGE("${it} depends on packages: ${TMPDLLDEP}")

		FILE(GLOB "${it}_msgfiles" ${${it}_PACKAGE_PATH}/msg/*.msg)
MESSAGE("CHECKING BUILD ${it}.Messages")
		IF(NOT ${it}_msgfiles STREQUAL "")
MESSAGE("SHOULD BUILD ${it}.Messages")
		ADD_CUSTOM_COMMAND(
			OUTPUT ${LIBRARY_OUTPUT_PATH}/${it}.Messages.dll ${LIBRARY_OUTPUT_PATH}/${it}.Messages.dll.mdb
	 	    COMMAND ${CMAKE_CSHARP_COMPILER}
		    ARGS -unsafe -debug+ -optimize+ -target:library -r:Mono.Posix.dll -r:${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll ${TMPLINK} -out:${LIBRARY_OUTPUT_PATH}/${it}.Messages.dll  ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/${it}/*.cs
		    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}		    
		    DEPENDS ${TMPDLLDEP} ${${it}_msgfiles} ${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll
		    COMMENT "Creating Csharp ${it}.Messages.dll"
		)
		SET(csharp_is_building_lib_${it}.Messages 1)
		SET("${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES" "${${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES} ${it}.Messages")
		ENDIF(NOT ${it}_msgfiles STREQUAL "")

	ENDFOREACH(it)
	SET(TMPLINK "")
	SET(TMPDLLDEP "")

	#link std_msgs
	SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/std_msgs.Messages.dll")
	SET(TMPDLLDEP "${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/std_msgs.Messages.dll")

	FOREACH(dd ${csmsgbuild_deps})
		FILE(GLOB XFILEEXISTS ${${dd}_PACKAGE_PATH}/msg/*.msg)			
		IF(NOT XFILEEXISTS STREQUAL "")
		SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/${dd}.Messages.dll")
		SET(TMPDLLDEP "${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/${dd}.Messages.dll")
		ENDIF(NOT XFILEEXISTS STREQUAL "")
	ENDFOREACH(dd)
	SEPARATE_ARGUMENTS(TMPLINK)
	SEPARATE_ARGUMENTS(TMPDLLDEP)
	FILE(GLOB "${PROJECT_NAME}_msgfiles" ${PROJECT_SOURCE_DIR}/msg/*.msg)
	IF(NOT ${PROJECT_NAME}_msgfiles STREQUAL "")
	ADD_CUSTOM_COMMAND(
		OUTPUT ${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.Messages.dll ${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.Messages.dll.mdb
		COMMAND ${CMAKE_CSHARP_COMPILER}
		ARGS -unsafe -debug+ -optimize+ -target:library -r:Mono.Posix.dll -r:${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll ${TMPLINK} -out:${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.Messages.dll  ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/${PROJECT_NAME}/*.cs
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}		    
		DEPENDS ${TMPDLLDEP} ${${PROJECT_NAME}_msgfiles} ${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll
		COMMENT "Creating Csharp ${PROJECT_NAME}.Messages.dll"
	)
	SET(csharp_is_building_lib_${PROJECT_NAME}.Messages 1)
	SET("${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES" "${${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES} ${PROJECT_NAME}.Messages")
	SET(TMPLINK "${TMPLINK} -r:${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.Messages.dll")
	SET(TMPDLLDEP "${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.Messages.dll")
	ENDIF(NOT ${PROJECT_NAME}_msgfiles STREQUAL "")
	
	SEPARATE_ARGUMENTS(TMPLINK)
	SEPARATE_ARGUMENTS(TMPDLLDEP)

	ADD_CUSTOM_COMMAND(
	    OUTPUT ${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.Communication.dll ${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.Communication.dll.mdb
	    COMMAND ${CMAKE_CSHARP_COMPILER}
	    ARGS -unsafe -debug+ -optimize+ -target:library -r:Mono.Posix.dll -r:${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll ${TMPLINK} -out:${LIBRARY_OUTPUT_PATH}/${PROJECT_NAME}.Communication.dll  ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/Node.cs ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/Timer.cs ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/Publisher.cs ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/RosSharp.cs
	    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	    DEPENDS ${TMPDLLDEP} ${LIBRARY_OUTPUT_PATH}/RosCS.Messages.dll ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/RosSharp.cs ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/Timer.cs ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/Node.cs ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/Publisher.cs
	    COMMENT "Creating RosSharp wrapper library"
	  )
	SET(csharp_is_building_lib_${PROJECT_NAME}.Communication 1)
	SET("${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES" "${${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES} ${PROJECT_NAME}.Communication")
endmacro(rosbuild_gen_cs_msg)


macro(CSHARP_MakeDoc) 
	foreach(it ${ARGN})
	set(CS_SUFFIX)
	if(${it}_is_exe)
		set(CS_SUFFIX ".exe")
	else(${it}_is_exe)
		set(CS_SUFFIX ".dll")		
	endif(${it}_is_exe)
	file(MAKE_DIRECTORY ${DOC_OUTPUT_PATH}/${it})
	ADD_CUSTOM_COMMAND(
	    OUTPUT ${DOC_OUTPUT_PATH}/${it}/index.xml
	    COMMAND monodocer 
	    ARGS -assembly:${LIBRARY_OUTPUT_PATH}/${it}${CS_SUFFIX} -importslashdoc:${DOC_OUTPUT_PATH}/${it}/${it}.xml -path:en -pretty --out=${DOC_OUTPUT_PATH}/${it}
	    WORKING_DIRECTORY ${DOC_OUTPUT_PATH}
	    DEPENDS ${LIBRARY_OUTPUT_PATH}/${it}${CS_SUFFIX}
	    COMMENT "Creating XML Documentation for ${it}"
	)	
	ADD_CUSTOM_COMMAND(
	    OUTPUT ${DOC_OUTPUT_PATH}/${it}/index.html
	    COMMAND mdoc
	    ARGS  export-html -o ${DOC_OUTPUT_PATH}/${it} ${DOC_OUTPUT_PATH}/${it}
	    WORKING_DIRECTORY ${DOC_OUTPUT_PATH}
	    DEPENDS ${DOC_OUTPUT_PATH}/${it}/index.xml
	    COMMENT "Creating HTML Documentation for ${it}"
	)
	ADD_CUSTOM_TARGET(CSharp_${it}_doc ALL
	    DEPENDS ${DOC_OUTPUT_PATH}/${it}/index.html
	  )
	endforeach(it)

endmacro(CSHARP_MakeDoc)
#macro(rosbuild_gen_cs_msg_bak)
#	SET(TMP)
#	FOREACH(it ${ARGN})
#	SET(TMP "${TMP} ${it}")
#	ENDFOREACH(it)
#	rosbuild_invoke_rospack(roscs "csgenerate" bpath export --lang=cs --attrib=bpath)
#	#rosbuild_get_msgs(_msglist)
#	#MESSAGE("hallo! ${_msglist}")
#	MESSAGE("exe ${csgenerate_bpath}/GenerateCode.exe ${PROJECT_NAME} ${ARGN}")
#	execute_process(
#   		COMMAND ${csgenerate_bpath}/GenerateCode.exe ${PROJECT_NAME} ${ARGN}
#		OUTPUT_VARIABLE _out
#   		ERROR_VARIABLE _err
#		RESULT_VARIABLE _failed
#		OUTPUT_STRIP_TRAILING_WHITESPACE
#	)
#	MESSAGE("Out: ${_out}")
#	MESSAGE("Err: ${_err}")
#	FILE(GLOB GEN_CPP_FILES ${PROJECT_SOURCE_DIR}/msg_cs_gen/cpp/*.cpp)
#	rosbuild_add_library(${PROJECT_NAME}.messages ${GEN_CPP_FILES})
#
#	SET("${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES" "")
#	FILE(GLOB GEN_CS_FILES ${PROJECT_SOURCE_DIR}/msg_cs_gen/csharp/*.cs)
#	CSHARP_ADD_TARGET_DEPENDENCY(${PROJECT_NAME}.Messages Mono.Posix)
#	CSHARP_ADD_LIBRARY(${PROJECT_NAME}.Messages ${GEN_CS_FILES})
#	SET("${PROJECT_NAME}_CSMESSAGE_DEPENDENCIES" "${PROJECT_NAME}.Messages")
#	
#endmacro(rosbuild_gen_cs_msg_bak)




