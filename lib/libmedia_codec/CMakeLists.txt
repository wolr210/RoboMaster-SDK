cmake_minimum_required(VERSION 3.15...3.31)
project (robomaster_media_codec)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/output)

set(PYBIND11_FINDPYTHON ON)
add_subdirectory(pybind11)

pybind11_add_module(libmedia_codec src/media_codec.cpp)

IF (CMAKE_SYSTEM_NAME MATCHES "Windows")
	SET(FFMPEG_ROOT "${CMAKE_SOURCE_DIR}/src/ffmpeg-4.2.2-win64-dev")
	SET(FFMPEG_INCLUDE_DIRS "${FFMPEG_ROOT}/include")

	# The vendored ffmpeg dev package only ships MinGW import libraries
	# (lib*.dll.a), which link.exe cannot read. Build MSVC import libraries out
	# of the .def files that ship next to them instead.
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(FFMPEG_IMPLIB_MACHINE X64)
	else()
		set(FFMPEG_IMPLIB_MACHINE X86)
	endif()

	get_filename_component(MSVC_TOOL_DIR "${CMAKE_CXX_COMPILER}" DIRECTORY)
	find_program(MSVC_LIB_TOOL NAMES lib.exe HINTS "${MSVC_TOOL_DIR}")
	if(NOT MSVC_LIB_TOOL)
		message(FATAL_ERROR "lib.exe was not found next to ${CMAKE_CXX_COMPILER}")
	endif()

	set(FFMPEG_IMPLIB_DIR "${CMAKE_CURRENT_BINARY_DIR}/implib")
	file(MAKE_DIRECTORY "${FFMPEG_IMPLIB_DIR}")

	set(FFMPEG_LIBRARIES "")
	foreach(entry "avcodec:avcodec-58" "avutil:avutil-56" "swscale:swscale-5" "swresample:swresample-3")
		string(REPLACE ":" ";" parts "${entry}")
		list(GET parts 0 lib_name)
		list(GET parts 1 dll_name)
		set(implib "${FFMPEG_IMPLIB_DIR}/${lib_name}.lib")
		set(def_source "${FFMPEG_ROOT}/lib/${dll_name}.def")
		if(NOT EXISTS "${implib}")
			if(NOT EXISTS "${def_source}")
				message(FATAL_ERROR "Missing ${def_source}, cannot build an import library for ${dll_name}.dll")
			endif()
			# The shipped .def files have no LIBRARY statement, so name the DLL
			# explicitly: the module has to import from ${dll_name}.dll.
			file(READ "${def_source}" def_body)
			set(def_generated "${FFMPEG_IMPLIB_DIR}/${lib_name}.def")
			file(WRITE "${def_generated}" "LIBRARY ${dll_name}.dll\n${def_body}")
			execute_process(
				COMMAND "${MSVC_LIB_TOOL}" /nologo "/def:${def_generated}"
				        /machine:${FFMPEG_IMPLIB_MACHINE} "/out:${implib}"
				RESULT_VARIABLE lib_result
				OUTPUT_VARIABLE lib_output
				ERROR_VARIABLE lib_error)
			if(NOT lib_result EQUAL 0)
				message(FATAL_ERROR "lib.exe failed for ${dll_name}.dll:\n${lib_output}\n${lib_error}")
			endif()
			message(STATUS "Generated import library ${implib}")
		endif()
		list(APPEND FFMPEG_LIBRARIES "${implib}")
	endforeach()

	target_link_libraries(libmedia_codec PRIVATE ${FFMPEG_LIBRARIES})
	target_include_directories(libmedia_codec PRIVATE ${FFMPEG_INCLUDE_DIRS})
ELSE ()
	# Ask pkg-config where the system ffmpeg lives: on Debian/Raspberry Pi OS
	# the headers sit under a multiarch include directory.
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
			libavcodec libavutil libswscale libswresample)
	message(STATUS "Using system ffmpeg ${FFMPEG_libavcodec_VERSION} (libavcodec)")
	target_link_libraries(libmedia_codec PRIVATE PkgConfig::FFMPEG)
ENDIF (CMAKE_SYSTEM_NAME MATCHES "Windows")
