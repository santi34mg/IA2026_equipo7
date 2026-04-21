# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

if(DEFINED ENV{IDF_PATH} AND NOT "$ENV{IDF_PATH}" STREQUAL "")
	set(_idf_path "$ENV{IDF_PATH}")
elseif(EXISTS "$ENV{HOME}/.espressif/v6.0/esp-idf/tools/cmake/project.cmake")
	set(_idf_path "$ENV{HOME}/.espressif/v6.0/esp-idf")
else()
	message(FATAL_ERROR "ESP-IDF not found. Set IDF_PATH or install ESP-IDF under ~/.espressif/v6.0/esp-idf")
endif()

if(EXISTS "$ENV{HOME}/.espressif/tools/xtensa-esp-elf")
	file(GLOB _xtensa_bins "$ENV{HOME}/.espressif/tools/xtensa-esp-elf/*/xtensa-esp-elf/bin")
	list(LENGTH _xtensa_bins _xtensa_bins_count)
	if(_xtensa_bins_count GREATER 0)
		list(SORT _xtensa_bins)
		list(GET _xtensa_bins -1 _xtensa_bin)
		set(ENV{PATH} "${_xtensa_bin}:$ENV{PATH}")
	endif()
endif()

if(EXISTS "$ENV{HOME}/.espressif/tools/python/v6.0/venv/bin/python")
	set(_idf_python "$ENV{HOME}/.espressif/tools/python/v6.0/venv/bin/python")
	set(PYTHON "${_idf_python}" CACHE STRING "Python interpreter for ESP-IDF" FORCE)
	set(ENV{PYTHON} "${_idf_python}")
elseif(NOT DEFINED ENV{PYTHON} OR "$ENV{PYTHON}" STREQUAL "")
	find_program(_python_exe NAMES python3 python)
	if(_python_exe)
		set(PYTHON "${_python_exe}" CACHE STRING "Python interpreter for ESP-IDF" FORCE)
		set(ENV{PYTHON} "${_python_exe}")
	endif()
endif()

set(PYTHON_DEPS_CHECKED 1)

include(${_idf_path}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(embedded)
