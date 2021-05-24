# include compiler flag check
include(CheckCompilerFlag)
include(CheckCCompilerFlag)
include(CheckCXXCompilerFlag)

# compiler is gnu or clang
if("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU" OR
   "${CMAKE_C_COMPILER_ID}" STREQUAL "Clang" OR
   "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
   "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")

  # C/C++ build release
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

  # C check -Wall
  check_c_compiler_flag(-Wall FLAG_SUPPORTED_Wall)
  if(FLAG_SUPPORTED_Wall)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
  endif(FLAG_SUPPORTED_Wall)
  # C++ check -Wall
  check_cxx_compiler_flag(-Wall FLAG_SUPPORTED_Wall)
  if(FLAG_SUPPORTED_Wall)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
  endif(FLAG_SUPPORTED_Wall)

  # C check -Wextra
  check_c_compiler_flag(-Wextra FLAG_SUPPORTED_Wextra)
  if(FLAG_SUPPORTED_Wextra)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wextra")
  endif(FLAG_SUPPORTED_Wextra)
  # C++ check -Wextra
  check_cxx_compiler_flag(-Wextra FLAG_SUPPORTED_Wextra)
  if(FLAG_SUPPORTED_Wextra)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra")
  endif(FLAG_SUPPORTED_Wextra)

  # C check -pedantic
  check_c_compiler_flag(-pedantic FLAG_SUPPORTED_pedantic)
  if(FLAG_SUPPORTED_pedantic)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pedantic")
  endif(FLAG_SUPPORTED_pedantic)
  # C++ check -pedantic
  check_cxx_compiler_flag(-pedantic FLAG_SUPPORTED_pedantic)
  if(FLAG_SUPPORTED_pedantic)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")
  endif(FLAG_SUPPORTED_pedantic)

  # C check -Wno-pedantic
  check_c_compiler_flag(-Wno-pedantic FLAG_SUPPORTED_Wno-pedantic)
  if(FLAG_SUPPORTED_Wno-pedantic)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-pedantic")
  endif(FLAG_SUPPORTED_Wno-pedantic)
  # C++ check -Wno-pedantic
  check_cxx_compiler_flag(-Wno-pedantic FLAG_SUPPORTED_Wno-pedantic)
  if(FLAG_SUPPORTED_Wno-pedantic)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pedantic")
  endif(FLAG_SUPPORTED_Wno-pedantic)

  # C check -Wimplicit-fallthrough
  check_c_compiler_flag(-Wimplicit-fallthrough FLAG_SUPPORTED_Wimplicit-fallthrough)
  if(FLAG_SUPPORTED_Wimplicit-fallthrough)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wimplicit-fallthrough")
  endif(FLAG_SUPPORTED_Wimplicit-fallthrough)
  # C++ check -Wimplicit-fallthrough
  check_cxx_compiler_flag(-Wimplicit-fallthrough FLAG_SUPPORTED_Wimplicit-fallthrough)
  if(FLAG_SUPPORTED_Wimplicit-fallthrough)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wimplicit-fallthrough")
  endif(FLAG_SUPPORTED_Wimplicit-fallthrough)

  # C check -Wsequence-point
  check_c_compiler_flag(-Wsequence-point FLAG_SUPPORTED_Wsequence-point)
  if(FLAG_SUPPORTED_Wsequence-point)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wsequence-point")
  endif(FLAG_SUPPORTED_Wsequence-point)
  # C++ check -Wsequence-point
  check_cxx_compiler_flag(-Wsequence-point FLAG_SUPPORTED_Wsequence-point)
  if(FLAG_SUPPORTED_Wsequence-point)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wsequence-point")
  endif(FLAG_SUPPORTED_Wsequence-point)

  # C check -Wswitch-default
  check_c_compiler_flag(-Wswitch-default FLAG_SUPPORTED_Wswitch-default)
  if(FLAG_SUPPORTED_Wswitch-default)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wswitch-default")
  endif(FLAG_SUPPORTED_Wswitch-default)
  # C++ check -Wswitch-default
  check_cxx_compiler_flag(-Wswitch-default FLAG_SUPPORTED_Wswitch-default)
  if(FLAG_SUPPORTED_Wswitch-default)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wswitch-default")
  endif(FLAG_SUPPORTED_Wswitch-default)

  # C check -Wswitch-enum
  check_c_compiler_flag(-Wswitch-enum FLAG_SUPPORTED_Wswitch-enum)
  if(FLAG_SUPPORTED_Wswitch-enum)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wswitch-enum")
  endif(FLAG_SUPPORTED_Wswitch-enum)
  # C++ check -Wswitch-enum
  check_cxx_compiler_flag(-Wswitch-enum FLAG_SUPPORTED_Wswitch-enum)
  if(FLAG_SUPPORTED_Wswitch-enum)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wswitch-enum")
  endif(FLAG_SUPPORTED_Wswitch-enum)

  # C check -Wtautological-compare
  check_c_compiler_flag(-Wtautological-compare FLAG_SUPPORTED_Wtautological-compare)
  if(FLAG_SUPPORTED_Wtautological-compare)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wtautological-compare")
  endif(FLAG_SUPPORTED_Wtautological-compare)
  # C++ check -Wtautological-compare
  check_cxx_compiler_flag(-Wtautological-compare FLAG_SUPPORTED_Wtautological-compare)
  if(FLAG_SUPPORTED_Wtautological-compare)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wtautological-compare")
  endif(FLAG_SUPPORTED_Wtautological-compare)

  # C check -Wfloat-equal
  check_c_compiler_flag(-Wfloat-equal FLAG_SUPPORTED_Wfloat-equal)
  if(FLAG_SUPPORTED_Wfloat-equal)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wfloat-equal")
  endif(FLAG_SUPPORTED_Wfloat-equal)
  # C++ check -Wfloat-equal
  check_cxx_compiler_flag(-Wfloat-equal FLAG_SUPPORTED_Wfloat-equal)
  if(FLAG_SUPPORTED_Wfloat-equal)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfloat-equal")
  endif(FLAG_SUPPORTED_Wfloat-equal)

  # C check -Wpointer-arith
  check_c_compiler_flag(-Wpointer-arith FLAG_SUPPORTED_Wpointer-arith)
  if(FLAG_SUPPORTED_Wpointer-arith)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wpointer-arith")
  endif(FLAG_SUPPORTED_Wpointer-arith)
  # C++ check -Wpointer-arith
  check_cxx_compiler_flag(-Wpointer-arith FLAG_SUPPORTED_Wpointer-arith)
  if(FLAG_SUPPORTED_Wpointer-arith)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpointer-arith")
  endif(FLAG_SUPPORTED_Wpointer-arith)

  # C check -Wpointer-compare
  check_c_compiler_flag(-Wpointer-compare FLAG_SUPPORTED_Wpointer-compare)
  if(FLAG_SUPPORTED_Wpointer-compare)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wpointer-compare")
  endif(FLAG_SUPPORTED_Wpointer-compare)
  # C++ check -Wpointer-compare
  check_cxx_compiler_flag(-Wpointer-compare FLAG_SUPPORTED_Wpointer-compare)
  if(FLAG_SUPPORTED_Wpointer-compare)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpointer-compare")
  endif(FLAG_SUPPORTED_Wpointer-compare)

  # C check -Wcast-align
  check_c_compiler_flag(-Wcast-align FLAG_SUPPORTED_Wcast-align)
  if(FLAG_SUPPORTED_Wcast-align)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wcast-align")
  endif(FLAG_SUPPORTED_Wcast-align)
  # C++ check -Wcast-align
  check_cxx_compiler_flag(-Wcast-align FLAG_SUPPORTED_Wcast-align)
  if(FLAG_SUPPORTED_Wcast-align)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wcast-align")
  endif(FLAG_SUPPORTED_Wcast-align)

  # C check -Wcast-qual
  check_c_compiler_flag(-Wcast-qual FLAG_SUPPORTED_Wcast-qual)
  if(FLAG_SUPPORTED_Wcast-qual)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wcast-qual")
  endif(FLAG_SUPPORTED_Wcast-qual)
  # C++ check -Wcast-qual
  check_cxx_compiler_flag(-Wcast-qual FLAG_SUPPORTED_Wcast-qual)
  if(FLAG_SUPPORTED_Wcast-qual)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wcast-qual")
  endif(FLAG_SUPPORTED_Wcast-qual)

  # C check -Wwrite-strings
  check_c_compiler_flag(-Wwrite-strings FLAG_SUPPORTED_Wwrite-strings)
  if(FLAG_SUPPORTED_Wwrite-strings)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wwrite-strings")
  endif(FLAG_SUPPORTED_Wwrite-strings)
  # C++ check -Wwrite-strings
  check_cxx_compiler_flag(-Wwrite-strings FLAG_SUPPORTED_Wwrite-strings)
  if(FLAG_SUPPORTED_Wwrite-strings)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wwrite-strings")
  endif(FLAG_SUPPORTED_Wwrite-strings)

  # C check -Wdangling-else
  check_c_compiler_flag(-Wdangling-else FLAG_SUPPORTED_Wdangling-else)
  if(FLAG_SUPPORTED_Wdangling-else)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wdangling-else")
  endif(FLAG_SUPPORTED_Wdangling-else)
  # C++ check -Wdangling-else
  check_cxx_compiler_flag(-Wdangling-else FLAG_SUPPORTED_Wdangling-else)
  if(FLAG_SUPPORTED_Wdangling-else)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wdangling-else")
  endif(FLAG_SUPPORTED_Wdangling-else)

  # C check -Wconversion
  check_c_compiler_flag(-Wconversion FLAG_SUPPORTED_Wconversion)
  if(FLAG_SUPPORTED_Wconversion)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wconversion")
  endif(FLAG_SUPPORTED_Wconversion)
  # C++ check -Wconversion
  check_cxx_compiler_flag(-Wconversion FLAG_SUPPORTED_Wconversion)
  if(FLAG_SUPPORTED_Wconversion)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wconversion")
  endif(FLAG_SUPPORTED_Wconversion)

  # C check -Wstring-compare
  check_c_compiler_flag(-Wstring-compare FLAG_SUPPORTED_Wstring-compare)
  if(FLAG_SUPPORTED_Wstring-compare)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wstring-compare")
  endif(FLAG_SUPPORTED_Wstring-compare)
  # C++ check -Wstring-compare
  check_cxx_compiler_flag(-Wstring-compare FLAG_SUPPORTED_Wstring-compare)
  if(FLAG_SUPPORTED_Wstring-compare)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wstring-compare")
  endif(FLAG_SUPPORTED_Wstring-compare)

endif("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU" OR
      "${CMAKE_C_COMPILER_ID}" STREQUAL "Clang" OR
      "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
      "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")

# compiler is gnu
if("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU" OR
   "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")

  # C check -Wswitch-unreachable
  check_c_compiler_flag(-Wswitch-unreachable FLAG_SUPPORTED_Wswitch-unreachable)
  if(FLAG_SUPPORTED_Wswitch-unreachable)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wswitch-unreachable")
  endif(FLAG_SUPPORTED_Wswitch-unreachable)
  # C++ check -Wswitch-unreachable
  check_cxx_compiler_flag(-Wswitch-unreachable FLAG_SUPPORTED_Wswitch-unreachable)
  if(FLAG_SUPPORTED_Wswitch-unreachable)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wswitch-unreachable")
  endif(FLAG_SUPPORTED_Wswitch-unreachable)

  # C check -Wstringop-truncation
  check_c_compiler_flag(-Wstringop-truncation FLAG_SUPPORTED_Wstringop-truncation)
  if(FLAG_SUPPORTED_Wstringop-truncation)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wstringop-truncation")
  endif(FLAG_SUPPORTED_Wstringop-truncation)
  # C++ check -Wstringop-truncation
  check_cxx_compiler_flag(-Wstringop-truncation FLAG_SUPPORTED_Wstringop-truncation)
  if(FLAG_SUPPORTED_Wstringop-truncation)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wstringop-truncation")
  endif(FLAG_SUPPORTED_Wstringop-truncation)

  # C check -Wbool-compare
  check_c_compiler_flag(-Wbool-compare FLAG_SUPPORTED_Wbool-compare)
  if(FLAG_SUPPORTED_Wbool-compare)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wbool-compare")
  endif(FLAG_SUPPORTED_Wbool-compare)
  # C++ check -Wbool-compare
  check_cxx_compiler_flag(-Wbool-compare FLAG_SUPPORTED_Wbool-compare)
  if(FLAG_SUPPORTED_Wbool-compare)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wbool-compare")
  endif(FLAG_SUPPORTED_Wbool-compare)

  # C check -Wshadow=global
  check_c_compiler_flag(-Wshadow=global FLAG_SUPPORTED_Wshadow)
  if(FLAG_SUPPORTED_Wshadow)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wshadow=global")
  endif(FLAG_SUPPORTED_Wshadow)
  # C++ check -Wshadow=global
  check_cxx_compiler_flag(-Wshadow=global FLAG_SUPPORTED_Wshadow)
  if(FLAG_SUPPORTED_Wshadow)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wshadow=global")
  endif(FLAG_SUPPORTED_Wshadow)

  # C check -Wlogical-op
  check_c_compiler_flag(-Wlogical-op FLAG_SUPPORTED_Wlogical-op)
  if(FLAG_SUPPORTED_Wlogical-op)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wlogical-op")
  endif(FLAG_SUPPORTED_Wlogical-op)
  # C++ check -Wlogical-op
  check_cxx_compiler_flag(-Wlogical-op FLAG_SUPPORTED_Wlogical-op)
  if(FLAG_SUPPORTED_Wlogical-op)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wlogical-op")
  endif(FLAG_SUPPORTED_Wlogical-op)

  # C check -Wno-pedantic-ms-format
  check_c_compiler_flag(-Wno-pedantic-ms-format FLAG_SUPPORTED_Wno-pedantic-ms-format)
  if(FLAG_SUPPORTED_Wno-pedantic-ms-format)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-pedantic-ms-format")
  endif(FLAG_SUPPORTED_Wno-pedantic-ms-format)
  # C++ check -Wno-pedantic-ms-format
  check_cxx_compiler_flag(-Wno-pedantic-ms-format FLAG_SUPPORTED_Wno-pedantic-ms-format)
  if(FLAG_SUPPORTED_Wno-pedantic-ms-format)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pedantic-ms-format")
  endif(FLAG_SUPPORTED_Wno-pedantic-ms-format)

endif("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU" OR
      "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")

# compiler is clang
if("${CMAKE_C_COMPILER_ID}" STREQUAL "Clang" OR
   "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")

  # C check -Wstring-concatenation
  check_c_compiler_flag(-Wstring-concatenation FLAG_SUPPORTED_Wstring-concatenation)
  if(FLAG_SUPPORTED_Wstring-concatenation)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wstring-concatenation")
  endif(FLAG_SUPPORTED_Wstring-concatenation)
  # C++ check -Wstring-concatenation
  check_cxx_compiler_flag(-Wstring-concatenation FLAG_SUPPORTED_Wstring-concatenation)
  if(FLAG_SUPPORTED_Wstring-concatenation)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wstring-concatenation")
  endif(FLAG_SUPPORTED_Wstring-concatenation)

  # C check -Wshadow-all
  check_c_compiler_flag(-Wshadow-all FLAG_SUPPORTED_Wshadow-all)
  if(FLAG_SUPPORTED_Wshadow-all)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wshadow-all")
  endif(FLAG_SUPPORTED_Wshadow-all)
  # C++ check -Wshadow-all
  check_cxx_compiler_flag(-Wshadow-all FLAG_SUPPORTED_Wshadow-all)
  if(FLAG_SUPPORTED_Wshadow-all)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wshadow-all")
  endif(FLAG_SUPPORTED_Wshadow-all)

endif("${CMAKE_C_COMPILER_ID}" STREQUAL "Clang" OR
      "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")

# compiler is msvc
if("${CMAKE_C_COMPILER_ID}" STREQUAL "MSVC" OR
   "${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")

  # nologo
  add_compile_options(/nologo)
  add_link_options(/nologo)

  # Remove unsafe warnings
  add_definitions(/D _CRT_SECURE_NO_WARNINGS)

endif("${CMAKE_C_COMPILER_ID}" STREQUAL "MSVC" OR
      "${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
