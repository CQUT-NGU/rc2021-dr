# include compiler flag check
include(CheckCCompilerFlag)
include(CheckCXXCompilerFlag)

# compiler is gnu or clang
if("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU" OR
   "${CMAKE_C_COMPILER_ID}" STREQUAL "Clang" OR
   "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
   "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")

  # C check -Wno-packed
  check_c_compiler_flag(-Wno-packed FLAG_SUPPORTED_Wno-packed)
  if(FLAG_SUPPORTED_Wno-packed)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-packed")
  endif(FLAG_SUPPORTED_Wno-packed)
  # C++ check -Wno-packed
  check_cxx_compiler_flag(-Wno-packed FLAG_SUPPORTED_Wno-packed)
  if(FLAG_SUPPORTED_Wno-packed)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-packed")
  endif(FLAG_SUPPORTED_Wno-packed)

endif("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU" OR
      "${CMAKE_C_COMPILER_ID}" STREQUAL "Clang" OR
      "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
      "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
