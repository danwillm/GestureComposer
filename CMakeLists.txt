cmake_minimum_required(VERSION 3.16)
project(GestureComposer VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(GC_IS_TOP_LEVEL ON)
else()
  set(GC_IS_TOP_LEVEL OFF)
endif()

option(GC_BUILD_TESTS            "Build GestureComposer tests"            ${GC_IS_TOP_LEVEL})
option(GC_BUILD_EXAMPLES         "Build GestureComposer examples"         ${GC_IS_TOP_LEVEL})
option(GC_BUILD_BENCHMARKS       "Build GestureComposer benchmarks"       ${GC_IS_TOP_LEVEL})
option(GC_BUILD_OPENXR_ADAPTER   "Provide the OpenXR adapter target"      ON)
option(GC_BUILD_VISIONOS_ADAPTER "Provide the visionOS adapter target"    ON)
option(GC_NO_EXCEPTIONS          "Compile the library without exceptions/RTTI" OFF)

add_library(gesture_composer STATIC
  src/composer.cpp
  src/gestures.cpp
  src/hand_tracker.cpp
  src/recorder.cpp
  src/synthetic_hand.cpp
  src/types.cpp
)
add_library(gesture_composer::gesture_composer ALIAS gesture_composer)
target_include_directories(gesture_composer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(gesture_composer PUBLIC cxx_std_17)
set_target_properties(gesture_composer PROPERTIES CXX_EXTENSIONS OFF POSITION_INDEPENDENT_CODE ON)

if(MSVC)
  target_compile_options(gesture_composer PRIVATE /W4 /permissive-)
  target_compile_definitions(gesture_composer PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
  target_compile_options(gesture_composer PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(GC_NO_EXCEPTIONS)
  if(MSVC)
    target_compile_options(gesture_composer PRIVATE /GR- /EHs-c-)
    target_compile_definitions(gesture_composer PRIVATE _HAS_EXCEPTIONS=0)
  else()
    target_compile_options(gesture_composer PRIVATE -fno-exceptions -fno-rtti)
  endif()
endif()

# Adapters are header-only INTERFACE targets.
if(GC_BUILD_OPENXR_ADAPTER)
  add_library(gesture_composer_openxr INTERFACE)
  add_library(gesture_composer::openxr ALIAS gesture_composer_openxr)
  target_include_directories(gesture_composer_openxr INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/adapters/openxr/include>)
  target_link_libraries(gesture_composer_openxr INTERFACE gesture_composer)
  # The consumer provides the OpenXR headers (e.g. link OpenXR::headers or OpenXR::openxr_loader).
  if(TARGET OpenXR::headers)
    target_link_libraries(gesture_composer_openxr INTERFACE OpenXR::headers)
  endif()
endif()

if(GC_BUILD_VISIONOS_ADAPTER)
  add_library(gesture_composer_visionos INTERFACE)
  add_library(gesture_composer::visionos ALIAS gesture_composer_visionos)
  target_include_directories(gesture_composer_visionos INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/adapters/visionos/include>)
  target_link_libraries(gesture_composer_visionos INTERFACE gesture_composer)
endif()

if(GC_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(GC_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(GC_BUILD_BENCHMARKS)
  add_subdirectory(bench)
endif()
