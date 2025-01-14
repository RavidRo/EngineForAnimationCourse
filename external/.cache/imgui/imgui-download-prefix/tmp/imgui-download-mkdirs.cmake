# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Dev/EngineForAnimationCourse/cmake/../external/imgui"
  "C:/Dev/EngineForAnimationCourse/build/imgui-build"
  "C:/Dev/EngineForAnimationCourse/external/.cache/imgui/imgui-download-prefix"
  "C:/Dev/EngineForAnimationCourse/external/.cache/imgui/imgui-download-prefix/tmp"
  "C:/Dev/EngineForAnimationCourse/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp"
  "C:/Dev/EngineForAnimationCourse/external/.cache/imgui/imgui-download-prefix/src"
  "C:/Dev/EngineForAnimationCourse/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp"
)

set(configSubDirs Debug;Release;MinSizeRel;RelWithDebInfo)
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Dev/EngineForAnimationCourse/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/${subDir}")
endforeach()
