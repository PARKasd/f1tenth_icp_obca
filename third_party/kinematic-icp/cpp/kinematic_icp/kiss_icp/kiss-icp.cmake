# MIT License
#
# Copyright (c) 2024 Tiziano Guadagnino, Benedikt Mersch, Ignacio Vizzo, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# Silence timestamp warning
if(CMAKE_VERSION VERSION_GREATER 3.24)
  cmake_policy(SET CMP0135 OLD)
endif()

include(FetchContent)
# Offline-first: prefer in-repo vendored copies (third_party/) so builds work on
# machines without internet access (e.g. the car's Jetson). Falls back to the
# upstream tarballs when the vendored tree is absent.
get_filename_component(_kinematic_icp_third_party "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
if(EXISTS "${_kinematic_icp_third_party}/kiss-icp/cpp/kiss_icp/CMakeLists.txt")
  # FETCHCONTENT_SOURCE_DIR_* makes FetchContent skip download AND patch steps;
  # third_party/sophus therefore ships with sophus.patch already applied.
  set(FETCHCONTENT_SOURCE_DIR_KISS_ICP "${_kinematic_icp_third_party}/kiss-icp" CACHE PATH "" FORCE)
  set(FETCHCONTENT_SOURCE_DIR_SOPHUS "${_kinematic_icp_third_party}/sophus" CACHE PATH "" FORCE)
  set(FETCHCONTENT_SOURCE_DIR_TESSIL "${_kinematic_icp_third_party}/robin-map" CACHE PATH "" FORCE)
endif()
FetchContent_Declare(kiss_icp URL https://github.com/PRBonn/kiss-icp/archive/refs/tags/v1.2.0.tar.gz SOURCE_SUBDIR
                                  cpp/kiss_icp)
FetchContent_MakeAvailable(kiss_icp)
