cmake_minimum_required(VERSION 3.22.1)
project(questcapture LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)
FetchContent_Declare(
    meta_openxr_sdk
    URL https://codeload.github.com/meta-quest/Meta-OpenXR-SDK/tar.gz/bbed2f20e38a5df7113630771c83cb8279e4fc26
    URL_HASH SHA256=0edf48d28b2e87e826d398d839d9cbeb3d0386b0a29a1b3bbfabda4d3e2255bf
)
FetchContent_MakeAvailable(meta_openxr_sdk)

add_subdirectory(
    ${meta_openxr_sdk_SOURCE_DIR}/Samples/3rdParty
    ${meta_openxr_sdk_BINARY_DIR}/3rdParty
)
add_subdirectory(
    ${meta_openxr_sdk_SOURCE_DIR}/Samples/SampleXrFramework
    ${meta_openxr_sdk_BINARY_DIR}/SampleXrFramework
)

add_library(questcapture SHARED src/main/cpp/quest_capture.cpp)
target_compile_definitions(questcapture PRIVATE XR_USE_TIMESPEC)
target_link_libraries(questcapture PRIVATE samplexrframework)
target_link_options(questcapture PRIVATE "-uANativeActivity_onCreate")
