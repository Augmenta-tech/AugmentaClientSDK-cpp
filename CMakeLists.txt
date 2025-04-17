cmake_minimum_required(VERSION 3.15)

include(FetchContent)

project(
    AugmentaClientSDK
    VERSION 0.1
    LANGUAGES CXX
)

add_library(AugmentaClientSDK
    src/AugmentaClientSDK.cpp
)

set_target_properties(AugmentaClientSDK 
    PROPERTIES
        CXX_STANDARD 17
)

target_include_directories(AugmentaClientSDK
    PRIVATE
        external
    PUBLIC
        include
)

set(ZSTD_BUILD_STATIC ON)
set(ZSTD_BUILD_SHARED OFF)
set(ZSTD_BUILD_COMPRESSION ON)
set(ZSTD_BUILD_DECOMPRESSION ON)
set(ZSTD_BUILD_DICTBUILDER ON)
set(ZSTD_BUILD_DEPRECATED OFF)
set(ZSTD_LEGACY_SUPPORT OFF)

FetchContent_Declare(
    zstd
    GIT_REPOSITORY https://github.com/facebook/zstd.git
    GIT_TAG 794ea1b0afca0f020f4e57b6732332231fb23c70 # v1.5.6
    SOURCE_SUBDIR build/cmake
)

FetchContent_MakeAvailable(zstd)

target_link_libraries(AugmentaClientSDK
    PRIVATE
        libzstd
)