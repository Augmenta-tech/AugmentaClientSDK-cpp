cmake_minimum_required(VERSION 3.15)

project(
    AugmentaServerLib
    VERSION 0.1
    LANGUAGES CXX
)

add_library(AugmentaServerLib
    src/AugmentaServerLib.cpp
)

set_target_properties(AugmentaServerLib 
    PROPERTIES
        CXX_STANDARD 17
)

target_include_directories(AugmentaServerLib
    PRIVATE
        external
    PUBLIC
        include
)