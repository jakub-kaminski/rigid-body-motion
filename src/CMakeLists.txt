cmake_minimum_required(VERSION 3.16)

add_library(RBMotion STATIC
        Kine.cpp
        RBLink.cpp
        RBTreeParser.cpp
        FKSolverInterface.cpp
        Utils.cpp
        RobotInterface.cpp
        )

target_include_directories(RBMotion PUBLIC
        ../lib/Eigen
        ../lib/URDF_Parser/include
        )

target_link_libraries(RBMotion urdfparser)