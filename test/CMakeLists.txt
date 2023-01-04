cmake_minimum_required(VERSION 3.8)

include(FetchContent)

FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.1.0)

FetchContent_MakeAvailable(Catch2)

# COMMENT: First, set a variable for all the required include directories. Using '..' so that it is always relative
# to the current directory, no matter where the top level CMakeLists.txt is
set(INCLUDES
        ../include/RBMotion
        ../lib/URDF_Parser/include
        ../lib/Eigen
        )

# COMMENT: This example does not compile so I left it commented out.
# add_executable(sampleTest sampleTest.cpp)
# target_include_directories(sampleTest PRIVATE ${INCLUDES})
# target_link_libraries(sampleTest RBMotion urdfparser)


add_executable(quatTest quatTest.cpp)
target_include_directories(quatTest PRIVATE ${INCLUDES})
target_link_libraries(quatTest RBMotion urdfparser)


add_executable(quatTut quatTut.cpp)
target_include_directories(quatTut PRIVATE ${INCLUDES})
target_link_libraries(quatTut RBMotion urdfparser)


add_executable(2dofTest 2dofTest.cpp)
target_include_directories(2dofTest PRIVATE ${INCLUDES})
target_link_libraries(2dofTest RBMotion urdfparser)


add_executable(vicariousTest vicariousTest.cpp Vicarious.cpp VicariousRobot.cpp)
target_include_directories(vicariousTest PRIVATE ${INCLUDES})
target_link_libraries(vicariousTest RBMotion urdfparser)


add_executable(vicariousTestLeft vicariousTestLeft.cpp Vicarious.cpp VicariousRobot.cpp)
target_include_directories(vicariousTestLeft PRIVATE ${INCLUDES})
target_link_libraries(vicariousTestLeft RBMotion urdfparser)


add_executable(UtilsTest UtilsTest.cpp)
target_include_directories(UtilsTest PRIVATE ${INCLUDES})
target_link_libraries(UtilsTest RBMotion urdfparser)

add_executable(vicariousTestMove vicariousTestMove.cpp Vicarious.cpp VicariousRobot.cpp)
target_include_directories(vicariousTestMove PRIVATE ${INCLUDES})
target_link_libraries(vicariousTestMove RBMotion urdfparser)
