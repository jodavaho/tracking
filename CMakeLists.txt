cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
find_package(cmake_modules REQUIRED)

#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
rosbuild_add_boost_directories()

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_add_library(tracking src/tracking.cpp src/algorithms.cpp)
#set_target_properties(tracking PROPERTIES COMPILE_DEFINITIONS "__TL_VERBOSE")

#                      EXECUTABLES
rosbuild_add_executable(estimate src/estimator.cpp)
target_link_libraries(estimate tracking)
rosbuild_link_boost(estimate program_options)

#rosbuild_add_executable(estimate_server src/estimator_server.cpp)
#target_link_libraries(estimate_server tracking)

#rosbuild_add_executable(onestep src/onestep.cpp)
#target_link_libraries(onestep tracking)

#rosbuild_add_executable(multistep src/multistep.cpp)
#target_link_libraries(multistep tracking)


#rosbuild_add_executable(fakerend src/faker_rend.cpp)
#rosbuild_add_executable(fakeout src/faker_out.cpp)


#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/icra2013)
rosbuild_add_executable(twostep_one src/twostep.cpp)
target_link_libraries(twostep_one tracking)
#set_target_properties(A PROPERTIES COMPILE_FLAGS "-DA")
rosbuild_add_executable(twostep_other src/twostep.cpp)
target_link_libraries(twostep_other tracking)
set_target_properties(twostep_other PROPERTIES COMPILE_DEFINITIONS "MAKE_RECIP")

rosbuild_add_executable(iwls_prior src/iwls_prior.cpp)
target_link_libraries(iwls_prior tracking)

#rosbuild_add_executable(icra2013 src/icratest.cpp)
#target_link_libraries(icra2013 tracking)
#rosbuild_add_executable(icra2013A src/icra_Atest.cpp)
#target_link_libraries(icra2013A tracking)
#rosbuild_add_executable(icra2013sim src/icra_sims.cpp)
#target_link_libraries(icra2013sim tracking)

#                      EXECUTABLES         OUT OF DATE
##rosbuild_add_executable(cautious_greedy_server src/server.cpp)
##rosbuild_add_executable(mutl src/newserver.cpp)
##rosbuild_add_executable(client src/client.cpp)
##rosbuild_add_executable(init_server src/init.cpp)
#

#   TESTS FOR THE ABOVE EXECUTABLES AND LIBRARIES
#rosbuild_add_executable(twostep_test src/twosteptest.cpp)
#target_link_libraries(twostep_test tracking)
#set_target_properties(twostep_test PROPERTIES EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/tests)

#rosbuild_add_executable(rosiwlstest src/rosiwlstest.cpp)
#target_link_libraries(rosiwlstest tracking)

#rosbuild_add_executable(ellipsetest src/ellipsetest.cpp)
#target_link_libraries(ellipsetest tracking)

#rosbuild_add_executable(ost src/onesteptest.cpp)
#target_link_libraries(ost tracking)

#BROKEN
#rosbuild_add_executable(rosiwlstest src/rosiwlstest.cpp )
#target_link_libraries(rosiwlstest tracking)

#rosbuild_add_executable(lit src/lineinttest.cpp )
#target_link_libraries(lit tracking)

#rosbuild_add_executable(cct src/circumcircletest.cpp )
#target_link_libraries(cct tracking)

#rosbuild_add_executable(batchtest src/batchtest.cpp)
#target_link_libraries(batchtest tracking)

#rosbuild_add_executable(ekftest src/ekftest.cpp )
#target_link_libraries(ekftest tracking)

#rosbuild_add_executable(iekftest src/iekftest.cpp )
#target_link_libraries(iekftest tracking)

#MAYBE BROKEN
#rosbuild_add_executable(iwlstest src/iwlstest.cpp )
#target_link_libraries(iwlstest tracking)

#MIGHT ALSO BE BROKEN
#rosbuild_add_executable(iwlstestprior src/iwlstest_prior.cpp )
#target_link_libraries(iwlstestprior tracking)

##             OTHER COMMANDS
#common commands for building c++ executables and libraries
#target_link_libraries(cct tracking)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})
