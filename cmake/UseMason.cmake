if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.mason/mason.cmake")
  #COMMAND git submodule update --init .mason WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  execute_process(
    COMMAND git clone -b master --single-branch https://github.com/mapbox/mason.git ${CMAKE_CURRENT_SOURCE_DIR}/.mason
 )
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/.mason/mason.cmake)

message(STATUS "adding 'mason_packages' to CMAKE_MODULE_PATH")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/")

#set(Boost_DEBUG ON)
set(Boost_NO_BOOST_CMAKE TRUE)
set(Boost_NO_SYSTEM_PATHS TRUE)
set(BOOST_INCLUDEDIR ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/headers/boost/1.61.0/include)

set(BOOST_LIBRARYDIR ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_manual_copy/1.61.0/lib)
#set(BOOST_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64)
#set(BOOST_LIBRARYDIR ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_libdate_time/1.61.0/lib)
#set(BOOST_LIBRARYDIR ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_libfilesystem/1.61.0/lib)
#set(BOOST_LIBRARYDIR ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_libiostreams/1.61.0/lib)
#set(BOOST_LIBRARYDIR ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_libprogram_options/1.61.0/lib)
#set(BOOST_LIBRARYDIR ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_libregex/1.61.0/lib)
#set(BOOST_LIBRARYDIR ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_libsystem/1.61.0/lib)
#set(BOOST_LIBRARYDIR ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_libthread/1.61.0/lib)

#mason_use(boost_liball_osrm VERSION 1.59.0)
#mason_use(boost_liball VERSION 1.59.0)
#mason_use(boost VERSION 1.59.0)
#mason_use(boost_libeverything VERSION 1.59.0)


mason_use(boost VERSION 1.61.0 HEADER_ONLY)
mason_use(boost_libdate_time VERSION 1.61.0)
mason_use(boost_libfilesystem VERSION 1.61.0)
mason_use(boost_libiostreams VERSION 1.61.0)
mason_use(boost_libprogram_options VERSION 1.61.0)
mason_use(boost_libregex VERSION 1.61.0)
mason_use(boost_libsystem VERSION 1.61.0)
mason_use(boost_libthread VERSION 1.61.0)
mason_use(boost_libtest VERSION 1.61.0)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_libdate_time/1.61.0/lib/libboost_date_time.a DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_manual_copy/1.61.0/lib/)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_libfilesystem/1.61.0/lib/libboost_filesystem.a DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_manual_copy/1.61.0/lib/)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_libiostreams/1.61.0/lib/libboost_iostreams.a DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_manual_copy/1.61.0/lib/)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_libprogram_options/1.61.0/lib/libboost_program_options.a DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_manual_copy/1.61.0/lib/)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_libregex/1.61.0/lib/libboost_regex.a DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_manual_copy/1.61.0/lib/)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_libsystem/1.61.0/lib/libboost_system.a DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_manual_copy/1.61.0/lib/)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_libthread/1.61.0/lib/libboost_thread.a DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_manual_copy/1.61.0/lib/)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_libtest/1.61.0/lib/libboost_unit_test_framework.a DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/mason_packages/linux-x86_64/boost_manual_copy/1.61.0/lib/)

message(STATUS "UseMason.cmake finished")
