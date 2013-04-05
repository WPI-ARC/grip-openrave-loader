
cmake_minimum_required (VERSION 2.6)
project (executeFromFileTab)

message (STATUS "Creating Makefile for executeFromFileTab")

IF(NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo Profile."
       FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

if(WIN32)
  set (CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
  message (STATUS "Setup Visual Studio Specific Flags")
  set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /GL /Gy /W1 /EHsc")
  set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc")
  set (CMAKE_DEBUG_POSTFIX "d")
  set (CMAKE_EXE_LINKER_FLAGS "/LTCG")
  set (CMAKE_MODULE_LINKER_FLAGS "/LTCG")
else ()
  set(CMAKE_INSTALL_PREFIX /usr )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -std=c++0x")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/grip")

find_package(DARTExt REQUIRED)
find_package(DART REQUIRED)
find_package(GRIP REQUIRED)

message("CMAKE_DEBUG_POSTFIX : " ${CMAKE_DEBUG_POSTFIX})

include_directories(${DARTExt_INCLUDEDIR})
include_directories(${DART_INCLUDEDIR})
include_directories(${GRIP_INCLUDEDIR})
link_directories(${DART_LIBRARY_DIRS})
link_directories(${GRIP_LIBRARY_DIRS})

message("DART_LIBRARIES : " ${DART_LIBRARIES})
message("GRIP_LIBRARY_DIRS : " ${GRIP_LIBRARIES})

FIND_PACKAGE(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})
#SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR})
#link_directories(${LIBXML2_LIBRARY})

message("LIBXML2_INCLUDE_DIR : " ${LIBXML2_INCLUDE_DIR})
message("LIBXML2_LIBRARY_DIRS : " ${LIBXML2_LIBRARY_DIRS})
message("LIBXML2_LIBRARIES : " ${LIBXML2_LIBRARIES})

# Search all header and source files
set ( executeFromFileTab_src executeFromFileTab.cpp executeFromFileTabApp.cpp Controller.cpp Grasper.cpp JointMover.cpp)
set ( executeFromFileTab_hdr executeFromFileTab.h Controller.h Grasper.h JointMover.h)

if(WIN32)
  set (wxWidgets_EXCLUDE_COMMON_LIBRARIES TRUE)
  set (wxWidgets_USE_LIBS base core gl png tiff jpeg zlib)
else(WIN32)
  set (wxWidgets_USE_LIBS base core gl)
endif(WIN32)

find_package (wxWidgets)
if (wxWidgets_FOUND)
  include (${wxWidgets_USE_FILE})
  include_directories(${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${WXWIDGETS_SHARED_LIBRARY_DIRS})

  if(MSVC)
    add_executable (executeFromFileTab WIN32 ${executeFromFileTab_src} ${executeFromFileTab_hdr})
    set_target_properties (executeFromFileTab PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
  else()
    add_executable (executeFromFileTab ${executeFromFileTab_src} ${executeFromFileTab_hdr})
  endif()

  target_link_libraries (executeFromFileTab ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES}  ${LIBXML2_LIBRARIES} ${Boost_LIBRARIES})

else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)



