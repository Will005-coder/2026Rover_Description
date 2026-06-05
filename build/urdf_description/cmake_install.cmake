<<<<<<< HEAD
# Install script for directory: /home/berenakpinar/ws_moveit2/src/urdf_description

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/berenakpinar/ws_moveit2/src/urdf_description/install/urdf_description")
=======
# Install script for directory: /home/berenakpinar/Downloads/2026Rover_description

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/berenakpinar/Downloads/2026Rover_description/install/urdf_description")
>>>>>>> caddd7ff5a319ac415dbe87685f8ec4c434882f2
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description" TYPE DIRECTORY FILES
<<<<<<< HEAD
    "/home/berenakpinar/ws_moveit2/src/urdf_description/launch"
    "/home/berenakpinar/ws_moveit2/src/urdf_description/config"
    "/home/berenakpinar/ws_moveit2/src/urdf_description/urdf"
    "/home/berenakpinar/ws_moveit2/src/urdf_description/meshes"
    "/home/berenakpinar/ws_moveit2/src/urdf_description/rviz"
=======
    "/home/berenakpinar/Downloads/2026Rover_description/launch"
    "/home/berenakpinar/Downloads/2026Rover_description/config"
    "/home/berenakpinar/Downloads/2026Rover_description/urdf"
    "/home/berenakpinar/Downloads/2026Rover_description/meshes"
    "/home/berenakpinar/Downloads/2026Rover_description/rviz"
>>>>>>> caddd7ff5a319ac415dbe87685f8ec4c434882f2
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/berenakpinar/ws_moveit2/src/urdf_description/build/urdf_description/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/urdf_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/berenakpinar/ws_moveit2/src/urdf_description/build/urdf_description/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/urdf_description")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/berenakpinar/Downloads/2026Rover_description/build/urdf_description/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/urdf_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/berenakpinar/Downloads/2026Rover_description/build/urdf_description/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/urdf_description")
>>>>>>> caddd7ff5a319ac415dbe87685f8ec4c434882f2
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description/environment" TYPE FILE FILES "/home/berenakpinar/ws_moveit2/src/urdf_description/build/urdf_description/ament_cmake_environment_hooks/ament_prefix_path.dsv")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description/environment" TYPE FILE FILES "/home/berenakpinar/Downloads/2026Rover_description/build/urdf_description/ament_cmake_environment_hooks/ament_prefix_path.dsv")
>>>>>>> caddd7ff5a319ac415dbe87685f8ec4c434882f2
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description/environment" TYPE FILE FILES "/home/berenakpinar/ws_moveit2/src/urdf_description/build/urdf_description/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description" TYPE FILE FILES "/home/berenakpinar/ws_moveit2/src/urdf_description/build/urdf_description/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description" TYPE FILE FILES "/home/berenakpinar/ws_moveit2/src/urdf_description/build/urdf_description/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description" TYPE FILE FILES "/home/berenakpinar/ws_moveit2/src/urdf_description/build/urdf_description/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description" TYPE FILE FILES "/home/berenakpinar/ws_moveit2/src/urdf_description/build/urdf_description/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description" TYPE FILE FILES "/home/berenakpinar/ws_moveit2/src/urdf_description/build/urdf_description/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/berenakpinar/ws_moveit2/src/urdf_description/build/urdf_description/ament_cmake_index/share/ament_index/resource_index/packages/urdf_description")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description/environment" TYPE FILE FILES "/home/berenakpinar/Downloads/2026Rover_description/build/urdf_description/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description" TYPE FILE FILES "/home/berenakpinar/Downloads/2026Rover_description/build/urdf_description/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description" TYPE FILE FILES "/home/berenakpinar/Downloads/2026Rover_description/build/urdf_description/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description" TYPE FILE FILES "/home/berenakpinar/Downloads/2026Rover_description/build/urdf_description/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description" TYPE FILE FILES "/home/berenakpinar/Downloads/2026Rover_description/build/urdf_description/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description" TYPE FILE FILES "/home/berenakpinar/Downloads/2026Rover_description/build/urdf_description/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/berenakpinar/Downloads/2026Rover_description/build/urdf_description/ament_cmake_index/share/ament_index/resource_index/packages/urdf_description")
>>>>>>> caddd7ff5a319ac415dbe87685f8ec4c434882f2
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description/cmake" TYPE FILE FILES
<<<<<<< HEAD
    "/home/berenakpinar/ws_moveit2/src/urdf_description/build/urdf_description/ament_cmake_core/urdf_descriptionConfig.cmake"
    "/home/berenakpinar/ws_moveit2/src/urdf_description/build/urdf_description/ament_cmake_core/urdf_descriptionConfig-version.cmake"
=======
    "/home/berenakpinar/Downloads/2026Rover_description/build/urdf_description/ament_cmake_core/urdf_descriptionConfig.cmake"
    "/home/berenakpinar/Downloads/2026Rover_description/build/urdf_description/ament_cmake_core/urdf_descriptionConfig-version.cmake"
>>>>>>> caddd7ff5a319ac415dbe87685f8ec4c434882f2
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description" TYPE FILE FILES "/home/berenakpinar/ws_moveit2/src/urdf_description/package.xml")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/urdf_description" TYPE FILE FILES "/home/berenakpinar/Downloads/2026Rover_description/package.xml")
>>>>>>> caddd7ff5a319ac415dbe87685f8ec4c434882f2
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
<<<<<<< HEAD
file(WRITE "/home/berenakpinar/ws_moveit2/src/urdf_description/build/urdf_description/${CMAKE_INSTALL_MANIFEST}"
=======
file(WRITE "/home/berenakpinar/Downloads/2026Rover_description/build/urdf_description/${CMAKE_INSTALL_MANIFEST}"
>>>>>>> caddd7ff5a319ac415dbe87685f8ec4c434882f2
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
