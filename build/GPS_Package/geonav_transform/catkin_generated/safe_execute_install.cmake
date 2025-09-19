execute_process(COMMAND "/home/yj/knu_roscore_ws/build/GPS_Package/geonav_transform/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/yj/knu_roscore_ws/build/GPS_Package/geonav_transform/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
