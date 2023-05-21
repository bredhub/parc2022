execute_process(COMMAND "/home/bredhub/catkin_ws/build/PARC-Engineers-League/parc_robot/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/bredhub/catkin_ws/build/PARC-Engineers-League/parc_robot/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
