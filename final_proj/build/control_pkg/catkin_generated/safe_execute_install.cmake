execute_process(COMMAND "/home/cc/ee106b/sp23/class/ee106b-abv/ros_workspace/final_proj/build/control_pkg/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/cc/ee106b/sp23/class/ee106b-abv/ros_workspace/final_proj/build/control_pkg/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
