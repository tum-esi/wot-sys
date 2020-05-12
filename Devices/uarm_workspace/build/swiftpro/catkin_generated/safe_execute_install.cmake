execute_process(COMMAND "/home/bowen/uarm_workspace/build/swiftpro/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/bowen/uarm_workspace/build/swiftpro/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
