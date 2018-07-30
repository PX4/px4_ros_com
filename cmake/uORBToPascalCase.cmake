# Run a script to convert the uORB msg definitions files to PascalCase
# so to be processed by generate_messages() on catkin

set(ROS_UORB_MSGS_LIST "")

function(glob_generate target file_glob)
    file(GLOB uorb_glob_files ${file_glob})
    set(UORB_MSGS_LIST)
    set(PX4_ROS_COM_MSG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg/")
    file(MAKE_DIRECTORY ${PX4_ROS_COM_MSG_DIR})
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/uORB2ROSMsgs.py ${PROJECT_NAME} ${PX4_FIRMWARE_MSG_DIR} ${PX4_ROS_COM_MSG_DIR}
	WORKING_DIRECTORY ${PX4_FIRMWARE_REPO_DIR}
	OUTPUT_QUIET
	)
    file(GLOB ros_glob_files ${PX4_ROS_COM_MSG_DIR}*.msg)
    message(STATUS "Added the following msgs to ROS2 msg generation")
    foreach(ros_glob_file ${ros_glob_files})
        get_filename_component(filename ${ros_glob_file} NAME)
    	list(APPEND ROS_UORB_MSGS_LIST "msg/${filename}")
	message(STATUS "\t${filename}")
    endforeach()
    set(ROS_UORB_MSGS "${ROS_UORB_MSGS_LIST}" CACHE INTERNAL "ROS_UORB_MSGS")
endfunction()

glob_generate(models_gen ${PX4_FIRMWARE_MSG_DIR}*.msg)
