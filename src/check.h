#ifndef __EUROBIN_IOT_CHECK_H
#define __EUROBIN_IOT_CHECK_H

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define RCCHECK(fn)                                                                             \
	{                                                                                           \
		rcl_ret_t temp_rc = fn;                                                                 \
		if ((temp_rc != RCL_RET_OK))                                                            \
		{                                                                                       \
			printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);        \
			vTaskDelete(NULL);                                                                  \
		}                                                                                       \
	}
#define RCSOFTCHECK(fn)                                                                           \
	{                                                                                             \
		rcl_ret_t temp_rc = fn;                                                                   \
		if ((temp_rc != RCL_RET_OK))                                                              \
		{                                                                                         \
			printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);        \
		}                                                                                         \
	}


#endif