#ifndef __EUROBIN_IOT_HALL_H
#define __EUROBIN_IOT_HALL_H

#include "check.h"

namespace eurobin_iot {
    namespace hall {
        static const uint8_t pin = 33;

        void createPublisher(String name_node ,rcl_publisher_t &pub, rcl_node_t &node) {
            String hall_topic_name = name_node + "/hall";
            RCCHECK(rclc_publisher_init_default(
				&pub,
				&node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
				hall_topic_name.c_str()));
        }
        
        void printLCD(std_msgs__msg__Int32 msg) {
            M5.Lcd.printf("Hall: %d\n", msg.data);
        }

        void publish(std_msgs__msg__Int32 msg, rcl_publisher_t pub) {
            if (digitalRead(pin))
				msg.data = 0;
			else
				msg.data = 1;
			printLCD(msg);
			RCSOFTCHECK(rcl_publish(&pub, &msg, NULL));
        }

        void setPin() {
            pinMode(pin, INPUT);
        }
    }
}

#endif