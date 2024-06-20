#ifndef __EUROBIN_IOT_SCALE_H
#define __EUROBIN_IOT_SCALE_H

#include "M5_Scales.h"
#include "check.h"

namespace eurobin_iot
{
    // https://docs.m5stack.com/en/unit/scales

    M5_Scales loadcell;

    namespace scale
    {
        void print()
        {
            int weight = loadcell.getWeight();
            Serial.print("Weight (g) ");
            Serial.println(weight);
        }
        int weight() {
            return loadcell.getWeight();
        }
        void tare() {
            loadcell.setOffset();
        }
        bool button() {
            return loadcell.getBtnStatus();
        }
        void init()
        {
            Serial.println("Initializing the scale... (I2C) ");
            if (!loadcell.begin(&Wire, 32, 33, M5_SCALES_DEFAULT_ADDR))
            {
                Serial.println("scales connect error");
                return;
            }
            bool ret = true;
            ret &= loadcell.setLEDSyncWeight(true);
            ret &= loadcell.setBtnOffsetControl(true);
            ret &= loadcell.setLEDColor(0x0000ff);
            if (ret)
                Serial.println("OK");
            else
                Serial.println("INIT ERROR");
            loadcell.setBtnOffsetControl(false);
            loadcell.setOffset();
            print();
        }

        void createPublisher(String name_node ,rcl_publisher_t &pub, rcl_node_t &node) {
            String scale_topic_name = name_node + "/scale";
            RCCHECK(rclc_publisher_init_default(
				&pub,
				&node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
				scale_topic_name.c_str()));
        }

        void printDistanceLCD(int w) {
            M5.Lcd.printf("weight: %d grams     \n", w);
        }

        void publish(std_msgs__msg__Int32 msg, rcl_publisher_t pub) {
            int w = scale::weight();
			msg.data = w;
			printDistanceLCD(w);
			RCSOFTCHECK(rcl_publish(&pub, &msg, NULL));
			if (scale::button())
			{
				scale::tare();
				Serial.println("scale::tare");
			}
        }

    }
} // namespace
#endif