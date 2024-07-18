#ifndef __EUROBIN_IOT_TOF_H
#define __EUROBIN_IOT_TOF_H
#include "i2c.h"
#include "check.h"


namespace eurobin_iot {
    namespace tof { // time_of_flight
        // Time of flight sensor
        bool ok = false;
        static const uint8_t VL53L0X_REG_SYSRANGE_START = 0x00;
        static const uint8_t VL53L0X_REG_RESULT_RANGE_STATUS= 0x14;
        static const uint8_t TOF_I2C_ADDRESS = 0x29; //I2C address of ToF
        uint16_t ambient_count, signal_count, dist;

        inline uint8_t check() {
            uint8_t error = i2c::test_address(TOF_I2C_ADDRESS);
            ok = (error == 0) ? true : false;
            return error;
        }

        void read(uint16_t* ambient_count, uint16_t *signal_count, uint16_t* dist) {
            i2c::write_byte_data_at(TOF_I2C_ADDRESS, VL53L0X_REG_SYSRANGE_START, 0x01);

            byte val = 0;
            int cnt = 0;
            while (cnt < 100) {  // 1 second waiting time max
                delay(10);
                val = i2c::read_byte_data_at(TOF_I2C_ADDRESS, VL53L0X_REG_RESULT_RANGE_STATUS);
                if (val & 0x01) break;
                cnt++;
            }
            
            uint8_t* data = i2c::read_block_data_at(TOF_I2C_ADDRESS, 0x14, 12);
            *ambient_count = bytes::makeuint16(data[7], data[6]);
            *signal_count = bytes::makeuint16(data[9], data[8]);
            *dist = bytes::makeuint16(data[11], data[10]);
        }

        void createPublisher(std_msgs__msg__Int16MultiArray &msg, int16_t *data, String name_node, rcl_publisher_t &pub, rcl_node_t &node) {
            String tof_topic_name = name_node + "/tof";
		    msg.data.capacity = 3;
		    msg.data.size = 3;
		    msg.data.data = data;
            if (tof::ok)
            {
                RCCHECK(rclc_publisher_init_default(
                    &pub,
                    &node,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
                    tof_topic_name.c_str()));
            }
        }

        /* void printDistanceLCD(uint16_t dist) {
            M5.Lcd.printf("Dist.: %d mm         \n", dist);
        } */

        void publish(std_msgs__msg__Int16MultiArray msg, rcl_publisher_t pub) {
            read(&ambient_count, &signal_count, &dist);
            msg.data.data[0] = dist;
			msg.data.data[1] = ambient_count;
			msg.data.data[2] = signal_count;
			RCSOFTCHECK(rcl_publish(&pub, &msg, NULL));
            //printDistanceLCD(dist);
        }

        void initializing() {
            Serial.println("Initializing I2C...");
            Serial.print("Time of flight: ");
            uint8_t error = check();
            if (ok)
                Serial.println("ok");
            else
            {
                Serial.print("Error Initializing I2C Tof");
                Serial.println(error);
            }
        }

}}// namespace
#endif