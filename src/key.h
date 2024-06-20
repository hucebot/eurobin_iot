#ifndef __EUROBIN_IOT_KEY_H
#define __EUROBIN_IOT_KEY_H

#include "check.h"

namespace eurobin_iot {
    namespace key {
        static const uint8_t pin = 33;
		static const uint8_t led_pin = 32;
		CRGB leds[1];

        void setPin() {
            pinMode(key::pin, INPUT_PULLUP);
            FastLED.addLeds<SK6812, led_pin, GRB>(key::leds, 1);
            key::leds[0] = CRGB::Blue;
            FastLED.setBrightness(0);
        }

        void createPublisher(String name_node, rcl_publisher_t &pub, rcl_node_t &node) {
            String key_topic_name = name_node + "/key";
            RCCHECK(rclc_publisher_init_default(
				&pub,
				&node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
				key_topic_name.c_str()));
        }

        void publish(std_msgs__msg__Int32 msg, rcl_publisher_t pub) {
            if (!digitalRead(pin))
			{
				leds[0] = CRGB::Blue;
				M5.Lcd.println(("Key: 1       "));
				FastLED.setBrightness(255);
				FastLED.show();
				msg.data = 1;
				eurobin_iot::sound::doorbell();
			}
			else
			{
				M5.Lcd.println(("Key: 0      "));
				leds[0] = CRGB::Red;
				FastLED.setBrightness(255);
				FastLED.show();
				msg.data = 0;
			}
			RCSOFTCHECK(rcl_publish(&pub, &msg, NULL));
        }
    }
}








#endif