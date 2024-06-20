#include <Arduino.h>
#ifdef M5CORE2
#include <M5Core2.h>
#endif
#ifndef M5CORE2
#include "M5Atom.h"
#endif
#include <WiFi.h>
#include <WiFiMulti.h>
#include <micro_ros_platformio.h>

#include <Preferences.h> // this the EEPROM/flash interface

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/string.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <FastLED.h>

#include "bytes.h"
#include "i2c.h"
#include "tof.h"
#include "sound.h"
#include "scale.h"

using namespace eurobin_iot;

namespace eurobin_iot
{
	uint8_t mode = 0;
	uint8_t init_mode = 0;
	bool wifi = false;

	namespace modes
	{
		enum
		{
			NONE = 0,
			KEY,
			TOF,
			HALL,
			SCALE,
			SIZE // number of modes
		};
	}
	class Node
	{
	public:
		Node() {}
		void init();
		void loop();

	protected:
		// Arduino-style objects
		Speaker speaker;
		Preferences prefs;
		WiFiMulti wifiMulti;

		// variables
		int butt_c_activated = 0;
		int butt_mode_activated = 0;
		int16_t data_tof[3]; // signed because no default message for unsigned...
		char16_t opened[20];

		// micro-ros stuffs
		rcl_node_t node;
		rcl_allocator_t allocator = rcl_get_default_allocator();
		rclc_support_t support;
		rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
		rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
		struct micro_ros_agent_locator locator;

		// msgs and topics
		rcl_publisher_t pub_button;
		std_msgs__msg__Int32 msg_button;
		rcl_publisher_t pub_tof;
		std_msgs__msg__Int16MultiArray msg_tof;
		rcl_publisher_t pub_scale;
		std_msgs__msg__Int32 msg_scale;
		rcl_publisher_t pub_key;
		std_msgs__msg__Int32 msg_key;
		rcl_publisher_t pub_hall;
		std_msgs__msg__Int32 msg_hall;
	};
	Node node;
}

const char *get_mode(uint8_t mode)
{
	switch (mode)
	{
	case 0:
		return "undefined";
	case 1:
		return "key";
	case 2:
		return "ToF";
	case 3:
		return "Hall";
	case 4:
		return "Scale/force";
	default:
		return "error";
	}
}

void setPinMode() {
        switch (eurobin_iot::mode)
        {
        case eurobin_iot::modes::KEY:
            {
            key::setPin();
            break;
            }
        case eurobin_iot::modes::HALL:
            {
            hall::setPin();
            break;
            }
        case eurobin_iot::modes::SCALE:
            {
            scale::init();
            break;
            }
        case eurobin_iot::modes::TOF:
            {
            tof::initializing();
            break;
            }
        default:
            {
            printf("error");
            break;
            }
        }
    }


namespace eurobin_iot
{
	void Node::init()
	{
		prefs.begin("eurobin_iot");
		eurobin_iot::mode = prefs.getUInt("mode", 0);
		eurobin_iot::init_mode = eurobin_iot::mode;

		Serial.printf("ROS_IOT -> MODE: %d %s\n", eurobin_iot::mode, get_mode(eurobin_iot::mode));

		if (eurobin_iot::mode == eurobin_iot::modes::TOF || eurobin_iot::mode == eurobin_iot::modes::SCALE)
			Wire.begin(); // join i2c bus (address optional for master)

		#ifdef M5CORE2
		M5.begin();
		// LCD
		M5.Lcd.fillScreen(BLACK); // Set the screen
		M5.Lcd.setCursor(0, 0);
		M5.Lcd.setTextSize(2);
		M5.Lcd.setTextColor(BLUE);
		M5.Lcd.printf("Eurobin IOT ROS2\n");
		M5.Lcd.printf("SSID: %s\n", config::wifi::essid);
		M5.Lcd.setTextColor(WHITE);
		#endif
		
		setPinMode();

		#ifdef M5CORE2
		speaker.begin();
		speaker.InitI2SSpeakOrMic(MODE_SPK);
		#endif


		printf("starting Wifi...\n");
		// Adding Wifi
		// IPAddress agent_ip(192, 168, 100, 2); // should be deduced by DHCP?
		while (wifiMulti.run() != WL_CONNECTED)
		{
			delay(500);
			printf("Waiting for wifi...\n");
		}

		M5.Lcd.setTextColor(GREEN, BLACK);
		M5.lcd.print("RSSI: ");
		M5.lcd.println(WiFi.RSSI());
		M5.lcd.print("IP address: ");
		M5.lcd.println(WiFi.localIP());
		printf("Wifi OK, %s\n", WiFi.SSID());
		printf("Agent: %d.%d.%d.%d:%d\n", config::agent::ip[0], config::agent::ip[1], config::agent::ip[2], config::agent::ip[3], config::agent::port);
		IPAddress agent_ip(config::agent::ip[0], config::agent::ip[1], config::agent::ip[2], config::agent::ip[3]);
		locator.address = agent_ip;
		locator.port = config::agent::port;
		rmw_uros_set_custom_transport(
			false,
			(void *)&locator,
			platformio_transport_open,
			platformio_transport_close,
			platformio_transport_write,
			platformio_transport_read);


		RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

		prefs.begin("eurobin_iot");
		int id = prefs.getUInt("id", 0);
		printf("My ID is: %d\n", id);
		String node_name = String("eurobin_iot_") + String(id);
		printf("ROS 2 Topic prefix: %s\n", node_name.c_str());
		RCCHECK(rclc_node_init_default(&node, node_name.c_str(), "", &support));

		// create publishers
		/// touch button (left)
		String button_topic_name = node_name + "/button_a";
		RCCHECK(rclc_publisher_init_default(
			&pub_button,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			button_topic_name.c_str()));

		/// Time of flight
		tof::createPublisher(msg_tof, data_tof, node_name, pub_tof, node);

		// scale
		if (eurobin_iot::init_mode == eurobin_iot::modes::SCALE)
		{
			scale::createPublisher(node_name, pub_scale, node);
		}

		// key
		//String key_topic_name = node_name + "/key";
		if (eurobin_iot::init_mode == eurobin_iot::modes::KEY)
		{
			key::createPublisher(node_name, pub_key, node);
		}
		// hall sensor
		//String hall_topic_name = node_name + "/hall";
		if (eurobin_iot::init_mode == eurobin_iot::modes::HALL)
		{
			hall::createPublisher(node_name, pub_hall, node);
		}

  		RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

		#ifdef M5CORE2
		// show the ID
		M5.Lcd.fillRoundRect(320 - 100, 240 - 70, 100, 70, 15, TFT_GREEN);
		M5.Lcd.setCursor(320 - 100 + 5, 240 - 58);
		M5.Lcd.setTextSize(14);
		M5.Lcd.setTextColor(TFT_WHITE);
		M5.Lcd.printf("%d", id);

		// mode
		M5.Lcd.fillRoundRect(140, 240 - 25, 60, 25, 10, TFT_GREEN);
		M5.Lcd.setCursor(150, 240 - 20);
		M5.Lcd.setTextSize(2);
		M5.Lcd.printf("mode");
		#endif

		// M5.Lcd.setTextColor(GREEN, BLACK);
		// M5.Lcd.printf("ROS2 Node ready\n");
		//eurobin_iot::sound::ding(); // say we are ready!
	}

	void Node::loop()
	{
		M5.update();
		M5.Lcd.setTextSize(2);
		usleep(100000);

		if (eurobin_iot::mode != eurobin_iot::init_mode)
		{
			M5.Lcd.setCursor(0, 240 - 20);
			M5.Lcd.printf("-> RESET\n");
		}

		M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
		M5.Lcd.setCursor(0, 90);

		M5.Lcd.printf("Mode:%s          \n", get_mode(eurobin_iot::mode));

		// button
		M5.Lcd.printf("Buttons: %d %d %d      \n", M5.BtnA.read(), M5.BtnB.read(), M5.BtnC.read());
		msg_button.data = M5.BtnA.read();
		RCSOFTCHECK(rcl_publish(&pub_button, &msg_button, NULL));
		if (msg_button.data == 1)
			eurobin_iot::sound::ding();

		// time-of-flight
		if (tof::ok)
		{
			tof::publish(msg_tof, pub_tof);
		}

		// scale
		if (eurobin_iot::init_mode == eurobin_iot::modes::SCALE)
		{
			scale::publish(msg_scale, pub_scale);
		}

		// red key
		if (eurobin_iot::init_mode == eurobin_iot::modes::KEY)
		{
			key::publish(msg_key, pub_key);
		}

		// hall sensor
		if (eurobin_iot::init_mode == eurobin_iot::modes::HALL)
		{
			hall::publish(msg_hall, pub_hall);
		}

		// mode
		if (M5.BtnB.read())
			butt_mode_activated++;
		if (butt_mode_activated > 5)
		{
			eurobin_iot::mode = (eurobin_iot::mode + 1) % eurobin_iot::modes::SIZE;
			prefs.putUInt("mode", eurobin_iot::mode);
			butt_mode_activated = 0;
		}

		if (M5.BtnC.read())
		{
			butt_c_activated++;
			M5.Lcd.printf("RESET ID [50] %d", butt_c_activated);
		}
		else
			butt_c_activated = 0;

		if (butt_c_activated >= 50)
		{
			M5.Lcd.printf(" => RESET ID");
			int r = (int)random(100);
			Serial.print("NEW ID:");
			prefs.putUInt("id", r);
			// preferences.end();
			usleep(1000);
			ESP.restart();
		}

		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100000);
	}
}


void setup()
{
	eurobin_iot::node.init();
}

void loop()
{
	eurobin_iot::node.loop();
}
