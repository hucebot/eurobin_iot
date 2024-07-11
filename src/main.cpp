#include <Arduino.h>
#include <M5Core2.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <micro_ros_platformio.h>
#include <chrono>
#include <ctime> 
#include "UNIT_UHF_RFID.h"

#include <Preferences.h> // this the EEPROM/flash interface

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/string.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <FastLED.h>
#include "time.h"
#include "VL53L1X.h"

#include "bytes.h"
#include "i2c.h"
#include "tof.h"
#include "sound.h"
#include "scale.h"

#include "config.h"

using namespace eurobin_iot;

namespace eurobin_iot
{
	uint8_t mode = 0;
	uint8_t init_mode = 0;
	bool wifi = false;
	VL53L1X tof_sensor;

	namespace modes
	{
		enum
		{
			NONE = 0,
			KEY,
			TOFM2,
			TOFM4,
			HALL,
			SCALE,
			RFID,
			TIMER,
			SIZE // number of modes
		};
	}
	namespace key
	{
		static const uint8_t pin = 33;
		static const uint8_t led_pin = 32;
		CRGB leds[1];
	}
	namespace hall
	{
		static const uint8_t pin = 33;
	}
	namespace rfid
	{
		static const uint8_t rx = 33;
		static const uint8_t tx = 32;
		bool ok=false;
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
		Unit_UHF_RFID uhf;

		// variables
		int butt_c_activated = 0;
		int butt_mode_activated = 0;
		int16_t data_tof[3]; // signed because no default message for unsigned...
		char msg_buffer[25];

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
		rcl_publisher_t pub_tofm2;
		std_msgs__msg__Int16MultiArray msg_tofm2;
		rcl_publisher_t pub_tofm4;
		std_msgs__msg__Int16 msg_tofm4;
		rcl_publisher_t pub_scale;
		std_msgs__msg__Int32 msg_scale;
		rcl_publisher_t pub_key;
		std_msgs__msg__Int32 msg_key;
		rcl_publisher_t pub_hall;
		std_msgs__msg__Int32 msg_hall;
		rcl_publisher_t pub_hfid;
		std_msgs__msg__String msg_hfid;
		std_msgs__msg__Bool msg_time;
		rcl_publisher_t pub_time;
	};
	Node node;
}

#define RCCHECK(fn)                                                                             \
	{                                                                                           \
		rcl_ret_t temp_rc = fn;                                                                 \
		if ((temp_rc != RCL_RET_OK))                                                            \
		{                                                                                       \
			printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);        \
			M5.Lcd.printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
			vTaskDelete(NULL);                                                                  \
		}                                                                                       \
	}
#define RCSOFTCHECK(fn)                                                                           \
	{                                                                                             \
		rcl_ret_t temp_rc = fn;                                                                   \
		if ((temp_rc != RCL_RET_OK))                                                              \
		{                                                                                         \
			printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);        \
			M5.Lcd.printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
		}                                                                                         \
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
		return "ToFM2";
	case 3:
		return "ToFM4";
	case 4:
		return "Hall";
	case 5:
		return "Scale/force";
	case 6:
		return "Rfid";
	case 7:
		return "Timer";
	default:
		return "error";
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

		if (eurobin_iot::mode == eurobin_iot::modes::TOFM2 ||
			eurobin_iot::mode == eurobin_iot::modes::TOFM4 || 
			eurobin_iot::mode == eurobin_iot::modes::SCALE || 
			eurobin_iot::mode == eurobin_iot::modes::TIMER)
			Wire.begin(); // join i2c bus (address optional for master)
		M5.begin();

		// LCD
		M5.Lcd.fillScreen(BLACK); // Set the screen
		M5.Lcd.setCursor(0, 0);
		M5.Lcd.setTextSize(2);
		M5.Lcd.setTextColor(BLUE);
		M5.Lcd.printf("Eurobin IOT ROS2\n");
		M5.Lcd.printf("SSID: %s\n", config::wifi::essid);
		M5.Lcd.setTextColor(WHITE);
		// check the time-of-flight
		if (eurobin_iot::mode == eurobin_iot::modes::TOFM2 ||
			eurobin_iot::mode == eurobin_iot::modes::TOFM4 || 
			eurobin_iot::mode == eurobin_iot::modes::TIMER)
		{
			Serial.println("Initializing I2C...");
			Serial.print("Time of flight: ");
			uint8_t error = tof::check();
			if (tof::ok) {
				if(eurobin_iot::mode != eurobin_iot::modes::TOFM2){
					tof_sensor.init();
					tof_sensor.setDistanceMode(VL53L1X::Short);
  					tof_sensor.setMeasurementTimingBudget(20000);
					tof_sensor.startContinuous(20);
				}
				Serial.println("ok");
			}
			else
			{
				Serial.print("error ");
				Serial.println(error);
			}
		}

		// setup the key button
		if (eurobin_iot::mode == eurobin_iot::modes::KEY)
		{
			pinMode(key::pin, INPUT_PULLUP);
			FastLED.addLeds<SK6812, key::led_pin, GRB>(key::leds, 1);
			key::leds[0] = CRGB::Blue;
			FastLED.setBrightness(0);
		}

		// setup the hall sensor button
		if (eurobin_iot::mode == eurobin_iot::modes::HALL)
		{
			pinMode(hall::pin, INPUT);
		}

		// scale
		if (eurobin_iot::mode == eurobin_iot::modes::SCALE)
		{
			scale::init();
		}

		//rfid
		if (eurobin_iot::mode == eurobin_iot::modes::RFID) {
			uhf.begin(&Serial2, 115200, eurobin_iot::rfid::rx, eurobin_iot::rfid::tx, false);
			String info = uhf.getVersion();
			if (info != "ERROR") {
				Serial.println(info);
				uhf.setTxPower(2600);
				rfid::ok = true;
			}
			else {
				Serial.println("Error connecting the RFID");
			}
			
		}

		speaker.begin();
		speaker.InitI2SSpeakOrMic(MODE_SPK);

		printf("starting Wifi...\n");
		// Adding Wifi
		// IPAddress agent_ip(192, 168, 100, 2); // should be deduced by DHCP?
		wifiMulti.addAP(config::wifi::essid, config::wifi::password);

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
		/* String button_topic_name = node_name + "/button_a";
		RCCHECK(rclc_publisher_init_default(
			&pub_button,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			button_topic_name.c_str())); */

		/// Time of flight M2
		if (tof::ok && eurobin_iot::init_mode == eurobin_iot::modes::TOFM2)
		{
			String tof_topic_name = node_name + "/tofm2";
			msg_tofm2.data.capacity = 3;
			msg_tofm2.data.size = 3;
			msg_tofm2.data.data = data_tof;
			RCCHECK(rclc_publisher_init_default(
				&pub_tofm2,
				&node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
				tof_topic_name.c_str()));
		}
		/// Time of flight M4
		if (tof::ok && eurobin_iot::init_mode == eurobin_iot::modes::TOFM4)
		{
			String tof_topic_name = node_name + "/tofm4";
			RCCHECK(rclc_publisher_init_default(
				&pub_tofm4,
				&node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
				tof_topic_name.c_str()));
		}


		// scale
		if (eurobin_iot::init_mode == eurobin_iot::modes::SCALE)
		{
			String scale_topic_name = node_name + "/scale";
			RCCHECK(rclc_publisher_init_default(
				&pub_scale,
				&node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
				scale_topic_name.c_str()));
		}

		// key
		if (eurobin_iot::init_mode == eurobin_iot::modes::KEY)
		{
			String key_topic_name = node_name + "/key";
			RCCHECK(rclc_publisher_init_default(
				&pub_key,
				&node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
				key_topic_name.c_str()));
		}
		// hall sensor
		if (eurobin_iot::init_mode == eurobin_iot::modes::HALL)
		{
			String hall_topic_name = node_name + "/hall";
			RCCHECK(rclc_publisher_init_default(
				&pub_hall,
				&node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
				hall_topic_name.c_str()));
		}

		// rfid sensor
		if (eurobin_iot::init_mode == eurobin_iot::modes::RFID)
		{
			String hfid_topic_name = node_name + "/rfid";
			msg_hfid.data.data = msg_buffer;
			msg_hfid.data.capacity = sizeof(msg_buffer);
			msg_hfid.data.size = 0;
			RCCHECK(rclc_publisher_init_default(
				&pub_hfid,
				&node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
				hfid_topic_name.c_str()));
		}

		//time
		if (eurobin_iot::init_mode == eurobin_iot::modes::TIMER)
		{
			String time_topic_name = node_name + "/time";
			RCCHECK(rclc_publisher_init_default(
				&pub_time,
				&node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
				time_topic_name.c_str()));
		}

  		RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

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

		// M5.Lcd.setTextColor(GREEN, BLACK);
		// M5.Lcd.printf("ROS2 Node ready\n");
		eurobin_iot::sound::ding(); // say we are ready!
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
		/* msg_button.data = M5.BtnA.read();
		RCSOFTCHECK(rcl_publish(&pub_button, &msg_button, NULL));
		if (msg_button.data == 1)
			eurobin_iot::sound::ding();
 */
		// time-of-flight M2
		if (tof::ok && eurobin_iot::init_mode == eurobin_iot::modes::TOFM2)
		{
			uint16_t ambient_count, signal_count, dist;
			tof::read(&ambient_count, &signal_count, &dist);
			//angle = (dist > 55) ? atan(dist/97.) * 180/PI: 0.; 
			//angle = atan(dist/155.) * 180/PI;
			// M5.Lcd.setCursor(0, 110);
			M5.Lcd.printf("Dist.: %d mm         \n", dist);
			//msg_tofm2.data.data[0] = tof_sensor.read();
			//msg_tofm2.data.data[1] = angle;
			msg_tofm2.data.data[0] = dist;
			msg_tofm2.data.data[1] = signal_count;
			msg_tofm2.data.data[2] = ambient_count;
			RCSOFTCHECK(rcl_publish(&pub_tofm2, &msg_tofm2, NULL));
		}

		// time-of-flight M4
		if (tof::ok && eurobin_iot::init_mode == eurobin_iot::modes::TOFM4 && tof::ok)
		{

			uint16_t dist = tof_sensor.read();
			M5.Lcd.printf("Dist.: %d mm         \n", dist);
			msg_tofm4.data = dist;
			RCSOFTCHECK(rcl_publish(&pub_tofm4, &msg_tofm4, NULL));
		}


		// scale
		if (eurobin_iot::init_mode == eurobin_iot::modes::SCALE && tof::ok)
		{
			// scale::print();
			int w = scale::weight();
			msg_scale.data = w;
			M5.Lcd.printf("weight: %d grams     \n", w);
			RCSOFTCHECK(rcl_publish(&pub_scale, &msg_scale, NULL));
			if (scale::button())
			{
				scale::tare();
				Serial.println("scale::tare");
			}
		}

		// red key
		if (eurobin_iot::init_mode == eurobin_iot::modes::KEY)
		{
			if (!digitalRead(key::pin))
			{
				key::leds[0] = CRGB::Blue;
				M5.Lcd.println(("Key: 1       "));
				FastLED.setBrightness(255);
				FastLED.show();
				msg_key.data = 1;
				eurobin_iot::sound::doorbell();
			}
			else
			{
				M5.Lcd.println(("Key: 0      "));
				key::leds[0] = CRGB::Red;
				FastLED.setBrightness(255);
				FastLED.show();
				msg_key.data = 0;
			}
			RCSOFTCHECK(rcl_publish(&pub_key, &msg_key, NULL));
		}

		// hall sensor
		if (eurobin_iot::init_mode == eurobin_iot::modes::HALL)
		{
			if (digitalRead(hall::pin))
				msg_hall.data = 0;
			else
				msg_hall.data = 1;
			M5.Lcd.printf("Hall: %d\n", msg_hall.data);
			RCSOFTCHECK(rcl_publish(&pub_hall, &msg_hall, NULL));
		}

		// rfid
		if (eurobin_iot::init_mode == eurobin_iot::modes::RFID && rfid::ok)
		{
			uint8_t result = uhf.pollingMultiple(10);
			if(result > 0) {
				for (uint8_t i = 0; i < result; i++) {
					Serial.println("epc: " + uhf.cards[i].epc_str);
					//M5.Lcd.printf("ID: %s \n", uhf.cards[i].epc_str.c_str());
					String epc = uhf.cards[i].epc_str;
					strncpy(msg_buffer, epc.c_str(), sizeof(msg_buffer) - 1);
					msg_buffer[sizeof(msg_buffer) - 1] = '\0';
					msg_hfid.data.size = epc.length();
					RCSOFTCHECK(rcl_publish(&pub_hfid, &msg_hfid, NULL));
					delay(100);
				}
				Serial.println("----------------------");
			}

			else {
				String no = "Nothing";
				Serial.println(no);
				Serial.println("----------------------");
				//M5.Lcd.printf("%s \n", no);
				strncpy(msg_buffer, no.c_str(), sizeof(msg_buffer) - 1);
				msg_buffer[sizeof(msg_buffer) - 1] = '\0';
				msg_hfid.data.size = no.length();
				RCSOFTCHECK(rcl_publish(&pub_hfid, &msg_hfid, NULL));
				delay(100);
			}
			
		}

		// timer
		if (eurobin_iot::init_mode == eurobin_iot::modes::TIMER && tof::ok) {
			uint16_t dist = tof_sensor.read();
			M5.Lcd.printf("Dist.: %d mm         \n", dist);
			if(dist < 200) {
				msg_time.data = true;
				RCSOFTCHECK(rcl_publish(&pub_time, &msg_time, NULL));
			}
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