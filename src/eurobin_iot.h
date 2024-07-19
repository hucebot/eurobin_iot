
namespace eurobin_iot {
    namespace modes {
        enum {
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
    // convert to strings (for debugging / display)
    const char* get_mode(uint8_t mode)
    {
        switch (mode)
        {
            case 0:
                return "UNDERFINED";
            case 1:
                return "KEY";
            case 2:
                return "TOFM2";
            case 3:
                return "TOFM4";
            case 4:
                return "HALL";
            case 5:
                return "SCALE";
            case 6:
                return "RFID";
            case 7:
                return "TIMER";
            default:
                return "ERROR";
        }
    }

#ifdef EUROBIN_IOT_ATOM
    namespace atom_display {
        char message_scroll[100];
	    int currentOffset = 0; // Current offset for scrolling
	    const long interval = 1000; 
	    unsigned long last_time, current_time;
	    int count_clicks = 0;
        CRGB colorMode = CRGB::Red;
        CRGB colorID = CRGB::White;
        uint8_t charBitmaps[][5] PROGMEM = {
            {0b00110, 0b01001, 0b01111, 0b01001, 0b01001}, // A 
            {0b01110, 0b01001, 0b01110, 0b01001, 0b01110}, // B 
            {0b00111, 0b01000, 0b01000, 0b01000, 0b00111}, // C 
            {0b01110, 0b01001, 0b01001, 0b01001, 0b01110}, // D 
            {0b01111, 0b01000, 0b01110, 0b01000, 0b01111}, // E 
            {0b01111, 0b01000, 0b01110, 0b01000, 0b01000}, // F 
            {0b01111, 0b01000, 0b01011, 0b01001, 0b01111}, // G 
            {0b01001, 0b01001, 0b01111, 0b01001, 0b01001}, // H 
            {0b01110, 0b00100, 0b00100, 0b00100, 0b01110}, // I 
            {0b00111, 0b00010, 0b00010, 0b01010, 0b00100}, // J 
            {0b01001, 0b01010, 0b01100, 0b01010, 0b01001}, // K 
            {0b01000, 0b01000, 0b01000, 0b01000, 0b01110}, // L 
            {0b01001, 0b01111, 0b01001, 0b01001, 0b01001}, // M 
            {0b01001, 0b01101, 0b01011, 0b01001, 0b01001}, // N 
            {0b00110, 0b01001, 0b01001, 0b01001, 0b00110}, // O 
            {0b01110, 0b01001, 0b01110, 0b01000, 0b01000}, // P 
            {0b00110, 0b01001, 0b01001, 0b01011, 0b00111}, // Q 
            {0b01110, 0b01001, 0b01110, 0b01010, 0b01001}, // R 
            {0b00111, 0b01000, 0b00110, 0b00001, 0b01110}, // S 
            {0b01110, 0b00100, 0b00100, 0b00100, 0b00100}, // T 
            {0b01001, 0b01001, 0b01001, 0b01001, 0b00110}, // U 
            {0b00101, 0b00101, 0b00101, 0b00101, 0b00010}, // V 
            {0b01001, 0b01001, 0b01001, 0b01111, 0b01001}, // W
            {0b01001, 0b01001, 0b00110, 0b01001, 0b01001}, // X 
            {0b00101, 0b00101, 0b00010, 0b00010, 0b00010}, // Y 
            {0b01111, 0b00001, 0b00110, 0b01000, 0b01111}, // Z 
            {0b00110, 0b01001, 0b01011, 0b01101, 0b00110}, // 0 
            {0b00100, 0b01100, 0b00100, 0b00100, 0b01110}, // 1 
            {0b01110, 0b00001, 0b00110, 0b01000, 0b01111}, // 2 
            {0b01110, 0b00001, 0b01110, 0b00001, 0b01110}, // 3 
            {0b01001, 0b01001, 0b01111, 0b00001, 0b00001}, // 4 
            {0b01111, 0b01000, 0b01110, 0b00001, 0b01110}, // 5 
            {0b00110, 0b01000, 0b01110, 0b01001, 0b01110}, // 6 
            {0b01111, 0b00001, 0b00010, 0b00010, 0b00010}, // 7 
            {0b00110, 0b01001, 0b00110, 0b01001, 0b00110}, // 8 
            {0b00110, 0b01001, 0b00111, 0b00001, 0b00110}, // 9 
            };

        void clear() {
            for (int8_t i = 0; i < 25; i++) {
                if(i == 0 || i == 5 || i == 10 || i == 15 || i == 20) continue;
                M5.dis.drawpix(i, CRGB::Black);
            }
        }

        void reset_id_mode_display() {
            bool change = false;
            while(true) {
                CRGB color = change ? CRGB::Red : CRGB::Blue;
                for (int8_t i = 0; i < 25; i++) {
                    if(i == 0 || i == 5 || i == 10 || i == 15 || i == 20) {
                        M5.dis.drawpix(i, color);
                    }
                }
                change = !change;
                usleep(200000);
            }
        }

        bool double_click() {
            if(M5.Btn.wasPressed()) {
                current_time = millis();
                if(count_clicks == 0) {
                    count_clicks++;
                    last_time = current_time;
                }
                else if (current_time - last_time <= interval) 
                    count_clicks++;

                else 
                    last_time = current_time;

                if(count_clicks == 2) {
                    count_clicks = 0;
                    return true; 
                }
            }
            return false;
        }

        void displayCharacter(int x, int y, uint8_t character[5], CRGB color) {
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 5; j++) {
                    bool pixelOn = (character[i] & (1 << (4 - j))) != 0;
                    if(x+j == 0) continue;
                    M5.dis.drawpix(x + j, y + i, pixelOn ? color : 0x000000);
                }
            }
        }

        void displayMessageFrame() {
            //M5.dis.clear();
            bool mode_number = true;
            clear();
            int len = strlen(message_scroll);
            for (int i = 0; i < len; i++) {
                if(i == len - 2 ) mode_number = false;
                if (message_scroll[i] >= 'A' && message_scroll[i] <= 'Z') {
                    displayCharacter(6 * i - currentOffset, 0, charBitmaps[message_scroll[i] - 'A'], colorMode);
                } else if (message_scroll[i] >= '0' && message_scroll[i] <= '9') {
                    displayCharacter(6 * i - currentOffset, 0, charBitmaps[message_scroll[i] - '0' + 26], mode_number ? colorMode:colorID );
                }
            }
        }
    }
#else
    namespace ui_core2 {
        uint8_t id;
        float battery;
        float battery_percentage;
        const long interval = 60000;
        unsigned long current_time,start_time;
        bool start=false;
        String topic_name;
        uint8_t mode;
        void init_ui_core2() {

            // Set the background as Black
            M5.Lcd.fillScreen(BLACK); 

            // Title
            M5.Lcd.fillRect(5,0,310,15, DARKGREY);
            M5.Lcd.setCursor(90, 0);
            M5.Lcd.setTextSize(2);
            M5.Lcd.setTextColor(WHITE);
            M5.Lcd.printf("Eurobin IOT\n");

            // SSID
            M5.Lcd.fillRect(5,20,310,35, DARKGREY);
            //M5.Lcd.fillRect(185,35, 130, 20, DARKGREY);
            M5.Lcd.setCursor(5, 20);
            M5.Lcd.printf("SSID:\n");
            M5.Lcd.setTextColor(GREEN);
            M5.Lcd.setCursor(5, 40);
            M5.Lcd.printf("%s", config::wifi::essid);
            
            M5.Lcd.setTextColor(WHITE);

            // RSSI
            //M5.Lcd.fillRect(190,20,125,20, DARKGREY);
            M5.Lcd.setCursor(193, 20);
            M5.Lcd.setTextColor(WHITE);
            M5.lcd.print("RSSI: ");
            M5.Lcd.setTextColor(GREEN);
            M5.lcd.println(WiFi.RSSI());

            // Batery
            M5.Lcd.drawRect(190,100, 125, 30, RED);
            M5.Lcd.setCursor(195,105);
            battery = M5.Axp.GetBatVoltage();
            battery_percentage = (battery < 3.2) ? 0:(battery - 3.2) * 100;
            M5.Lcd.setTextColor(WHITE);
            M5.Lcd.printf("Power:");
            M5.Lcd.setTextColor(GREEN, BLACK);
            M5.Lcd.printf("%.1f", battery_percentage);

            // IP adress
            M5.Lcd.fillRect(5,60,310,15, DARKGREY);
            M5.Lcd.setCursor(5, 60);
            M5.Lcd.setTextColor(WHITE);
            M5.lcd.print("IP address: ");
            M5.Lcd.setTextColor(GREEN);
            M5.lcd.println(WiFi.localIP());

            // Topic name
            M5.Lcd.fillRect(5,80, 310, 15, DARKGREY);
            M5.Lcd.setCursor(5,80);
            M5.Lcd.setTextColor(TFT_WHITE);
            M5.Lcd.printf("Topic:");
            M5.Lcd.setTextColor(TFT_GREEN);
            M5.Lcd.printf("%s", topic_name.c_str());

            // ID
            M5.Lcd.fillRoundRect(320 - 100, 240 - 70, 100, 70, 15, TFT_YELLOW);
            M5.Lcd.setCursor(320 - 100 + 10, 240 - 58);
            M5.Lcd.setTextSize(14);
            M5.Lcd.setTextColor(TFT_BLACK);
            M5.Lcd.printf("%d", id);

            // mode symbol
            M5.Lcd.fillRoundRect(140, 240 - 25, 60, 25, 10, TFT_YELLOW);
            M5.Lcd.setCursor(150, 240 - 20);
            M5.Lcd.setTextSize(2);
            M5.Lcd.printf("mode");

            // mode microcontroller
            M5.Lcd.setTextColor(TFT_WHITE);
            M5.Lcd.setCursor(5, 105);
            M5.Lcd.printf("Mode:");
            M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
            M5.Lcd.printf("%s       ", get_mode(mode));

            // rectangle
            M5.Lcd.drawRect(0,100, 190, 65, RED);
        }

        void check_battery() {
            if(!start) {
                start_time = millis();
                start = !start;
            }
            else {
                current_time = millis();
                if(current_time - start_time > interval) {
                    battery = M5.Axp.GetBatVoltage();
                    battery_percentage = (battery < 3.2) ? 0:(battery - 3.2) * 100;
                    start = !start;
                }
            }
        }
    }
#endif

    namespace key {
        #ifdef EUROBIN_IOT_CORES2
        static const uint8_t pin = 33;
        static const uint8_t led_pin = 32;
        #else
        static const uint8_t pin = 32;
        static const uint8_t led_pin = 26;
        #endif
        CRGB leds[1];
    } // namespace key
    namespace hall {
        #ifdef EUROBIN_IOT_CORES2
        static const uint8_t pin = 33;
        #else
        static const uint8_t pin = 32;
        #endif
    }
    namespace rfid
	{
        #ifdef EUROBIN_IOT_CORES2
		static const uint8_t rx = 33;
		static const uint8_t tx = 32;
        #else 
        static const uint8_t rx = 32;       
		static const uint8_t tx = 26;
        #endif
		bool ok=false;
	}

    class Node {
    public:
        Node() {}
        void init();
        void loop();

    protected:
        void init_m5();
        void init_wifi();
        void init_sensors();
        void init_ros();
        void update_sensors();

        // Arduino-style objects
#ifdef EUROBIN_IOT_CORES2
        Speaker speaker;
#endif
        Preferences prefs;
        WiFiMulti wifiMulti;
        Unit_UHF_RFID uhf;
        VL53L1X tof_sensor;

        // variables
        uint8_t id = -1;
        uint8_t mode = 0;
        uint8_t init_mode = 0;

        int butt_c_activated = 0;
        int butt_mode_activated = 0;
        int16_t data_tofm2[3]; // signed because no default message for unsigned...
        char rfid_msg_buffer[25];
		String topic_name;

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
		std_msgs__msg__Bool msg_timer;
		rcl_publisher_t pub_timer;
    };
    // !! Unique instance / singleton
    Node node;
} // namespace eurobin_iot

#ifdef EUROBIN_IOT_CORES2
#define EUROBIN_IOT_ERROR()                                                      \
    printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
    M5.Lcd.printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);
#else
#define EUROBIN_IOT_ERROR() \
    printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);
#endif

#define RCCHECK(fn)                    \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            EUROBIN_IOT_ERROR()        \
            vTaskDelete(NULL);         \
        }                              \
    }

#define RCSOFTCHECK(fn)                \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            EUROBIN_IOT_ERROR();       \
        }                              \
    }

namespace eurobin_iot {
    void Node::init_m5()
    {
        prefs.begin("eurobin_iot");
        mode = prefs.getUInt("mode", 0);
        init_mode = mode;
        prefs.begin("eurobin_iot");
        id = prefs.getUInt("id", 0);

        Serial.printf("ROS_IOT -> MODE: %d %s\n", mode, get_mode(mode));
#ifdef EUROBIN_IOT_ATOM
        String config = (0 <= id && id <= 9 ) ? "%s0%d":"%s%d";
        snprintf(atom_display::message_scroll, sizeof(atom_display::message_scroll), config.c_str(), get_mode(mode), id);
		//     void begin(bool SerialEnable = true, bool I2CEnable = true, bool DisplayEnable = false);
		M5.begin(true, true, true); // enable the display matrix
        if (mode == modes::TOFM2 || mode == modes::SCALE || mode == modes::TOFM4 || mode == modes::TIMER)
            Wire.begin(25,21);
#else   
        ui_core2::mode = mode;
        ui_core2::id = id;
        M5.begin();
        if (mode == modes::TOFM2 || mode == modes::SCALE || mode == modes::TOFM4 || mode == modes::TIMER)
            Wire.begin(); // join i2c bus (address optional for master)
#endif
    }

    void Node::init_sensors()
    {
        // check the time-of-flight
        if (mode == modes::TOFM2 || mode == modes::TOFM4 || mode == modes::TIMER) {
            Serial.println("Initializing I2C...");
            Serial.print("Time of flight: ");
            uint8_t error = tof::check();
            if (tof::ok) {
                if(mode != modes::TOFM2){
					tof_sensor.init();
					tof_sensor.setDistanceMode(VL53L1X::Short);
  					tof_sensor.setMeasurementTimingBudget(20000);
					tof_sensor.startContinuous(20);
				}
                #ifdef EUROBIN_IOT_ATOM
                M5.dis.drawpix(5, CRGB::Green);
                #endif
                Serial.println("ok");
            }
            else {
                #ifdef EUROBIN_IOT_ATOM
                M5.dis.drawpix(5, CRGB::Red);
                #endif
                Serial.print("error ");
                Serial.println(error);
            }
        }

        // setup the key button
        if (mode == modes::KEY) {
            pinMode(key::pin, INPUT_PULLUP);
            FastLED.addLeds<SK6812, key::led_pin, GRB>(key::leds, 1);
            key::leds[0] = CRGB::Blue;
            #ifdef EUROBIN_IOT_CORES2
            FastLED.setBrightness(0);
            #endif
        }

        // setup the hall sensor button
        if (mode == modes::HALL) {
            pinMode(hall::pin, INPUT);
        }

        // scale
        if (mode == modes::SCALE) {
            scale::init();
            #ifdef EUROBIN_IOT_ATOM
			if(scale::ok) {
				M5.dis.drawpix(5, CRGB::Green);
			}
			else {
				M5.dis.drawpix(5, CRGB::Red);
			}
            #endif
        }

        //rfid
		if (mode == modes::RFID) {
			uhf.begin(&Serial2, 115200, rfid::rx, rfid::tx, false);
			String info = uhf.getVersion();
			if (info != "ERROR") {
                #ifdef EUROBIN_IOT_ATOM
                M5.dis.drawpix(5, CRGB::Green);
                #endif
				Serial.println(info);
				uhf.setTxPower(2600);
				rfid::ok = true;
			}
			else {
                #ifdef EUROBIN_IOT_ATOM
                M5.dis.drawpix(5, CRGB::Red);
                #endif
				Serial.println("Error connecting the RFID");
			}
			
		}
#ifdef EUROBIN_IOT_CORES2
        speaker.begin();
        speaker.InitI2SSpeakOrMic(MODE_SPK);
#endif
    }

    void Node::init_wifi()
    {
        wifiMulti.addAP(config::wifi::essid, config::wifi::password);
        while (wifiMulti.run() != WL_CONNECTED) {
            delay(500);
            printf("Waiting for wifi...\n");
            #ifdef EUROBIN_IOT_ATOM
            M5.dis.drawpix(0, CRGB::Red);
            #endif
        }
        #ifdef EUROBIN_IOT_ATOM
        M5.dis.drawpix(0, CRGB::Green);
        #endif
        printf("Wifi OK\n");
    }

    void Node::init_ros()
    {
        printf("Agent: %d.%d.%d.%d:%d\n", config::agent::ip[0], config::agent::ip[1], config::agent::ip[2], config::agent::ip[3], config::agent::port);
        IPAddress agent_ip(config::agent::ip[0], config::agent::ip[1], config::agent::ip[2], config::agent::ip[3]);
        locator.address = agent_ip;
        locator.port = config::agent::port;
        rmw_uros_set_custom_transport(
            false,
            (void*)&locator,
            platformio_transport_open,
            platformio_transport_close,
            platformio_transport_write,
            platformio_transport_read);

        RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

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

        /// Time of flight M2
        if (tof::ok && init_mode == modes::TOFM2) {
            topic_name = node_name + "/tofm2";
            msg_tofm2.data.capacity = 3;
            msg_tofm2.data.size = 3;
            msg_tofm2.data.data = data_tofm2;
            RCCHECK(rclc_publisher_init_default(
                &pub_tofm2,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
                topic_name.c_str()));
        }

        /// Time of flight M4
		if (tof::ok && init_mode == modes::TOFM4) {
			topic_name = node_name + "/tofm4";
			RCCHECK(rclc_publisher_init_default(
				&pub_tofm4,
				&node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
				topic_name.c_str()));
		}

        // scale sensor
        if (init_mode == modes::SCALE && scale::ok) {
            topic_name = node_name + "/scale";
            RCCHECK(rclc_publisher_init_default(
                &pub_scale,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                topic_name.c_str()));
        }

        // key
        if (init_mode == modes::KEY) {
            topic_name = node_name + "/key";
            RCCHECK(rclc_publisher_init_default(
                &pub_key,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                topic_name.c_str()));
        }
        // hall sensor
        if (init_mode == modes::HALL) {
            topic_name = node_name + "/hall";
            RCCHECK(rclc_publisher_init_default(
                &pub_hall,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                topic_name.c_str()));
        }

        // rfid sensor
		if (rfid::ok && init_mode== eurobin_iot::modes::RFID) {
			topic_name = node_name + "/rfid";
			msg_hfid.data.data = rfid_msg_buffer;
			msg_hfid.data.capacity = sizeof(rfid_msg_buffer);
			msg_hfid.data.size = 0;
			RCCHECK(rclc_publisher_init_default(
				&pub_hfid,
				&node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
				topic_name.c_str()));
		}

		//time
		if (tof::ok && init_mode == eurobin_iot::modes::TIMER) {
			topic_name = node_name + "/timer";
			RCCHECK(rclc_publisher_init_default(
				&pub_timer,
				&node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
				topic_name.c_str()));
		}
#ifdef EUROBIN_IOT_CORES2
        ui_core2::topic_name = topic_name;
#endif

        RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    }

    void Node::update_sensors()
    {
        M5.update();

        //battery = M5.Axp.GetBatVoltage();
		//battery_percentage = (battery < 3.2) ? 0:(battery - 3.2) * 100;

        // time-of-flight M2
        if (tof::ok && init_mode == modes::TOFM2) {
            uint16_t ambient_count, signal_count, dist;
            tof::read(&ambient_count, &signal_count, &dist);
            msg_tofm2.data.data[0] = dist;
            msg_tofm2.data.data[1] = ambient_count;
            msg_tofm2.data.data[2] = signal_count;
            RCSOFTCHECK(rcl_publish(&pub_tofm2, &msg_tofm2, NULL));
        }

        // time-of-flight M4
		if (tof::ok && init_mode == eurobin_iot::modes::TOFM4 && tof::ok) {
			uint16_t dist = tof_sensor.read();
			msg_tofm4.data = dist;
			RCSOFTCHECK(rcl_publish(&pub_tofm4, &msg_tofm4, NULL));
		}

        // scale
        if (init_mode == modes::SCALE && scale::ok) {
            // scale::print();
            int w = scale::weight();
            msg_scale.data = w;
            RCSOFTCHECK(rcl_publish(&pub_scale, &msg_scale, NULL));
            if (scale::button()) {
                scale::tare();
                Serial.println("scale::tare");
            }
        }

        // red key
        if (init_mode == modes::KEY) {
            if (!digitalRead(key::pin)) {
                key::leds[0] = CRGB::Blue;
                
                msg_key.data = 1;
                #ifdef EUROBIN_IOT_CORES2
                FastLED.setBrightness(255);
                eurobin_iot::sound::doorbell();
                #endif
                FastLED.show();
            }
            else {
                key::leds[0] = CRGB::Red;
                #ifdef EUROBIN_IOT_CORES2
                FastLED.setBrightness(255);
                #endif
                FastLED.show();
                msg_key.data = 0;
            }
            RCSOFTCHECK(rcl_publish(&pub_key, &msg_key, NULL));
        }

        // hall sensor
        if (init_mode == modes::HALL) {
            if (digitalRead(hall::pin))
                msg_hall.data = 0;
            else
                msg_hall.data = 1;
            RCSOFTCHECK(rcl_publish(&pub_hall, &msg_hall, NULL));
        }

        // rfid sensor
		if (init_mode == eurobin_iot::modes::RFID && rfid::ok) {
			uint8_t result = uhf.pollingMultiple(10);
			if(result > 0) {
				for (uint8_t i = 0; i < result; i++) {
					Serial.println("epc: " + uhf.cards[i].epc_str);
					String epc = uhf.cards[i].epc_str;
					strncpy(rfid_msg_buffer, epc.c_str(), sizeof(rfid_msg_buffer) - 1);
					rfid_msg_buffer[sizeof(rfid_msg_buffer) - 1] = '\0';
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
				strncpy(rfid_msg_buffer, no.c_str(), sizeof(rfid_msg_buffer) - 1);
				rfid_msg_buffer[sizeof(rfid_msg_buffer) - 1] = '\0';
				msg_hfid.data.size = no.length();
				RCSOFTCHECK(rcl_publish(&pub_hfid, &msg_hfid, NULL));
				delay(100);
			}
			
		}

        // timer
		if (init_mode == eurobin_iot::modes::TIMER && tof::ok) {
			uint16_t dist = tof_sensor.read();
            msg_timer.data = (dist < 200) ? true:false;
            if(msg_timer.data)
                RCSOFTCHECK(rcl_publish(&pub_timer, &msg_timer, NULL));
		}

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }
} // namespace eurobin_iot