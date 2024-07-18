
namespace eurobin_iot {
    namespace modes {
        enum {
            NONE = 0,
            KEY,
            TOF,
            HALL,
            SCALE,
            SIZE // number of modes
        };
    }
    // convert to strings (for debugging / display)
    const char* get_mode(uint8_t mode)
    {
        switch (mode) {
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
    namespace key {
        static const uint8_t pin = 33;
        static const uint8_t led_pin = 32;
        CRGB leds[1];
    } // namespace key
    namespace hall {
        static const uint8_t pin = 33;
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

        // variables
        uint8_t id = -1;
        uint8_t mode = 0;
        uint8_t init_mode = 0;

        int butt_c_activated = 0;
        int butt_mode_activated = 0;
        int16_t data_tof[3]; // signed because no default message for unsigned...

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
        if (mode == modes::TOF || mode == modes::SCALE)
            Wire.begin(); // join i2c bus (address optional for master)
#ifdef EUROBIN_IOT_ATOM
		//     void begin(bool SerialEnable = true, bool I2CEnable = true, bool DisplayEnable = false);
		M5.begin(true, true, true); // enable the display matrix
#else
        M5.begin();
#endif
    }

    void Node::init_sensors()
    {
        // check the time-of-flight
        if (mode == modes::TOF) {
            Serial.println("Initializing I2C...");
            Serial.print("Time of flight: ");
            uint8_t error = tof::check();
            if (tof::ok)
                Serial.println("ok");
            else {
                Serial.print("error ");
                Serial.println(error);
            }
        }

        // setup the key button
        if (mode == modes::KEY) {
            pinMode(key::pin, INPUT_PULLUP);
            FastLED.addLeds<SK6812, key::led_pin, GRB>(key::leds, 1);
            key::leds[0] = CRGB::Blue;
            FastLED.setBrightness(0);
        }

        // setup the hall sensor button
        if (mode == modes::HALL) {
            pinMode(hall::pin, INPUT);
        }

        // scale
        if (mode == modes::SCALE) {
            scale::init();
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
        }
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

        /// Time of flight
        String tof_topic_name = node_name + "/tof";
        msg_tof.data.capacity = 3;
        msg_tof.data.size = 3;
        msg_tof.data.data = data_tof;
        if (tof::ok) {
            RCCHECK(rclc_publisher_init_default(
                &pub_tof,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
                tof_topic_name.c_str()));
        }

        // scale
        String scale_topic_name = node_name + "/scale";
        if (init_mode == modes::SCALE) {
            RCCHECK(rclc_publisher_init_default(
                &pub_scale,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                scale_topic_name.c_str()));
        }

        // key
        String key_topic_name = node_name + "/key";
        if (init_mode == modes::KEY) {
            RCCHECK(rclc_publisher_init_default(
                &pub_key,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                key_topic_name.c_str()));
        }
        // hall sensor
        String hall_topic_name = node_name + "/hall";
        if (init_mode == modes::HALL) {
            RCCHECK(rclc_publisher_init_default(
                &pub_hall,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                hall_topic_name.c_str()));
        }

        RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    }

    void Node::update_sensors()
    {
        M5.update();

        // time-of-flight
        if (tof::ok) {
            uint16_t ambient_count, signal_count, dist;
            tof::read(&ambient_count, &signal_count, &dist);
            msg_tof.data.data[0] = dist;
            msg_tof.data.data[1] = ambient_count;
            msg_tof.data.data[2] = signal_count;
            RCSOFTCHECK(rcl_publish(&pub_tof, &msg_tof, NULL));
        }

        // scale
        if (init_mode == modes::SCALE) {
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
                FastLED.setBrightness(255);
                FastLED.show();
                msg_key.data = 1;
                eurobin_iot::sound::doorbell();
            }
            else {
                key::leds[0] = CRGB::Red;
                FastLED.setBrightness(255);
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

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }
} // namespace eurobin_iot