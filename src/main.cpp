#include <Arduino.h>

#ifdef EUROBIN_IOT_ATOM
#include <M5Atom.h>
#else
#include <M5Core2.h>
#endif

#include <WiFi.h>
#include <WiFiMulti.h>
#include <micro_ros_platformio.h>

#include <Preferences.h> // this the EEPROM/flash interface

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <FastLED.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/int32.h>

#include "bytes.h"
#include "i2c.h"
#include "scale.h"
#include "sound.h"
#include "tof.h"

#include "config.h"
#include "eurobin_iot.h"

///////////////////////////////////////////////////
#ifdef EUROBIN_IOT_CORES2

namespace eurobin_iot {
    // Version for M5Core2
    void Node::init()
    {
        init_m5();

        // LCD
        M5.Lcd.fillScreen(BLACK); // Set the screen
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setTextColor(BLUE);
        M5.Lcd.printf("Eurobin IOT ROS2\n");
        M5.Lcd.printf("SSID: %s\n", config::wifi::essid);
        M5.Lcd.setTextColor(WHITE);

        init_wifi();

        M5.Lcd.setTextColor(GREEN, BLACK);
        M5.lcd.print("RSSI: ");
        M5.lcd.println(WiFi.RSSI());
        M5.lcd.print("IP address: ");
        M5.lcd.println(WiFi.localIP());

        init_sensors();

        init_ros();

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
        update_sensors();

        if (mode != init_mode) {
            M5.Lcd.setCursor(0, 240 - 20);
            M5.Lcd.printf("-> RESET\n");
        }

        M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
        M5.Lcd.setCursor(0, 90);

        M5.Lcd.printf("Mode:%s          \n", get_mode(mode));

        // button
        M5.Lcd.printf("Buttons: %d %d %d      \n", M5.BtnA.read(), M5.BtnB.read(), M5.BtnC.read());
        msg_button.data = M5.BtnA.read();
        RCSOFTCHECK(rcl_publish(&pub_button, &msg_button, NULL));
        if (msg_button.data == 1)
            eurobin_iot::sound::ding();

        // time-of-flight
        if (tof::ok)
            M5.Lcd.printf("Dist.: %d mm         \n", msg_tof.data.data[0]);

        // scale
        if (init_mode == modes::SCALE)
            M5.Lcd.printf("weight: %d grams     \n", msg_scale.data);

        // red key
        if (init_mode == modes::KEY) {
            if (msg_key.data == 1)
                M5.Lcd.println(("Key: 1       "));
            else
                M5.Lcd.println(("Key: 0      "));
        }

        // hall sensor
        if (init_mode == modes::HALL)
            M5.Lcd.printf("Hall: %d\n", msg_hall.data);

        // mode
        if (M5.BtnB.read())
            butt_mode_activated++;
        if (butt_mode_activated > 5) {
            mode = (mode + 1) % modes::SIZE;
            prefs.putUInt("mode", mode);
            butt_mode_activated = 0;
        }

        if (M5.BtnC.read()) {
            butt_c_activated++;
            M5.Lcd.printf("RESET ID [50] %d", butt_c_activated);
        }
        else
            butt_c_activated = 0;

        if (butt_c_activated >= 50) {
            M5.Lcd.printf(" => RESET ID");
            int r = (int)random(100);
            Serial.print("NEW ID:");
            prefs.putUInt("id", r);
            // preferences.end();
            usleep(1000);
            ESP.restart();
        }
    }
} // namespace eurobin_iot
///////////////////////////////////////////////////
#elif defined(EUROBIN_IOT_ATOM)

namespace eurobin_iot {
    // Version for M5Core2
    void Node::init()
    {
        init_m5();
		M5.dis.drawpix(0, 0x00ff00);        
        init_wifi();
		M5.dis.drawpix(1, 0x00ff00);        
        init_sensors();
		M5.dis.drawpix(2, 0x00ff00);
        init_ros();
		M5.dis.drawpix(3, 0x00ff00);        

    }

    void Node::loop()
    {
		static uint8_t c = 0;
        update_sensors();
		if (c++ % 2 == 0)
			M5.dis.drawpix(4, 0x0000ff);        
		else
			M5.dis.drawpix(4, 0x000000);        
		
    }
} // namespace eurobin_iot

#endif



void setup()
{
    eurobin_iot::node.init();
}

void loop()
{
    eurobin_iot::node.loop();
}