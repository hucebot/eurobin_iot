#include <Arduino.h>

#ifdef EUROBIN_IOT_ATOM
#include <M5Atom.h>
#else
#include <M5Core2.h>
#endif

#include <WiFi.h>
#include <WiFiMulti.h>
#include <micro_ros_platformio.h>
#include <chrono>
#include <ctime> 
#include "UNIT_UHF_RFID.h"

#include <Preferences.h> // this the EEPROM/flash interface

#include <stdio.h>
#include <string.h>
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
#include "VL53L1X.h"

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

        init_wifi();

        init_sensors();

        init_ros();

        init_ui_core2::init_ui_core2();

        eurobin_iot::sound::ding(); // say we are ready!
    }

    void Node::loop()
    {
        update_sensors();

        if (mode != init_mode) {
            // If mode has been changed, show on the screen the new mode and warn the user to reset core2
            M5.Lcd.setTextColor(TFT_WHITE);
			M5.Lcd.setCursor(5, 105);
			M5.Lcd.printf("Mode:");
			M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
			M5.Lcd.printf("%s       ", get_mode(mode));
			M5.Lcd.drawRect(0,100, 190, 65, RED);
            M5.Lcd.setCursor(0, 240 - 20);
            M5.Lcd.printf("-> RESET\n");
        }

        

        // button
        M5.Lcd.setCursor(5, 125);
		M5.Lcd.setTextColor(TFT_WHITE);
		M5.Lcd.printf("Btns:");
		M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
		M5.Lcd.printf("%d %d %d", M5.BtnA.read(), M5.BtnB.read(), M5.BtnC.read());
		M5.Lcd.setCursor(5, 145);
        msg_button.data = M5.BtnA.read();
        RCSOFTCHECK(rcl_publish(&pub_button, &msg_button, NULL));
        if (msg_button.data == 1)
            eurobin_iot::sound::ding();

        // time-of-flight m2
        if (tof::ok && init_mode == modes::TOFM2)
            M5.Lcd.printf("Dist:%d mm  ", msg_tofm2.data.data[0]);


        // time-of-flight m4
        if (init_mode == modes::TOFM4 && tof::ok)
            M5.Lcd.printf("Dist:%d mm  ", msg_tofm4.data);

        // scale sensor
        if (init_mode == modes::SCALE && scale::ok)
            M5.Lcd.printf("weight:%d g  ", msg_scale.data);

        // red key
        if (init_mode == modes::KEY) {
            M5.Lcd.printf("Key:%d", msg_key.data);
        }

        // hall sensor
        if (init_mode == modes::HALL)
            M5.Lcd.printf("Hall:%d", msg_hall.data);

        // timer 
        if(init_mode == modes::TIMER && tof::ok) {
            M5.Lcd.printf("Timer:%d", msg_timer.data);
        }

        // mode
        if (M5.BtnB.read())
            butt_mode_activated++;
        else 
            butt_mode_activated = 0;
        if (butt_mode_activated > 5) {
            mode = (mode + 1) % modes::SIZE;
            prefs.putUInt("mode", mode);
            butt_mode_activated = 0;
        }

        if (M5.BtnC.read()) {
            M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
			M5.Lcd.setCursor(5, 170);
            butt_c_activated++;
            M5.Lcd.printf("RESET ID [50] %d ", butt_c_activated);
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
		//M5.dis.drawpix(0, 0x00ff00);        
        init_wifi();
		//M5.dis.drawpix(1, 0x00ff00);        
        init_sensors();
		//M5.dis.drawpix(2, 0x00ff00);
        init_ros();
		//M5.dis.drawpix(3, 0x00ff00);        

    }

    void Node::loop()
    {
        atom_display::displayMessageFrame();
    	atom_display::currentOffset++;
    	if (atom_display::currentOffset >= strlen(atom_display::message_scroll) * 6) {
      		atom_display::currentOffset = 0;
    	}


        update_sensors();

        if(atom_display::double_click()) {
			mode = (mode + 1) % modes::SIZE;
			prefs.putUInt("mode", mode);
			atom_display::reset_id_mode_display();
		}

		if(M5.Btn.pressedFor(5000)) {
			int r = (int)random(100);
			Serial.print("NEW ID:");
			prefs.putUInt("id", r);
			// preferences.end();
			M5.dis.fillpix(CRGB::Blue);
			usleep(2000000);
			ESP.restart();
		}

		//static uint8_t c = 0;
        
		/* if (c++ % 2 == 0)
			M5.dis.drawpix(4, 0x0000ff);        
		else
			M5.dis.drawpix(4, 0x000000);   */      
		
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