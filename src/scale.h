#ifndef __EUROBIN_IOT_SCALE_H
#define __EUROBIN_IOT_SCALE_H

#include "M5_Scales.h"

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

    }
} // namespace
#endif