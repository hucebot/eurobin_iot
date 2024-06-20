// 1. Fill this form with the information of your network
// 2. Rename it config.h 
#ifndef EUROBIN_IOT_CONFIG_H_
#define EUROBIN_IOT_CONFIG_H_

namespace eurobin_iot {
    namespace config {
        namespace wifi {
            static constexpr char essid[] = "FIXME_YOUR_ESSID";
            static constexpr char password[] = "FIXME_YOUR_WIFI_PASSWORD";
        }
        namespace agent { // FIXME: port and IP of the microros agent
            static constexpr unsigned port = 8888;
            static constexpr int ip[4] = {192, 168, 50, 100};
        }
    }
}

#endif