//
// Created by advcc on 1/24/22.
//

#ifndef KNIFEFISH_ARMCONNECTIONSTATUS_HPP
#define KNIFEFISH_ARMCONNECTIONSTATUS_HPP

#include <string>

struct ArmConnectionStatus {
    ::std::string ipAddress;
    ::std::uint16_t serverVersion;
};

#endif //KNIFEFISH_ARMCONNECTIONSTATUS_HPP
