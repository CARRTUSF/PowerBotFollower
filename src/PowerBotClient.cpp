/* 
 * File:   PowerBotClient.cpp
 * Author: Andoni Aguirrezabal
 * 
 * Created on August 19, 2014, 4:09 PM
 */

#include "PowerBotClient.h"
#include <iostream>

PowerBotClient::PowerBotClient() :
    pbOutputNumbersCB(this, &PowerBotClient::handleOutputNumbers)
{
    Aria::init();
    lastPosX = 0.0;
    lastPosY = 0.0;
    lastHeading = 0.0;
}

PowerBotClient::~PowerBotClient() {
}

bool PowerBotClient::connect() {
    if(pbClient.blockingConnect("192.168.0.100", 7272)) {
        pbClient.runAsync();
        pbClient.addHandler("updateNumbers", &pbOutputNumbersCB);
        return 1;
    } else {
        return 0;
    }
}

bool PowerBotClient::getRunningWithLock() {
    return pbClient.getRunningWithLock();
}

void PowerBotClient::handleOutputNumbers(ArNetPacket *packet) {
    
}