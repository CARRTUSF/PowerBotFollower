/* 
 * File:   PowerBotClient.cpp
 * Author: Andoni Aguirrezabal
 * 
 * Created on August 19, 2014, 4:09 PM
 */

#include "PowerBotClient.h"
#include <iostream>
#include <tgmath.h>

#define PI 3.14159265

PowerBotClient::PowerBotClient() :
    pbOutputNumbersCB(this, &PowerBotClient::handleOutputNumbers)
{
    Aria::init();
    pbVoltage = 0.0;
    pbX = 0.0;
    pbY = 0.0;
    pbTh = 0.0;
    pbVel = 0.0;
    pbRotVel = 0.0;
    pbLatVel = 0.0;
    pbTemperature = 0.0;
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
    pbVoltage = ((double)packet->bufToByte2())/10.0;
    pbX = (double)packet->bufToByte4();
    pbY = (double)packet->bufToByte4();
    pbTh = (double)packet->bufToByte2();
    pbVel = (double)packet->bufToByte2();
    pbRotVel = (double)packet->bufToByte2();
    pbLatVel = (double)packet->bufToByte2();
    pbTemperature = (double)packet->bufToByte();
}

void PowerBotClient::transformPoints(double &outX, double &outY) {
    double pbThRad = (((pbTh - 90)*PI)/180);
    outX = (outX*cos(pbThRad) - outY*sin(pbThRad)) + pbX;
    outY = (outY*cos(pbThRad) + outX*sin(pbThRad)) + pbY;
}
