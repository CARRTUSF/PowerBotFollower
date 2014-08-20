/* 
 * File:   PowerBotClient.h
 * Author: Andoni Aguirrezabal
 *
 * Created on August 19, 2014, 4:09 PM
 */

#ifndef POWERBOTCLIENT_H
#define	POWERBOTCLIENT_H

#include "Aria.h"
#include "ArNetworking.h"

class PowerBotClient {
public:
    PowerBotClient();
    virtual ~PowerBotClient();
    
public:
    bool connect();
    bool getRunningWithLock();
    
private:
    void handleOutputNumbers(ArNetPacket *packet);

private:
    ArClientBase pbClient;
    ArFunctor1C<PowerBotClient, ArNetPacket *> pbOutputNumbersCB;
    
    float lastPosX;
    float lastPosY;
    float lastHeading;
};

#endif	/* POWERBOTCLIENT_H */
