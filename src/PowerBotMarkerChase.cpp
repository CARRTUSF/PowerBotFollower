/* 
 * File:   PowerBotMarkerChase.cpp
 * Author: andoni
 * 
 * Created on August 24, 2014, 2:46 PM
 */

#include "PowerBotMarkerChase.h"

// Constructor
PowerBotMarkerChase::PowerBotMarkerChase() : 
    ArAction("PowerBotMarkerChase", "Chases the marker."),
    clientSocket("192.168.0.100:3250"), networkOut(clientSocket),
    serverSocket("0.0.0.0:3251"), networkIn(serverSocket)
{
    myState = NO_TARGET;
    myMarkerLastSeen.setToNow();
    myMaxTime = 1000;
}

// Destructor
PowerBotMarkerChase::~PowerBotMarkerChase(void) {
}

// Fire action?
ArActionDesired *PowerBotMarkerChase::fire(ArActionDesired currentDesired) {
    bool flag = false;
    double xRel = 0;
    std::string inputBuffer;
    
    // Send the Coordinate Request
    networkOut << "REQ" << std::endl;

    // Reset the desired action
    myDesiredAction.reset();
    
    // Get the coordinate results
    getline(networkIn, inputBuffer);
    
    // If the command isn't a NOM command, set the marker to now
    if((inputBuffer.compare("NOM") != 0)) {
        flag = true;
        myMarkerLastSeen.setToNow();
    }
    
    // If we have not seen a blob in a while...
    if (myMarkerLastSeen.mSecSince() > myMaxTime) {
        if(myState != NO_TARGET) ArLog::log(ArLog::Normal, "Target Lost");
        myState = NO_TARGET;
    } else {
        // If we see a blob and haven't seen one before..
        if(myState != TARGET) {
            ArLog::log(ArLog::Normal, "Target Acquired");
        }
        myState = TARGET;
    }
    
    if((myState == TARGET) && (flag == true)) {
        xRel = std::atof(inputBuffer.c_str());
        
        ArLog::log(ArLog::Normal, "Turning");
        
        // Set the heading and velocity for the robot
        if (ArMath::fabs(xRel) < .10)
        {
            myDesiredAction.setDeltaHeading(0);
        }
        else
        {
            if (ArMath::fabs(-xRel * 10) <= 10)
                myDesiredAction.setDeltaHeading(-xRel * 10);
            else if (-xRel > 0)
                myDesiredAction.setDeltaHeading(10);
            else
                myDesiredAction.setDeltaHeading(-10);
        }

        //myDesiredAction.setVel(200);
        return &myDesiredAction;        
    } 
    
    // If we have no target, then don't set any action and let lower priority
    // actions (e.g. stop) control the robot.
    return &myDesiredAction;
}