/* 
 * File:   PowerBotMarkerChase.cpp
 * Author: andoni
 * 
 * Created on August 24, 2014, 2:46 PM
 */

#include "PowerBotMarkerChase.h"

// Constructor
PowerBotMarkerChase::PowerBotMarkerChase() : 
    ArAction("PowerBotMarkerChase", "Chases the marker.") {
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
    //Reset the desired action
    myDesiredAction.reset();
}