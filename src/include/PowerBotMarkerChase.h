/* 
 * File:   PowerBotMarkerChase.h
 * Author: andoni
 *
 * Created on August 24, 2014, 2:46 PM
 */

#ifndef POWERBOTMARKERCHASE_H
#define	POWERBOTMARKERCHASE_H

#include "Aria.h"

class PowerBotMarkerChase : public ArAction{
public:
    // The state of the chase action
    enum State {
        NO_TARGET,      // There is no target in view
        TARGET,         // This is a target in view
    };

    PowerBotMarkerChase(void); //Constructor
    ~PowerBotMarkerChase(void); //Destructor
    
    // The action
    ArActionDesired *fire(ArActionDesired currentDesired);
    
    // Return the current state of this action
    State getState(void) { return myState; }

    // Height and width of pixels from frame-grabber
    enum {
        WIDTH = 160,
        HEIGHT = 120
    };

protected:
    ArActionDesired myDesiredAction;
    ArTime myMarkerLastSeen;
    State myState;
    int myMaxTime;
};

#endif	/* POWERBOTMARKERCHASE_H */

