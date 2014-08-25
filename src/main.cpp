/* 
 * File:   main.cpp
 * Author: Andoni Aguirrezabal
 *
 * Created on August 24, 2014, 2:19 PM
 */

#include "PowerBotMarkerChase.h"

using namespace std;

// Callback to enable/disable the keyboard driving action
void toggleAction(ArAction* action)
{
    if(action->isActive()) {
        action->deactivate();
        ArLog::log(ArLog::Normal, "%s action is now deactivated.", action->getName());
    } else {
        action->activate();
        ArLog::log(ArLog::Normal, "%s action is now activated.", action->getName());
    }
}

int main(int argc, char** argv) {
    Aria::init();
    ArArgumentParser parser(&argc, argv); //Argument Parser
    parser.loadDefaultArguments(); //Load default arguments, in case
    ArRobot pbRobot; //PowerBot Reference
    // Used to initiate the Connection to PowerBot
    ArRobotConnector pbRobotConnector(&parser, &pbRobot); 
    // Used to initiate the Connection to PowerBot's laser
    ArLaserConnector pbLaserConnector(&parser, &pbRobot, &pbRobotConnector);
    // Sonar for basic obstacle avoidance
    ArSonarDevice pbSonar;
    
    // Connect to the robot, get some initial data from it such as type and name,
    // and then load parameter files for this robot.
    if(!pbRobotConnector.connectRobot()) {
        ArLog::log(ArLog::Terse, "PowerBotFollower: Could not connect to the robot.");
        if(parser.checkHelpAndWarnUnparsed())
        {
            // -help not given
            Aria::logOptions();
            Aria::exit(1);
        }
    }
    
    if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
    }

    ArLog::log(ArLog::Normal, "PowerBotFollower: Connected to robot.");
    
    pbRobot.runAsync(true); //Run the PowerBot processing in the background
    
    // Connect to laser(s) as defined in parameter files.
    // (Some flags are available as arguments to connectLasers() to control error 
    // behavior and to control which lasers are put in the list of lasers stored 
    // by ArRobot. See docs for details.)
    if(!pbLaserConnector.connectLasers())
    {
        ArLog::log(ArLog::Terse, "Warning: Could not connect to configured lasers. ");
    }
    
    // A key handler to take input from keyboard
    ArKeyHandler keyHandler;
    Aria::setKeyHandler(&keyHandler);
    
    // Robot motion limiter actions (if obstacles are detected by sonar or laser)
    ArActionLimiterForwards pbSpeedLimiter("speed limiter near", 350, 800, 200);
    ArActionLimiterForwards pbSpeedLimiterFar("speed limiter far", 400, 1250, 300);
    ArActionLimiterBackwards pbBackwardsLimiter;
    ArActionConstantVelocity pbStop("stop", 0);
 
    // The marker following action
    PowerBotMarkerChase pbMarkerChaser;
    
    // Keyboard Teleoperation action
    ArActionKeydrive keydriveAction;
    
    // Use the "a" key to activate/deactivate keydrive mode
    keyHandler.addKeyHandler('a', new ArGlobalFunctor1<ArAction*>(&toggleAction, &keydriveAction));
    
    // Let Aria know about the key handler
    Aria::setKeyHandler(&keyHandler);
    
    // Add the key handler to the robot
    pbRobot.attachKeyHandler(&keyHandler);
    
    // Lock The Robot Instance
    pbRobot.lock();
    
    // Add the sonar to the robot
    pbRobot.addRangeDevice(&pbSonar);
  
    // Artificially keep the robot from going too fast
    pbRobot.setAbsoluteMaxTransVel(400);
    
    // Enable the motors
    pbRobot.comInt(ArCommands::ENABLE, 1);
    
    // Turn off the amigobot sounds
    pbRobot.comInt(ArCommands::SOUNDTOG, 0);
    
    // Add the actions to the robot in descending order of importance.
    pbRobot.addAction(&pbSpeedLimiter, 7);
    pbRobot.addAction(&pbSpeedLimiterFar, 6);
    pbRobot.addAction(&pbBackwardsLimiter, 5);
    pbRobot.addAction(&keydriveAction, 4);
    pbRobot.addAction(&pbMarkerChaser, 3);
    pbRobot.addAction(&pbStop, 1);
    
    // Unlock The Robot Instance
    pbRobot.unlock();
    
    // Start with keydrive action disabled. Use the 'a' key to turn it on.
    keydriveAction.deactivate();
    
    // Run the robot processing cycle until the connection is lost
    ArLog::log(ArLog::Normal, "Running. Send Marker data on UDP port 3251, or use 'a' key to switch to keyboard driving mode.");
    
    // Suspend this thread until the robot thread has finished
    pbRobot.waitForRunExit();
    
    Aria::exit(0);
    return 0;
}
