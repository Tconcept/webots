// File:          EPuckGoForward.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/LED.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#include <math.h>
// All the webots classes are defined in the "webots" namespace
#define TIME_STEP 64

using namespace webots;


#define MAX_SPEED 6.28


// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv)
{
  // create the Robot instance.
  Robot *robot = new Robot();
  
  

  // get the time step of the current world.
  int timeStep = (int) robot->getBasicTimeStep();
  
  
   Motor *leftMotor = robot->getMotor("left wheel motor");
   Motor *rightMotor = robot->getMotor("right wheel motor");
   

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  LED *led = robot->getLED("ledname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
  
    DistanceSensor *ps[8];
    char psNames[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
    };

  for (int i = 0; i < 8; i++) {
  ps[i] = robot->getDistanceSensor(psNames[i]);
  ps[i]->enable(TIME_STEP);
  }
  
  
   
  
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);
    
    
    double psValues[8];
    for (int i = 0; i < 8 ; i++)
    psValues[i] = ps[i]->getValue();
    
    
    bool right_obstacle = psValues[0] > 70.0 || psValues[1] > 70.0 || psValues[2] > 70.0;
    bool left_obstacle = psValues[5] > 70.0 || psValues[6] > 70.0 || psValues[7] > 70.0;
    
    double leftSpeed  = 0.5 * MAX_SPEED;
    double rightSpeed = 0.5 * MAX_SPEED;
// modify speeds according to obstacles
    if (left_obstacle) {
  // turn right
    leftSpeed  += 0.5 * MAX_SPEED;
    rightSpeed -= 0.5 * MAX_SPEED;
  }
    else if (right_obstacle) {
  // turn left
  leftSpeed  -= 0.5 * MAX_SPEED;
  rightSpeed += 0.5 * MAX_SPEED;
  }
// write actuators inputs
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
 

    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  led->set(1);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
