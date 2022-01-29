// File:          armController.cpp
// Date:
// Description:
// Author:
// Modifications:

// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/PositionSensor.hpp>
#include "armRobot.hpp"
#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#define TIMESTEP 64

using namespace webots;
using namespace std;


int main(int argc, char **argv) {
  
  vector<string> motor{"motor1", "motor2", "motor3", };
  vector<string> grapMotor{"armLeft", "armRight"};
  vector<string> touchSens{"sensLeft", "sensRight"};
  vector<string> posSens{ "posMotor1","posMotor2","posMotor3", "posArmLeft", "posArmRight"};
  Robot *robot = new Robot();
  
  ArmRobot rob(robot, motor, grapMotor, posSens, touchSens);
  
  array<double, 3> posdepart = {0.31,0.06,-0.2};
  vector <array<double, 3>> posDepart = {
    {0.31,0.06,-0.2},
    {0.31,0.06,0.2},
    {0.11,0.06,-0.2},
    {0.11,0.06,0.2},
  };
  array<double, 3> posarrive ={-0.21,0.25,0.1};
  vector <array<double, 3>> posArrive = {
    {-0.21,0.25,0.1},
    {-0.21,0.25,0.1},
    {-0.21,0.25,-0.15},
    {-0.21,0.25,-0.15},
  };
  
  //rob.getTo(pos[0],pos[1],pos[2]);
  
  int timeStep = TIMESTEP;

  while (robot->step(timeStep) != -1) {
    //cout<<rob.grabIt()<<endl;
    
    if (rob.moveObject(posDepart, posArrive)){
      break;
     }
  };

  delete robot;
  return 0;
}
