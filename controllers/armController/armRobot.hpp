#pragma once
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/PositionSensor.hpp>
#include<iostream>
#include<string>
#include<vector>
#include<array>
#include<cmath>
//importation des differentes bibliotheques necessaires

using namespace std;
using namespace webots;
//utilisation des namespace pour raccourcire le code


//creation de la class ArmRobot
class ArmRobot{
  public:
    ArmRobot(Robot *robot, vector<string> &motorName, vector<string> &grapMotorName, vector<string> &posSensName,vector<string> &touchSensName);
    void setArmPos(const array<double, 3> &pos);
    void setArmPos(const double &x, const double &y, const double &z);
    int grabIt();
    int letIt();
    bool moveObject(const array<double,3> &posDepart, const array<double,3> &posArrive);
    bool moveObject(const vector<array<double, 3>> &posDepart, const array<double,3> &posArrive);
    bool moveObject(const vector<array<double, 3>> &posDepart, const vector<array<double, 3>> &posArrive);
    bool initPos();
    
    
  private:
    //variable
    int state=0;
    int m_move=0;
    int m_stepAngle=0;
    bool m_initPos=false;
    const double m_delta = 0.00001;
    array<double,3> m_angle={0.0,0.0,0.0};
    array<double,3> theta={0.0,0.0,0.0};
    array<double,3> a={0.1,0.17,0.27};
    array<double,3> r={0.0,0.0,0.0};
    array<double,3> phi={0.0,0.0,0.0};
    vector<string> m_motorName, m_grapMotorName, m_posSensName, m_touchSensName;
    Robot *m_robot;
    vector<Motor*> m_armMotors;
    vector<Motor*> m_grapMotors;
    vector<TouchSensor*> m_touchSens;
    vector<PositionSensor*> m_posSens;
    //methode
    void init(double pos=0.0, double velocity=1.0, int TIMESTEP=64);
    void initMotor(double pos=0.0, double velocity=1.0);
    void initPosSensor(int TIMESTEP=64);
    void initTouchSensor(int TIMESTEP=64);
    void computAngle(const double &x, const double &y, const double &z);
    //void setMotorAngle(array<double,3> &angle);
    bool setMotorAngle(array<double,3> &angle);
    bool verifyArmPos(const array<double, 3> &pos);
    bool verifyArmAngle(const array<double, 3> &angle);
    bool verifyOneAngle(PositionSensor const *posSens, double const &angle);
        

  


};