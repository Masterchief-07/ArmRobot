#include "armRobot.hpp"

ArmRobot::ArmRobot(Robot *robot, vector<string> &motorName, vector<string> &grapMotorName, vector<string> &posSensName,vector<string> &touchSensName): m_motorName(motorName), m_grapMotorName(grapMotorName), m_posSensName(posSensName), m_touchSensName(touchSensName){
    m_robot = robot;
    init();
}

//initialisation du robot;
void ArmRobot::init(double pos, double velocity, int TIMESTEP){
    initMotor(pos, velocity);
    initPosSensor(TIMESTEP);
    initTouchSensor(TIMESTEP);
}

//initialisation des differents moteurs
void ArmRobot::initMotor(double pos, double velocity){
    //moteur du bras
    for (auto &v:m_motorName){
        m_armMotors.push_back(m_robot->getMotor(v));
        (m_armMotors.back())->setPosition(pos);
        (m_armMotors.back())->setVelocity(velocity);
        //cout<<v<<endl;
    }
    m_armMotors[1]->setPosition(1.5807);
    //moteur de la main
    for (auto &v: m_grapMotorName){
        m_grapMotors.push_back(m_robot->getMotor(v));
        (m_grapMotors.back())->setPosition(INFINITY);
        (m_grapMotors.back())->setVelocity(0.0);
        //cout<<v<<endl;
    }
}

//initialisation des capteurs de positions
void ArmRobot::initPosSensor(int TIMESTEP){
    for (auto &v:m_posSensName){
        m_posSens.push_back(m_robot->getPositionSensor(v));
        m_posSens.back()->enable(TIMESTEP);
        //cout<<v<<endl;
    }

}

//initiatisation des capteurs de toucher
void ArmRobot::initTouchSensor(int TIMESTEP){
    for (auto &v:m_touchSensName){
        m_touchSens.push_back(m_robot->getTouchSensor(v));
        m_touchSens.back()->enable(TIMESTEP);
        //cout<<v<<endl;
    }

}

//deplace le robot
void ArmRobot::setArmPos(const double &x, const double &y, const double &z){
    computAngle(x, y,z);
    cout<<m_angle[0]<<"   "<<m_angle[1]<<"   "<<m_angle[2]<<"   "<<endl;
    setMotorAngle(m_angle);
}

void ArmRobot::setArmPos(const array<double, 3> &pos){
    setArmPos(pos[0],pos[1],pos[2]);
}

//modifier l'angle des moteurs du robot
/*void ArmRobot::setMotorAngle(array<double,3> &angle){
    for (int i=0; i<angle.size();i++){
        m_armMotors[i]->setPosition(angle[i]);
    } 
}*/
bool ArmRobot::setMotorAngle(array<double,3> &angle){
    //int test=0;
    /*switch (m_stepAngle)
    {
        case 0:
            m_armMotors[1]->setPosition(angle[1]);
            m_armMotors[2]->setPosition(angle[2]);
            m_stepAngle = (verifyOneAngle(m_posSens[1],angle[1]) && verifyOneAngle(m_posSens[2],angle[2])) ? 1:0;
            break;

        case 1:
            m_armMotors[0]->setPosition(angle[0]);
            m_stepAngle = verifyOneAngle(m_posSens[0],angle[0]) ? 2:1;
            break;

        default:
            m_stepAngle=0;
            return true;
            break;
    }*/
    for (int i=0; i<angle.size();i++){
        m_armMotors[i]->setPosition(angle[i]);
    }
    return verifyArmAngle(angle);
}

//attraper un objet
int ArmRobot::grabIt(){
    cout<<m_posSens[3]->getValue()<<endl;
    if (m_posSens[3]->getValue()<=-0.02 || m_posSens[4]->getValue()>=0.02){
        m_grapMotors[0]->setVelocity(0.0);
        m_grapMotors[1]->setVelocity(0.0);
        return -1; //i nya rien a attraper
    }
    if (m_touchSens[0]->getValue()==0.0 || m_touchSens[1]->getValue()==0.0){
        m_grapMotors[0]->setVelocity(-0.01);
        m_grapMotors[1]->setVelocity(0.01);
        return 0; //en cours d'execution
    }
    else{
        m_grapMotors[0]->setVelocity(-0.001);
        m_grapMotors[1]->setVelocity(0.001);
        return 1; //il a attraper
    } 
}

//relacher un objet
int ArmRobot::letIt(){
    if (m_posSens[3]->getValue()<0 || m_posSens[4]->getValue()>0){
        m_grapMotors[0]->setPosition(0);
        m_grapMotors[0]->setVelocity(0.1);
        m_grapMotors[1]->setPosition(0);
        m_grapMotors[1]->setVelocity(0.1);
        return 0;
    }
    else{
        m_grapMotors[0]->setPosition(INFINITY);
        m_grapMotors[0]->setVelocity(0.0);
        m_grapMotors[1]->setPosition(INFINITY);
        m_grapMotors[1]->setVelocity(0.0);
        return 1;
    }

}


//verifier la possition
bool ArmRobot::verifyArmPos(const array<double, 3> &pos){
    computAngle(pos[0], pos[1], pos[2]);
    cout<<"pos: "<<"    "<<m_posSens[0]->getValue()<<"  "<<m_posSens[1]->getValue()<<"  "<<m_posSens[2]->getValue()<<"  ";
    //bool verify = m_posSens[0]->getValue() <= m_angle[0]+0.001 && m_posSens[1]->getValue() <= m_angle[1]+0.001 && m_posSens[2]->getValue() <= m_angle[2]+0.001;
    //cout<<verify<<endl;
    return verifyArmAngle(m_angle);
}

bool ArmRobot::verifyArmAngle(const array<double, 3> &angle){
    int test=0;
    for (int i=0; i<angle.size();i++){
        //if (m_posSens[i]->getValue() <= (angle[i]+m_delta) && m_posSens[i]->getValue() >= (angle[i]-m_delta)){
        if (verifyOneAngle(m_posSens[i], angle[i])){
            test+=1;}
    }
    return test==3;
}

//test un seul angle
bool ArmRobot::verifyOneAngle(PositionSensor const *posSens, double const &angle){
    return (posSens->getValue() <= angle+m_delta) && (posSens->getValue() >= angle-m_delta);
}

//calcule les angles pour positioner le robot
void ArmRobot::computAngle(const double &x, const double &y, const double &z){
    double modT = sqrt(pow(x,2)+pow(z,2));
    theta[0] = acos((x)/(modT));
    //theta[0] = -atan(z/x);
    m_angle[0] = z>0? -theta[0]:theta[0];
    r[0] = sqrt((pow(x,2)+pow(z,2)));
    r[1] = y - a[0];
    r[2] = sqrt((pow(r[0],2)+pow(r[1],2)));
    phi[1] = -atan(r[1]/r[0]);
    phi[0] = acos((pow(a[2],2)-pow(a[1],2)-pow(r[2],2))/(-2*a[1]*r[2]));
    theta[1] = phi[1] - phi[0];
    phi[2] = acos((pow(r[2],2)-pow(a[1],2)-pow(a[2],2))/(-2*a[1]*a[2]));
    theta[2] = 3.14159-phi[2];
    //angle1 = -theta[0];
    m_angle[1] = -theta[1];
    m_angle[2] = -theta[2];
}

bool ArmRobot::initPos(){
    array<double, 3> angle = {0.0, 1.5807, -1.5807}; //attak angle
    setMotorAngle(angle);
    m_initPos = verifyArmAngle(angle)? true:m_initPos;
    return m_initPos;
}

//deplace un objet
bool ArmRobot::moveObject(const array<double,3> &posDepart, const array<double,3> &posArrive){
    array<double, 3> angle = {m_angle[0], 1.5807, -1.5807}; //attak angle
    switch (state)
    {
        case 0:
            computAngle(posDepart[0],posDepart[1],posDepart[2]);
            angle[0] = m_angle[0];
            state = setMotorAngle(angle) ? 1:0;
            break;
        case 1:
            state = setMotorAngle(m_angle) ? 2:1;
            break;
        case 2:
            state = (grabIt() == 1) ? 3:2;
            break;

        case 3:
            state = setMotorAngle(angle) ? 4:3;
            break;
        
        case 4:
            setArmPos(posArrive);
            state = (verifyArmPos(posArrive)) ? 5:4;
            break;
        case 5:
            state = (letIt()==1) ? 6:5;
            break;
        default:
            state =0;
            return true;
            break;
    }
    return false;
}

bool ArmRobot::moveObject(const vector<array<double, 3>> &posDepart, const array<double,3> &posArrive){
    if (m_move < posDepart.size()){
        if (moveObject(posDepart[m_move], posArrive)){
            m_move++;
        }
        return false;
    }
    m_move=0;
    return true;
}

bool ArmRobot::moveObject(const vector<array<double, 3>> &posDepart, const vector<array<double, 3>> &posArrive){
    if (m_move < posDepart.size()){
        if (moveObject(posDepart[m_move], posArrive[m_move])){
            m_move++;
        }
        return false;
    }
    m_move=0;
    return true;
}