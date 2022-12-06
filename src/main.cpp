#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/PhysicsClientSharedMemory_C_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "LinearMath/btMatrixX.h"
#include "BulletInverseDynamics/IDConfig.hpp"
#include "BulletInverseDynamics/IDMath.hpp"
#include "BulletInverseDynamics/MultiBodyTree.hpp"
#include <thread>

#include "Bullet3Common/b3HashMap.h"
#include "Bullet3Common/b3Random.h"

#include <Eigen/Dense>
#include "Utils/b3Clock.h"
#include "../lib/ModernRobotics.h"

#include <vector>
#include <iostream>


#include "Indy7.h"

#include "serialPort/SerialPort.h"
#include <csignal>


using namespace std;
using namespace Eigen;
using namespace mr;

extern const int CONTROL_RATE;
const int CONTROL_RATE = 2000;
const b3Scalar FIXED_TIMESTEP = 1.0 / ((b3Scalar)CONTROL_RATE);
b3SharedMemoryCommandHandle command;
int statusType, ret;

VectorXd motor_q(2);
VectorXd motor_torque(2);

void func1(){
    cout<<"Func 1"<<endl;
     SerialPort serial("/dev/ttyUSB0",78,4800000,20000);
  // send message struct
    MOTOR_send motor_run1, motor_stop1;
    MOTOR_send motor_run2, motor_stop2;
    // receive message struct
    MOTOR_recv motor_r1;
    MOTOR_recv motor_r2;    
    motor_torque[0] = 0.0; 
    motor_torque[1] = 0.0;
    // set the id of motor
    motor_run1.id = 0;
    motor_run1.motorType = MotorType::A1;
    motor_run1.mode = 10;
    motor_run1.T = 0.0;
    motor_run1.W = 0.0;
    motor_run1.Pos = 0.0;
    motor_run1.K_P = 0.0;
    motor_run1.K_W = 0.0;

    motor_run2.id = 1;
    motor_run2.motorType = MotorType::A1;
    motor_run2.mode = 10;
    motor_run2.T = 0.0;
    motor_run2.W = 0.0;
    motor_run2.Pos = 0.0;
    motor_run2.K_P = 0.0;
    motor_run2.K_W = 0.0;

    motor_stop1.id = motor_run1.id;
    motor_stop1.motorType = motor_run1.motorType;
    motor_stop1.mode = 0;

    motor_stop2.id = motor_run2.id;
    motor_stop2.motorType = motor_run2.motorType;
    motor_stop2.mode = 0;
    
    motor_r1.motorType = motor_run1.motorType;

    // encode data into motor commands
    modify_data(&motor_run1);
    modify_data(&motor_stop1);
    
    modify_data(&motor_run2);
    modify_data(&motor_stop2);

    while(1){
        motor_run1.T = motor_torque[0];
        motor_run2.T = motor_torque[1];    
	modify_data(&motor_run1);     
	modify_data(&motor_run2);           
        serial.sendRecv(&motor_run1, &motor_r1);
        serial.sendRecv(&motor_run2, &motor_r2);        
        // decode data from motor states
        extract_data(&motor_r1);
        extract_data(&motor_r2);        
//        std::cout << "Pos: " << motor_r.Pos << std::endl;
        motor_q[0] = motor_r1.Pos/9.1;
        motor_q[1] = motor_r2.Pos/9.1;        
        usleep(20000);
    }

}
int main()
{


   

    std::thread t1 = std::thread(func1);


    b3PhysicsClientHandle client = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
    if (!b3CanSubmitCommand(client))
    {
        printf("Not connected, start a PyBullet server first, using python -m pybullet_utils.runServer\n");
        exit(0);
    }
    b3RobotSimulatorClientAPI_InternalData data;
    data.m_physicsClientHandle = client;
    data.m_guiHelper = 0;
    b3RobotSimulatorClientAPI_NoDirect sim;
    sim.setInternalData(&data);
    

    sim.resetSimulation();
    sim.setGravity( btVector3(0 , 0 ,-9.8));
    //int plane = sim.loadURDF("plane.urdf");
    int robotId = sim.loadURDF("model/a1_motor/a1_motor.urdf");    
    sim.setTimeStep(FIXED_TIMESTEP);
    double t = 0;

    b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
    for (int i = 1; i<3;i++){
        controlArgs.m_maxTorqueValue  =0.0;
        sim.setJointMotorControl(robotId,i,controlArgs);
    }   
    VectorXd q(2);
    VectorXd qdot(2);
    VectorXd ddq(2);
    VectorXd prev_q(2);
    prev_q[0] = 0;
    prev_q[1] = 0;    
    ddq[0] = 0;
    ddq[1] = 0;
    
    VectorXd joint_forces(2);

    while(1){

    b3JointSensorState jointStates;
    int numJoints = sim.getNumJoints(robotId);

    cout<<motor_torque<<endl;
    /*
    for (int i = 1; i < 3; i++)
    {
        if(sim.getJointState(robotId,i, &jointStates)){
            q[i-1] = jointStates.m_jointPosition;
            qdot[i-1] = jointStates.m_jointVelocity;
        }

    }
    */
    qdot[0] = (prev_q[0] - motor_q[0])/FIXED_TIMESTEP;
    qdot[1] = (prev_q[1] - motor_q[1])/FIXED_TIMESTEP;
    prev_q[0] = motor_q[0];
    prev_q[1] = motor_q[1];    

	bool ret =  sim.calculateInverseDynamics(robotId,motor_q.data(), qdot.data(), ddq.data(), motor_torque.data());	
	
     
    b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_TORQUE);
   
    for (int i = 1; i<3;i++){
        controlArgs.m_maxTorqueValue  =motor_torque[i-1];
        sim.setJointMotorControl(robotId,i,controlArgs);
        sim.resetJointState(robotId, i, motor_q[i-1]);
    }   
    
    /*
    b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_PD);
    for (int i = 1; i<3;i++){
        controlArgs.m_maxTorqueValue  =6.0;
        controlArgs.m_targetPosition  =motor_q[i-1];
        controlArgs.m_kp = 10.0;
        controlArgs.m_kd = 1.0;        
        
        sim.setJointMotorControl(robotId,i,controlArgs);
    } 
    */
	//indy7.setTorques(&sim,torque,MAX_TORQUES);
	sim.stepSimulation();	
	b3Clock::usleep(1000. * 1000. * FIXED_TIMESTEP);
	t = t+FIXED_TIMESTEP;	
    }

    
    
    
}
