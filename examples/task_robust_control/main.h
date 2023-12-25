#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <LR/include/LR_Control.h>
#include <SimRobot/Robot.h>
#include <RobotSimulator/b3RobotSimulatorClientAPI.h>
#define MS_TO_US 1000000
#define CYCLE_NS 1000000
double fixedTimeStep = 0.001;
double gt;
void sim_setup(b3RobotSimulatorClientAPI *sim){
	//Camera Setup
	sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
	sim->configureDebugVisualizer( COV_ENABLE_SHADOWS, 0);
	btVector3 targetPos(0,0,0);
	sim->resetDebugVisualizerCamera(3.5, -45, 135, targetPos);
	//TimeOut Setup
	sim->setTimeOut(10);
	//Physics Setup
	sim->setGravity(btVector3(0, 0, -9.8));
	b3RobotSimulatorSetPhysicsEngineParameters args;
	sim->getPhysicsEngineParameters(args);
	sim->setTimeStep(fixedTimeStep);

	const char* plane_urdf_file_path = "../../../urdf/indy7/plane.urdf";
	int planeId = sim->loadURDF(plane_urdf_file_path);	    

	//Realtime Option Setup
	sim->setRealTimeSimulation(false);
}
void setHinfGain(MatrixNd& Hinf_K_gamma){
    JVec invL2sqr,K;
    invL2sqr<<1000.0,1000.0,800.0,600.0,600.0,600.0;
    K<<50,50,30,20,20,1;
    for (int i=0; i<JOINTNUM; ++i){
        switch(i){
		case 0:
		    Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;break;
		case 1:
		    Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;break;
		case 2:
		    Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;break;
		case 3:
		    Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;break;
		case 4:
		    Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;break;
		case 5:
		    Hinf_K_gamma(i,i) = K(i)+1.0/invL2sqr(i) ;break;
        }
    }
}

