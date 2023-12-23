#include "main.h"
#include "spdlog/spdlog.h"

b3RobotSimulatorClientAPI *sim;
int main(void){
    struct timespec next_period;
	clock_gettime(CLOCK_MONOTONIC, &next_period);
	//Simulation Setup
	sim = new b3RobotSimulatorClientAPI();
	bool isConnected = sim->connect(eCONNECT_GUI);
	sim_setup(sim);
	//Load URDF
	const char* urdfFilePath = "../../../urdf/indy7/indy7.urdf";
	const char* urdfFilePath_visual = "../../../urdf/indy7/indy7_visual.urdf";
	b3RobotSimulatorLoadUrdfFileArgs urdf_args;	
	int robotId_visual = sim->loadURDF(urdfFilePath_visual,urdf_args);	
	urdf_args.m_flags |= URDF_USE_INERTIA_FROM_FILE;
	urdf_args.m_flags |= URDF_USE_SELF_COLLISION;    
	int robotId = sim->loadURDF(urdfFilePath,urdf_args);	
	
	
	//SimRobot Setup
	Robot robot = Robot(sim,robotId);
	Robot robot_des = Robot(sim,robotId_visual);
	LR_Control control;
	control.LRSetup(urdfFilePath);	
	bool is_run = 1;
	double dt = fixedTimeStep;
	//Simulation Loop
	JVec q,q_dot,e_int,e_dot,q_des,q_dot_des,q_ddot_des,q_start,q_end;	
	JVec max_torques;
	max_torques << 431.97,431.97,197.23,79.79,79.79,79.79;
	q_start<<0.0,0.0,0.0,0.0,0.0,0.0;
	q_end<<0.0,0.0,-1.5708,0.0,-1.5708,0.0;
	//Gain Setup
	MatrixNd Hinf_Kp=MatrixNd::Identity()*100.0;
	MatrixNd Hinf_Kv=MatrixNd::Identity()*20.0;
	MatrixNd Hinf_K_gamma=MatrixNd::Identity();
	setHinfGain(Hinf_K_gamma);
	
	double Tf = 10.0;
	gt = 0.0;
	std::cout<<"START SIMULATION"<<std::endl;
	int print_cnt = 0;

	while(is_run){
		//Calc time period
        next_period.tv_nsec += CYCLE_NS;
        if (next_period.tv_nsec >= 1000000000) {
            next_period.tv_nsec -= 1000000000;
            next_period.tv_sec++;
        }
		// Get robot states
		q = robot.get_q();
		q_dot = robot.get_q_dot();		
		JointTrajectory(q_start, q_end, Tf, gt , 5 , q_des,  q_dot_des, q_ddot_des);
		// Controller
		JVec e = q_des-q;
		e_dot = q_dot_des - q_dot;
		MassMat M = control.MassMatrix(q);
		MatrixNd C = control.CoriolisMatrix(q,q_dot);    
		JVec G = control.GravityForces(q);	
		JVec q_ddot_ref = q_ddot_des+Hinf_Kv*e_dot+Hinf_Kp*e;
		JVec q_dot_ref = q_dot_des+Hinf_Kv*e+Hinf_Kp*e_int;		
		JVec tau = M*q_ddot_ref+C*q_dot_ref+G+Hinf_K_gamma*(e_dot + Hinf_Kv*e + Hinf_Kp*e_int);
		e_int += e*dt;
		// Set torques
		spdlog::info("gt : {:03.3f}",gt);
		robot_des.reset_q(q_des);
		robot.set_torques(tau,max_torques);
		sim->stepSimulation();
		gt+=dt;
		//Time sleep
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
	}
}
