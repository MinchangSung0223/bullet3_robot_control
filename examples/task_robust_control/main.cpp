#include "main.h"
#include <math.h>

#include "spdlog/spdlog.h"

b3RobotSimulatorClientAPI *sim;
void print_JVec(JVec val,char *str){
    spdlog::info("{:s} : {:03.8f}\t{:03.8f}\t{:03.8f}\t{:03.8f}\t{:03.8f}\t{:03.8f}",str,val(0),val(1),val(2),val(3),val(4),val(5));
}
void print_Vector6d(Vector6d val,char *str){
    spdlog::info("{:s} : {:03.8f}\t{:03.8f}\t{:03.8f}\t{:03.8f}\t{:03.8f}\t{:03.8f}",str,val(0),val(1),val(2),val(3),val(4),val(5));
}
void saturate_Vector6d(Vector6d& val,double max_val){
		for(int i =0;i<6;i++){
			if(std::isnan(val(i))){
				if(val(i)>0) val(i) = max_val;
				else val(i) = -max_val;
			}else{
				if(val(i)>0){
					if(abs(val(i))>max_val) val(i) = max_val;

				}else{
					if(abs(val(i))>max_val) val(i) = -max_val;
				}
			}
		}
		
}
int main(void){
    struct timespec next_period;
	clock_gettime(CLOCK_MONOTONIC, &next_period);
	//Simulation Setup
	sim = new b3RobotSimulatorClientAPI();
	bool isConnected = sim->connect(eCONNECT_GUI);
	sim_setup(sim);
	//Load URDF
	const char* urdfFilePath = "../urdf/indy7/indy7.urdf";
	const char* urdfFilePath_visual = "../urdf/indy7/indy7_visual.urdf";
	b3RobotSimulatorLoadUrdfFileArgs urdf_args;	
	int robotId_visual = sim->loadURDF(urdfFilePath_visual,urdf_args);	
	urdf_args.m_flags |= URDF_USE_INERTIA_FROM_FILE;
	urdf_args.m_flags |= URDF_USE_SELF_COLLISION;    
	int robotId = sim->loadURDF(urdfFilePath,urdf_args);	
	
	
	//SimRobot Setup
	JVec q_start;
	q_start<<0.0,0.0,-1.5708,0.0,-1.5708,0.0;
	Robot robot = Robot(sim,robotId, q_start);
	Robot robot_des = Robot(sim,robotId_visual,q_start);
	robot.draw_eef_T(0.1, 2);
	//robot_des.draw_eef_T(0.2, 5);
	LR_Control control;
	control.LRSetup(urdfFilePath);	
	bool is_run = 1;
	double dt = fixedTimeStep;
	//Simulation Loop
	JVec q,q_dot,e_int,e_dot,q_des,q_dot_des,q_ddot_des;	
	JVec max_torques;
	max_torques << 431.97,431.97,197.23,79.79,79.79,79.79;
	q_des = q_start;
	//Gain Setup
	Matrix6d Task_Kp=Matrix6d::Identity()*1000.0;
	Matrix6d Task_Kv=Matrix6d::Identity()*200.0;
	MatrixNd Hinf_K_gamma=Matrix6d::Identity();
	setHinfGain(Hinf_K_gamma);
	SE3 T_des = lr::FKinBody(control.M,control.Blist,q_start);
	SE3 T_start = lr::FKinBody(control.M,control.Blist,q_start);
	Vector6d disp;
	disp<<-0.1,0.0,0.0,0.0,0.0,0.0;
	SE3 T_end = T_start*lr::MatrixExp6(lr::VecTose3(disp));
	Vector6d V_des =Vector6d::Zero();
	Vector6d V_dot_des =Vector6d::Zero();
	Vector6d lambda_int =Vector6d::Zero();
	Vector6d V_start = Vector6d::Zero();
	Vector6d V_end = Vector6d::Zero();
	Vector6d V_dot_start = Vector6d::Zero();
	Vector6d V_dot_end = Vector6d::Zero();
	double Tf = 10.0;
	gt = 0.0;
	std::cout<<"START SIMULATION"<<std::endl;
	int print_cnt = 0;
	JVec q_ddot = JVec::Zero();

    
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
		// Trajectory
		LieScrewTrajectory(T_start,T_end,V_start,V_end,V_dot_start,V_dot_end,Tf,gt,T_des,V_des,V_dot_des);	
		// Controller
		SE3 T = lr::FKinBody(control.M,control.Blist,q);
		Jacobian Jb = lr::JacobianBody(control.Blist,q);
		Jacobian Jb_dot = lr::dJacobianBody(control.M,control.Blist,q,q_dot);
		SE3 T_err = lr::TransInv(T)*T_des;
		SE3 invT_err = TransInv(T_err);
		Vector6d V = Jb*q_dot;
		Vector6d V_err = V_des-Ad(invT_err)*V;
		Vector6d lambda = lr::se3ToVec(lr::MatrixLog6(T_err));
		saturate_Vector6d(V_err,0.001);
		saturate_Vector6d(lambda,0.001);
		Vector6d lambda_dot = dlog6(-lambda)*V_err;
		Vector6d V_dot = Jb_dot*q_dot + Jb*q_ddot;
		Vector6d V_dot_err = V_dot_des-Ad(invT_err)*V_dot+ad(V_err)*V_dot;
		Vector6d lambda_dot_ref =Task_Kv*lambda+Task_Kp*lambda_int; 
		Vector6d lambda_ddot_ref=Task_Kv*lambda_dot +Task_Kp*lambda;
		Vector6d V_ref = Ad(T_err)*(V_des + dexp6(-lambda)*(lambda_dot_ref));
		Vector6d V_dot_ref = Ad(T_err)*(V_dot_des + (dexp6(-lambda)*lambda_ddot_ref) + ad(V_err)*V_dot - (ddexp6(-lambda,-lambda_dot)*lambda_dot));
		pinvJacobian pinvJb = Jb.transpose()*(Jb*Jb.transpose()).inverse();
		JVec q_ddot_ref = pinvJb*(V_dot_ref-Jb_dot*q_dot);
		q_ddot = q_ddot_ref;
		JVec q_dot_ref = pinvJb*V_ref;
		JVec e_dot = q_dot_ref - q_dot;
		JVec tau_ref =Hinf_K_gamma*e_dot;
		MassMat M = control.MassMatrix(q);
		MatrixNd C = control.CoriolisMatrix(q,q_dot);    
		JVec G = control.GravityForces(q);	
		JVec tau = M*q_ddot_ref +C*q_dot_ref +G+tau_ref;
		lambda_int += lambda*dt;
		// Set torques
		//spdlog::info("gt : {:03.3f}",gt);
		//std::cout<<lambda<<std::endl;
		print_Vector6d(lambda,"lambda");
		bool ret = lr::IKinBody(control.Blist, control.M, T_des,q_des, 0.01, 0.001) ;
		robot_des.reset_q(q_des);
		robot.set_torques(tau,max_torques);
		sim->stepSimulation();
		gt+=dt;
		//Time sleep
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
	}
}
