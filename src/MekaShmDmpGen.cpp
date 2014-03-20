/*
M3 -- Meka Robotics Real-Time Control System
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <rtai_sched.h>
#include <stdio.h>
#include <signal.h>
#include <rtai_shm.h>
#include <rtai.h>
#include <rtai_sem.h>
//#include <m3rt/base/m3ec_def.h>
#include <m3rt/base/m3rt_def.h>
#include <rtai_nam2num.h>
#include <rtai_registry.h>
#include <eigen3/Eigen/Core>
#include "m3/robots/humanoid_shm_sds.h"

//////////////////////////////////// DMP //////////////////////////////////////
#include "dmp/Dmp.hpp"
#include "dmp/Trajectory.hpp"
#include "dynamicalsystems/DynamicalSystem.hpp"
#include "dynamicalsystems/ExponentialSystem.hpp"
#include "dynamicalsystems/SigmoidSystem.hpp"
#include "dynamicalsystems/TimeSystem.hpp"
#include "dynamicalsystems/SpringDamperSystem.hpp"
#include "functionapproximators/FunctionApproximatorLWR.hpp"
#include "functionapproximators/MetaParametersLWR.hpp"
#include "functionapproximators/ModelParametersLWR.hpp"

//////////////////////////////////// RT VARS //////////////////////////////////////
//#define RT_TIMER_TICKS_NS_BOT_SHM (10000000) //Period of rt-timer expressed in ns 10000000/1e9 = 0.01s
#define NANO2SEC(a)	a/1e9
#define SEC2NANO(a)	a*1e9
#define BOT_SHM "TSHMM"
#define BOT_CMD_SEM "TSHMC"
#define BOT_STATUS_SEM "TSHMS"
//#define CLOSED_LOOP
//static M3HumanoidShmSdsCommand cmd;
//static M3HumanoidShmSdsStatus status;
//static int sds_status_size = sizeof(M3HumanoidShmSdsStatus);;
//static int sds_cmd_size = sizeof(M3HumanoidShmSdsCommand);
//static int sys_thread_active = 0;
static int stop_thread = 0;
static void end_thread(int dummy) { stop_thread = 1; }

#include <iostream>
#include <fstream> 
#include <iterator>

using namespace Eigen;
using namespace DmpBbo;

using namespace std;
void WriteTxtFile( const char* filename,std::vector<std::vector<double> >& values ) {
    ofstream myfile (filename);
    size_t row = 0;
    size_t col = 0;
    size_t nb_rows = values.size();
    size_t nb_cols = values[0].size();
    if (myfile.is_open())
    {
        while(row < nb_rows) {
	    while(col < nb_cols){
		if (col == nb_cols-1)
			myfile << values[row][col] << "\n";
		else
			myfile << values[row][col] << " ";
		col++;  
	 }
	    col=0;
            row++;
        }
        myfile.close();
	std::cout << "File ["<<filename<<"] write with success  ["<<nb_rows<<" rows, "<<nb_cols<<" cols] "<<std::endl;
    }
    else{
	 std::cout << "Unable to open file : ["<<filename<<"]"<<std::endl;
	 getchar();
    }
    return;
}

bool getSemAddr(const char* sem_name,SEM* &sem){
	sem = (SEM*)rt_get_adr(nam2num(sem_name));
	if (!sem)
		return false;
	return true;
}

class M3ShmManager
{
	public:
		M3ShmManager(int Ndof):Ndof_(Ndof),sds_status_size(sizeof(M3HumanoidShmSdsStatus)),sds_cmd_size(sizeof(M3HumanoidShmSdsCommand)){
			memset(&cmd, 0, sds_cmd_size); // Initialize cmd
			memset(&status, 0, sds_status_size); // Initialize status
		}
		
		~M3ShmManager(){
			delete status_sem;
			delete command_sem;
			//delete m3_sds;
			//rt_sem_delete(command_sem);
			//rt_sem_delete(status_sem);
			rt_shm_free(nam2num(bot_shm_name_.c_str()));
		}
		
		M3HumanoidShmSdsCommand cmd;
		M3HumanoidShmSdsStatus status;
		int sds_status_size;
		int sds_cmd_size;
		SEM* status_sem;
		SEM* command_sem;
		M3Sds* m3_sds;
		std::string bot_shm_name_;
		int Ndof_;
		
		bool statusSemInit(std::string sem_name){
			if(getSemAddr(sem_name.c_str(),status_sem))
				return true;
			else
				return false;
		}
		bool commandSemInit(std::string sem_name){
			if(getSemAddr(sem_name.c_str(),command_sem))
				return true;
			else
				return false;
		}
		
		////////////////////////// COMMAND /////////////////////////////
		void stepCommand(VectorXd& joints_cmd) // FIX: No checks now
		{	
			SetTimestamp(GetTimestamp()); //Pass back timestamp as a heartbeat
			for (int i = 0; i < Ndof_; i++) // FIX
			{
				cmd.right_arm.ctrl_mode[i] = JOINT_ARRAY_MODE_THETA_GC;
				cmd.right_arm.q_desired[i] = (mReal)joints_cmd[i];
				cmd.right_arm.tq_desired[i] = 100.0;
				cmd.right_arm.slew_rate_q_desired[i] = 1000; // 10.0
				cmd.right_arm.q_stiffness[i] = 1.0;
			}
			// Lock the semaphore and copy the output data
			rt_sem_wait(command_sem);
			memcpy(this->m3_sds->cmd, &cmd, sds_cmd_size); // This is failing somehow
			rt_sem_signal(command_sem);
		}

		////////////////////////// STATUS /////////////////////////////
		void stepStatus(VectorXd& joints_status) // FIX: No checks now
		{	
			// Lock the semaphore and copy the input data
			rt_sem_wait(status_sem);
			memcpy(&status, this->m3_sds->status, sds_status_size);
			rt_sem_signal(status_sem);
			// Convert status into Eigen vector
			for (int i = 0; i < Ndof_; i++) // FIX
			{
				joints_status[i] = DEG2RAD(status.right_arm.theta[i]); // pos
				//joints_status[i+7] = DEG2RAD(status.right_arm.thetadot[i]); // vel
			}
		}	
		
		bool m3sdsInit(std::string bot_shm_name){
			if (this->m3_sds = (M3Sds*)rt_shm_alloc(nam2num(bot_shm_name.c_str()),sizeof(M3Sds),USE_VMALLOC)){
				bot_shm_name_ = bot_shm_name; 
				return true;
			}
			else
				return false;
		}
		
		///////  Timing functions:
		void SetTimestamp(int64_t  timestamp)
		{
			cmd.timestamp = timestamp;
			return;
		}

		int64_t GetTimestamp()
		{
			return status.timestamp;
		}
};

/////// Shared data structure:
struct DmpData
{
	Dmp* dmp; 
	double dt;
	int Ndof;
	int n_time_steps_trajectory;
	~DmpData(){ delete dmp;}
};

///////  Trajectory generator:
Trajectory generateViapointTrajectory(const VectorXd& ts, const VectorXd& y_first, const VectorXd& y_last, const double& Tf, const double& Ti)
{
    //VectorXd y_first = VectorXd::Zero(n_dims);
    //VectorXd y_last  = VectorXd::Ones(n_dims) * 0.3;
    
    assert(y_first.size() == y_last.size());
    
    int n_dims = y_first.size();
    double viapoint_time = (Tf -Ti)/2;

    VectorXd y_yd_ydd_viapoint = VectorXd::Zero(3*n_dims);
    
    for(int i = 0; i<n_dims; i++)
	    y_yd_ydd_viapoint[i] = 0.5;//(y_last[i] - y_first[i])/2;
    
    return  Trajectory::generatePolynomialTrajectoryThroughViapoint(ts,y_first,y_yd_ydd_viapoint,viapoint_time,y_last);
}

Dmp* generateDemoDmp(double dt, int Ndof, double Ti, double Tf, int& n_time_steps_trajectory){

	// GENERATE A TRAJECTORY
	n_time_steps_trajectory = (int)((Tf-Ti)/dt) + 1;

	// Some default values for dynamical system
	VectorXd y_init(Ndof); 
	y_init   << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	VectorXd y_attr(Ndof);
	y_attr << 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4;
	
	VectorXd ts = VectorXd::LinSpaced(n_time_steps_trajectory,Ti,Tf); // From Ti to Tf in n_time_steps_trajectory steps
	
	Trajectory trajectory = generateViapointTrajectory(ts, y_init, y_attr, Tf, Ti);
  
        // MAKE THE FUNCTION APPROXIMATORS
        // Initialize some meta parameters for training LWR function approximator
        int n_basis_functions = 25;
        int input_dim = 1;
        double overlap = 0.01;
        MetaParametersLWR* meta_parameters = new MetaParametersLWR(input_dim,n_basis_functions,overlap);
        FunctionApproximatorLWR* fa_lwr = new FunctionApproximatorLWR(meta_parameters);

	//Dmp::DmpType dmp_type = Dmp::KULVICIUS_2012_JOINING;
	Dmp::DmpType dmp_type = Dmp::IJSPEERT_2002_MOVEMENT;
	
	std::vector<FunctionApproximator*> function_approximators(Ndof);    	
	for (int dd=0; dd<Ndof; dd++)
		function_approximators[dd] = fa_lwr->clone();
	
	  /*Dmp(double tau, Eigen::VectorXd y_init, Eigen::VectorXd y_attr, std::vector<FunctionApproximator*> function_approximators, DmpType dmp_type=KULVICIUS_2012_JOINING);*/
	
	Dmp* dmp = new Dmp(Tf,y_init,y_attr,function_approximators,dmp_type);
	
	dmp->train(trajectory);
	
	return dmp;  
}

void* dmpLoop(void* args){
	
	// Retrain the dmp_data
	DmpData* dmp_data = (DmpData*) args;
	
	// Variables used to print on a txt file
	std::vector<double> curr_x_cmd;
	std::vector<std::vector<double> > out_x_cmd;
	curr_x_cmd.resize(dmp_data->Ndof*2); // Store the velocities too
	std::vector<double> curr_x_status;
	std::vector<std::vector<double> > out_x_status;
	curr_x_status.resize(dmp_data->Ndof*2);
	
	// Create the dmp task
	RT_TASK* dmp_task;
	//Args: Name, Priority, Stack Size, max_msg_size, Policy, cpus_allowed
	if (!(dmp_task = rt_task_init_schmod(nam2num( "DMP" ),0,0,0,SCHED_FIFO,0xF)))
	{
		printf("ERROR: Cannot initialize dmp task\n");
		return 0;
	}
	rt_allow_nonroot_hrt();
	
	static M3ShmManager shm_manager(dmp_data->Ndof);
	if(!shm_manager.m3sdsInit(BOT_SHM)){
		printf("Unable to find the %s shared memory.\n","TSHMM");
		rt_task_delete(dmp_task);
		return 0;
	}
	if(!shm_manager.statusSemInit(BOT_STATUS_SEM)){
		printf("Unable to find the %s semaphore.\n","TSHMS");
		rt_task_delete(dmp_task);
		return 0;
	}
	if(!shm_manager.commandSemInit(BOT_CMD_SEM)){
		printf("Unable to find the %s semaphore.\n","TSHMC");
		rt_task_delete(dmp_task);
		return 0;
	}
	
	RTIME tick_period = nano2count(SEC2NANO(dmp_data->dt)); // This is ~=dt
	
	int dmp_dim = dmp_data->dmp->dim();
	static VectorXd x_dummy(dmp_dim),x_cmd(dmp_dim),xd_cmd(dmp_dim),x_status(dmp_dim),xd_status(dmp_dim);
	
	// Start the real time task
	printf("Starting real-time task\n");
	rt_task_make_periodic(dmp_task, rt_get_time() + tick_period, tick_period);
	mlockall(MCL_CURRENT | MCL_FUTURE); // Prevent memory swaps
	rt_make_hard_real_time();
	
	// Set the initial conditions
        dmp_data->dmp->integrateStart(x_status,xd_status);
	
	// RT Loop
	long long start_time, end_time, curr_dt, elapsed_time;
	start_time = nano2count(rt_get_cpu_time_ns());
	int loop_cntr = 0;
	while(!stop_thread && loop_cntr <= 10*dmp_data->n_time_steps_trajectory)
	{
		// Read the motors state
#ifdef CLOSED_LOOP	
		shm_manager.stepStatus(x_status);
		x_dummy = x_status;
#else
		shm_manager.stepStatus(x_dummy);
#endif
		// Save to std vectors (to write into a txt file)
		for (int i = 0; i < curr_x_status.size(); i++){
			curr_x_status[i] = x_dummy[i];
			curr_x_cmd[i] = x_cmd[i];
		}
		out_x_status.push_back(curr_x_status);
		out_x_cmd.push_back(curr_x_cmd);
		
		//start_time = nano2count(rt_get_cpu_time_ns());
		
		// Integrate dmp
               dmp_data->dmp->integrateStep(dmp_data->dt/10,x_status,x_cmd,xd_cmd);
	       
	        std::cout << "************" << std::endl;
		std::cout << "*** x_status ***" << std::endl;
		std::cout << x_status << std::endl;
		std::cout << "*** x_cmd ***" << std::endl;
		std::cout << x_cmd << std::endl;
		
		x_status = x_cmd;

	       // Write the motors state
	       shm_manager.stepCommand(x_cmd);
		
		// And waits until the end of the period.
		rt_task_wait_period();
		
		//end_time = nano2count(rt_get_cpu_time_ns());
		//curr_dt = end_time - start_time;
		loop_cntr++;
	}
	end_time = nano2count(rt_get_cpu_time_ns());
	elapsed_time = end_time - start_time;
	printf("Elapsed time: %f s\n",NANO2SEC((double)count2nano(elapsed_time)));
	
	// Write to file
	WriteTxtFile("output_status_closed_loop_vel.txt",out_x_status);
	WriteTxtFile("output_cmd_closed_loop_vel.txt",out_x_cmd);
	
	// Task terminated
	printf("Removing real-time task\n");
	rt_task_delete(dmp_task);
	return 0;
}	


int main(int argc, char *argv[])
{
	signal(SIGINT, end_thread);
	
	// Generate a demo dmp and the shared data structure
	DmpData* dmp_data = new DmpData;
	double Tf = 3; //sec
	double Ti = 0.0;
	dmp_data->dt = 0.0250;
	dmp_data->Ndof = 7;
	dmp_data->dmp = generateDemoDmp(dmp_data->dt,dmp_data->Ndof,Ti,Tf,dmp_data->n_time_steps_trajectory);
	
	// Create a thread
	rt_allow_nonroot_hrt(); // It is necessary to spawn tasks
	int thread_id = rt_thread_create((void*)dmpLoop,dmp_data,10000);
	
	// Join with the thread
	rt_thread_join(thread_id);
	
	// Destroy the dmp_data
	delete dmp_data;
}


