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
//#include <m3rt/base/m3rt_def.h>
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
/*static M3HumanoidShmSdsCommand cmd;
static M3HumanoidShmSdsStatus status;
static int sds_status_size;
static int sds_cmd_size;*/

using namespace Eigen;
using namespace DmpBbo;

class M3ShmManager
{
	public:
		M3ShmManager(){
			sds_status_size = sizeof(M3HumanoidShmSdsStatus);
			sds_cmd_size = sizeof(M3HumanoidShmSdsCommand);
			memset(&cmd, 0, sds_cmd_size); // Initialize cmd
		}
		
		~M3ShmManager(){
			delete status_sem;
			delete command_sem;
			delete m3_sds;
		}
		
		M3HumanoidShmSdsCommand cmd;
		M3HumanoidShmSdsStatus status;
		int sds_status_size;
		int sds_cmd_size;
		SEM* status_sem;
		SEM* command_sem;
		M3Sds* m3_sds;
		
		bool statusSemInit(std::string sem_name){
			
			if(getSemAddr(sem_name.c_str(),status_sem))
				return true;
			else
				return false;
			/*status_sem = (SEM*)rt_get_adr(nam2num(sem_name.c_str()));
			if (!status_sem)
				return false;
			return true;*/
		}
		bool commandSemInit(std::string sem_name){
			if(getSemAddr(sem_name.c_str(),command_sem))
				return true;
			else
				return false;
			/*command_sem = (SEM*)rt_get_adr(nam2num(sem_name.c_str()));
			if (!status_sem)
				return false;
			return true;*/
		}
		
		////////////////////////// COMMAND /////////////////////////////
		void stepCommand(VectorXd& joints_cmd,VectorXd& joints_cmd_dot)
		{
			for (int i = 0; i < joints_cmd.size(); i++) // FIX
			{
				cmd.right_arm.ctrl_mode[i] = JOINT_ARRAY_MODE_THETA_GC;
				cmd.right_arm.q_desired[i] = joints_cmd[i];
				cmd.right_arm.tq_desired[i] = 40.0;
				cmd.right_arm.slew_rate_q_desired[i] = 300; // 10.0
				cmd.right_arm.q_stiffness[i] = 1.0;
			}

			// Lock the semaphore and copy the output data
			rt_sem_wait(command_sem);
			memcpy(m3_sds->cmd, &cmd, sds_cmd_size);
			rt_sem_signal(command_sem);
			
		}

		////////////////////////// STATUS /////////////////////////////
		void stepStatus(VectorXd& joints_status,VectorXd& joints_status_dot)
		{
			// Lock the semaphore and copy the input data
			rt_sem_wait(status_sem);
			memcpy(&status, m3_sds->status, sds_status_size);
			rt_sem_signal(status_sem);
			
			// Convert status into Eigen vector
			for (int i = 0; i < joints_status.size(); i++) // FIX
			{
				joints_status[i] = status.right_arm.theta[i];
				joints_status_dot[i] = status.right_arm.thetadot[i];
			}
			
			
		}	
		
		bool m3sdsInit(std::string bot_shm_name){
			if (m3_sds = (M3Sds*)rt_shm_alloc(nam2num(bot_shm_name.c_str()),sizeof(M3Sds),USE_GFP_KERNEL))
				return true;
			else
				return false;
		}

	protected:
		bool getSemAddr(const char* sem_name,SEM* sem){
			sem = (SEM*)rt_get_adr(nam2num(sem_name));
			if (!sem)
				return false;
			return true;
		}
		
};

/////// Shared data structure:
struct DmpData
{
	Dmp* dmp; 
	double dt;
	~DmpData(){ delete dmp;}
};

///////  Trajectory generator:
Trajectory generateTrajectory(const VectorXd& ts, const VectorXd& y_first, const VectorXd& y_last, const double& Tf, const double& Ti)
{
    //VectorXd y_first = VectorXd::Zero(n_dims);
    //VectorXd y_last  = VectorXd::Ones(n_dims) * 0.3;
    
    assert(y_first.size() == y_last.size());
    
    int n_dims = y_first.size();
    double viapoint_time = (Tf -Ti)/2;

    VectorXd y_yd_ydd_viapoint = VectorXd::Zero(3*n_dims);
    
    for(int i = 0; i<n_dims; i++)
	    y_yd_ydd_viapoint[i] = (y_last[i] - y_first[i])/2;
    
    //y_yd_ydd_viapoint.segment(0*n_dims,n_dims).fill(viapoint_location); // y
    
    return  Trajectory::generatePolynomialTrajectoryThroughViapoint(ts,y_first,y_yd_ydd_viapoint,viapoint_time,y_last);
}

Dmp* generateDemoDmp(double dt){

	// GENERATE A TRAJECTORY
        // Some default values for integration
	//double dt = 0.001; //sec
	double Tf = 6; //sec
	double Ti = 0.0;
	int n_time_steps_trajectory = (int)((Tf-Ti)/dt) + 1 ;

	// Some default values for dynamical system
	//double tau = 0.6; 
	int dim = 7;
	VectorXd y_init(dim); 
	y_init   << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	VectorXd y_attr(dim);
	y_attr << 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4;
	
	VectorXd ts = VectorXd::LinSpaced(n_time_steps_trajectory,Ti,Tf); // From Ti to Tf in n_time_steps_trajectory steps
	
	Trajectory trajectory = generateTrajectory(ts, y_init, y_attr, Tf, Ti);
  
        // MAKE THE FUNCTION APPROXIMATORS
        // Initialize some meta parameters for training LWR function approximator
        int n_basis_functions = 25;
        int input_dim = 1;
        double overlap = 0.01;
        MetaParametersLWR* meta_parameters = new MetaParametersLWR(input_dim,n_basis_functions,overlap);
        FunctionApproximatorLWR* fa_lwr = new FunctionApproximatorLWR(meta_parameters);

	//Dmp::DmpType dmp_type = Dmp::KULVICIUS_2012_JOINING;
	Dmp::DmpType dmp_type = Dmp::IJSPEERT_2002_MOVEMENT;
	
	std::vector<FunctionApproximator*> function_approximators(dim);    	
	for (int dd=0; dd<dim; dd++)
		function_approximators[dd] = fa_lwr->clone();
	
	Dmp* dmp = new Dmp(dim,function_approximators,dmp_type);
	
	dmp->train(trajectory);
	
	return dmp;  
	
}

void* dmpLoop(void* args){
	
	/*SEM* status_sem;
        SEM* command_sem;
	
	sds_status_size = sizeof(M3HumanoidShmSdsStatus);
        sds_cmd_size = sizeof(M3HumanoidShmSdsCommand);
        memset(&cmd, 0, sds_cmd_size); // Initialize cmd
        */
        
	M3ShmManager shm_manager;
	
	// Retrain the dmp_data
	DmpData* dmp_data = (DmpData*) args;
	
	// Attach the thread to the shared memory
	//M3Sds* m3_sds = new M3Sds;
	/*if(!attachM3Sds(m3_sds)){
		printf("ERROR: Cannot attach the thread to the m3 shared data structure\n");
		return 0;
	}*/
	
	// Create the dmp task
	RT_TASK* dmp_task;
	//Args: Name, Priority, Stack Size, max_msg_size, Policy, cpus_allowed
	if (!(dmp_task = rt_task_init_schmod(nam2num( "DMP" ),0,0,0,SCHED_FIFO,0xF)))
	{
		printf("ERROR: Cannot initialize dmp task\n");
		return 0;
	}
	if(!shm_manager.m3sdsInit("TSHMM")){
		printf("Unable to find the %s shared memory.\n","TSHMM");
		rt_task_delete(dmp_task);
		return 0;
	}
	if(!shm_manager.statusSemInit("TSHMS")){
		printf("Unable to find the %s semaphore.\n","TSHMS");
		rt_task_delete(dmp_task);
		return 0;
	}
	if(!shm_manager.commandSemInit("TSHMC")){
		printf("Unable to find the %s semaphore.\n","TSHMC");
		rt_task_delete(dmp_task);
		return 0;
	}
	
	RTIME tick_period = nano2count(SEC2NANO(dmp_data->dt)); // This is ~=dt
	
	int dmp_dim = dmp_data->dmp->dim();
	VectorXd x(dmp_dim),xd(dmp_dim),x_updated(dmp_dim),xd_updated(dmp_dim);
	
	// Dmp initialization
	// Read the motors state
	//shm_manager.stepStatus(x,xd); //FIX This is crashing badly
	// Set the initial conditions
        dmp_data->dmp->integrateStart(x,xd);
	
	// Start the real time task
	printf("Starting real-time task\n");
	rt_task_make_periodic(dmp_task, rt_get_time() + tick_period, tick_period);
	rt_make_hard_real_time();

	// RT Loop
	//long long start_time, end_time, curr_dt;
	while(1)
	{
		// Read the motors state
		
		
		//start_time = nano2count(rt_get_cpu_time_ns());
		
		// Integrate dmp
               dmp_data->dmp->integrateStep(dmp_data->dt,x,x_updated,xd_updated);
		x = x_updated;
		
		std::cout << "****" << std::endl;
		std::cout << x.segment(0,7) << std::endl;
		
		//printf("LOOP -- Period time: %f\n",NANO2SEC((double)count2nano(curr_dt)));
		
		// And waits until the end of the period.
		rt_task_wait_period();
		//end_time = nano2count(rt_get_cpu_time_ns());
		//curr_dt = end_time - start_time;
	}

	// Task terminated
	rt_task_delete(dmp_task);
	return 0;
}	


int main(int argc, char *argv[])
{
	// Generate a demo dmp and the shared data structure
	DmpData* dmp_data = new DmpData;
	dmp_data->dt = 0.01;
	dmp_data->dmp = generateDemoDmp(dmp_data->dt);

	// Create a thread
	rt_allow_nonroot_hrt(); // It is necessary to spawn tasks
	int thread_id = rt_thread_create((void*)dmpLoop,dmp_data,10000);
	
	// Join with the thread
	rt_thread_join(thread_id);
	
	// Destroy the dmp_data
	delete dmp_data;
}


