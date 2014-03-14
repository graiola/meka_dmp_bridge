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
#define RT_TIMER_TICKS_NS_BOT_SHM (10000000) //Period of rt-timer
#define BOT_SHM "TSHMM"
#define BOT_CMD_SEM "TSHMC"
#define BOT_STATUS_SEM "TSHMS"

////////////////////////////////////////////////////////////////////////////////////
static int sys_thread_active = 0;
static int sys_thread_end = 0;
static int end = 0;
static int hst;
static M3HumanoidShmSdsCommand cmd;
static M3HumanoidShmSdsStatus status;
static int sds_status_size;
static int sds_cmd_size;
static long step_cnt = 0;
static void endme(int dummy) { end=1; }
////////////////////////////////////////////////////////////////////////////////////

using namespace Eigen;
using namespace DmpBbo;

///////  Periodic Control Loop:
void StepHumanoidShm();

///////  Trajectory generator:
Trajectory getDemoTrajectory(const VectorXd& ts, const VectorXd& y_first, const VectorXd& y_last, const double& Tf, const double& Ti);

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

///////  Shared data structure:
typedef struct
{
        M3Sds m3sds; // SDS = shared data structure
        int n_dims;
        Dmp* dmp;
	double dt;
	
} M3DMPSds;

////////////////////////// MAIN COMPUTATION METHOD /////////////////////////////
void StepHumanoidShm(int cntr,VectorXd &joints_cmds_step,VectorXd & joints_cmds_dot)
{
    SetTimestamp(GetTimestamp()); //Pass back timestamp as a heartbeat

      for (int i = 0; i < 7; i++)
      {
        cmd.right_arm.ctrl_mode[i] = JOINT_ARRAY_MODE_THETA_GC;
        cmd.right_arm.q_desired[i] = joints_cmds_step(i);
        cmd.right_arm.tq_desired[i] = 40.0;
        cmd.right_arm.slew_rate_q_desired[i] = 300; // 10.0
        cmd.right_arm.q_stiffness[i] = 1.0;
      }
}

////////////////////////// RTAI PROCESS BOILERPLATE /////////////////////////////
static void* rt_system_thread(void * arg)
{
        SEM * status_sem;
        SEM * command_sem;
        RT_TASK *task;
        int cntr_outer_loop = 0;
        M3DMPSds* dmpsds = (M3DMPSds*) arg;
	
	VectorXd x(dmpsds->dmp->dim()),xd(dmpsds->dmp->dim()),x_updated(dmpsds->dmp->dim()),xd_updated(dmpsds->dmp->dim());
		
        usleep(100);

        printf("Starting real-time thread\n",0);

        sds_status_size = sizeof(M3HumanoidShmSdsStatus);
        sds_cmd_size = sizeof(M3HumanoidShmSdsCommand);

        memset(&cmd, 0, sds_cmd_size);

        task = rt_task_init_schmod(nam2num("MAMMT"), 0, 0, 0, SCHED_FIFO, 0xF); //TSHMP
        rt_allow_nonroot_hrt();
        if (task == NULL)
        {
                printf("Failed to create RT-TASK TSHMP\n",0);
                return 0;
        }
        status_sem = (SEM*)rt_get_adr(nam2num(BOT_STATUS_SEM));
        command_sem = (SEM*)rt_get_adr(nam2num(BOT_CMD_SEM));
        if (!status_sem)
        {
                printf("Unable to find the %s semaphore.\n",BOT_STATUS_SEM);
                rt_task_delete(task);
                return 0;
        }
        if (!command_sem)
        {
                printf("Unable to find the %s semaphore.\n",BOT_CMD_SEM);
                rt_task_delete(task);
                return 0;
        }

        RTIME tick_period = nano2count(RT_TIMER_TICKS_NS_BOT_SHM);
        RTIME now = rt_get_time();
        rt_task_make_periodic(task, now + tick_period, tick_period);
        mlockall(MCL_CURRENT | MCL_FUTURE);
        rt_make_hard_real_time();
        long long start_time, end_time, dt;
        long long step_cnt = 0;
        sys_thread_active = 1;

        // Read the motors state
        rt_sem_wait(status_sem);
        memcpy(&status, dmpsds->m3sds.status, sds_status_size);
        rt_sem_signal(status_sem);

        // Convert status into Eigen vector
        /*for (int i = 0; i < dmpsds->n_dims; i++)
        {
            x(i) = status.right_arm.theta[i];
            xd(i) = status.right_arm.thetadot[i];
        }*/
       
        // Initialization
        dmpsds->dmp->integrateStart(x,xd);

        while(!sys_thread_end)
        {
                start_time = nano2count(rt_get_cpu_time_ns());

                // Read the motors state
                rt_sem_wait(status_sem);
                memcpy(&status, dmpsds->m3sds.status, sds_status_size);
                rt_sem_signal(status_sem);

                // Convert status into Eigen vector
                /*for (int i = 0; i < dmpsds->n_dims; i++)
                {
                    x(i) = status.right_arm.theta[i];
                    xd(i) = status.right_arm.thetadot[i];
                }*/

                //MatrixXd joints_cmds_step = sys->joints_cmds.block(std::floor(cntr_outer_loop/sys->n_time_steps_inner_loop),0,1,sys->n_dims);
                //MatrixXd joints_cmds_dot_step = sys->joints_cmds_dot.block(std::floor(cntr_outer_loop/sys->n_time_steps_inner_loop),0,1,sys->n_dims);

                // Integrate dmp
                dmpsds->dmp->integrateStep(dmpsds->dt,x,x_updated,xd_updated); //FIX: Should I use the dt from the thread?
		
		//FIX now it is not in closed loop
		x = x_updated; // is it rt safe?

                StepHumanoidShm(cntr_outer_loop,x_updated,xd_updated);

                rt_sem_wait(command_sem);
                memcpy(dmpsds->m3sds.cmd, &cmd, sds_cmd_size);
                rt_sem_signal(command_sem);

                end_time = nano2count(rt_get_cpu_time_ns());
                dt = end_time - start_time;
                /*
                Check the time it takes to run components, and if it takes longer
                than our period, make us run slower. Otherwise this task locks
                up the CPU.*/
                if (dt > tick_period && step_cnt>10)
                {
                        printf("Step %lld: Computation time of components is too long. Forcing all components to state SafeOp.\n",step_cnt);
                        printf("Previous period: %f. New period: %f\n", (double)count2nano(tick_period),(double)count2nano(dt));
                        tick_period = dt;
                        rt_task_make_periodic(task, end + tick_period,tick_period);
                }
                step_cnt++;
                rt_task_wait_period();
        }
        printf("Exiting RealTime Thread...\n",0);
        rt_make_soft_real_time();
        rt_task_delete(task);
        sys_thread_active = 0;
        return 0;
}

////////////////////////////////////////////////////////////////////////////////////
int main (void)
{

        RT_TASK* task;
        M3DMPSds* dmpsds;

	if (dmpsds = (M3DMPSds*)rt_shm_alloc(nam2num(BOT_SHM),sizeof(M3DMPSds),USE_GFP_KERNEL)) //USE_VMALLOC
                printf("Allocated shared memory, it's time to rock baby! \n");
        else
        {
                printf("Rtai_malloc failure for %s\n",BOT_SHM);
                return 0;
        }
	
        signal(SIGINT, endme);

        // GENERATE A TRAJECTORY
        // Some default values for integration
	double dt = 0.001; //sec
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
	
	Trajectory trajectory = getDemoTrajectory(ts, y_init, y_attr, Tf, Ti);
  
        // MAKE THE FUNCTION APPROXIMATORS
        // Initialize some meta parameters for training LWR function approximator
        int n_basis_functions = 25;
        int input_dim = 1;
        double overlap = 0.01;
        MetaParametersLWR* meta_parameters = new MetaParametersLWR(input_dim,n_basis_functions,overlap);
        FunctionApproximatorLWR* fa_lwr = new FunctionApproximatorLWR(meta_parameters);

	Dmp::DmpType dmp_type = Dmp::KULVICIUS_2012_JOINING;
	dmp_type = Dmp::IJSPEERT_2002_MOVEMENT;
	
	std::vector<FunctionApproximator*> function_approximators(dim);    	
	for (int dd=0; dd<dim; dd++)
		function_approximators[dd] = fa_lwr->clone();
	
	dmpsds->dmp = new Dmp(dim,function_approximators,dmp_type);  
        // And train it. Passing the save_directory will make sure the results are saved to file.
        dmpsds->dmp->train(trajectory);
        dmpsds->n_dims = dim;
	dmpsds->dt = dt;
	
        rt_allow_nonroot_hrt();

        hst = rt_thread_create((void*)rt_system_thread, dmpsds, 10000);
        usleep(100000); //Let start up Note: give time to the creation of the rt thread

        if (!sys_thread_active)
        {
                rt_task_delete(task);
                rt_shm_free(nam2num(BOT_SHM));
                printf("Startup of thread failed.\n",0);
                return 0;
        }
        while(!end)
        {
                usleep(250000);

        }
        printf("Removing RT thread...\n",0);
        sys_thread_end=1;
        rt_thread_join(hst);
        if (sys_thread_active)printf("Real-time thread did not shutdown correctly\n");
        rt_task_delete(task);
        rt_shm_free(nam2num(BOT_SHM));
        return 0;
}

Trajectory getDemoTrajectory(const VectorXd& ts, const VectorXd& y_first, const VectorXd& y_last, const double& Tf, const double& Ti)
{
    //VectorXd y_first = VectorXd::Zero(n_dims);
    //VectorXd y_last  = VectorXd::Ones(n_dims) * 0.3;
    
    assert(y_first.size() == y_last.size());
    
    int n_dims = y_first.size();
    double viapoint_time = (Tf -Ti)/2;
    double viapoint_location = 0.2;

    VectorXd y_yd_ydd_viapoint = VectorXd::Zero(3*n_dims);
    
    for(int i = 0; i<n_dims; i++)
	    y_yd_ydd_viapoint[i] = (y_last[i] - y_first[i])/2;
    
    //y_yd_ydd_viapoint.segment(0*n_dims,n_dims).fill(viapoint_location); // y
    
    return  Trajectory::generatePolynomialTrajectoryThroughViapoint(ts,y_first,y_yd_ydd_viapoint,viapoint_time,y_last);
}


