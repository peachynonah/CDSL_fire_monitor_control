//------   system   ------//
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/mman.h>	  // memory mapping
#include <sys/resource.h> //stack memory extension

//------   shared memory header   ------//
#include <sys/types.h> // Type
#include <fcntl.h>	   // File Control

//------   signal header   ------//
#include <signal.h>

//------   RT header   ------//
#include "rt_posix.h"
#include <limits.h> // Limits
#include <sched.h>	// Scheduler

//------   end of cannon control header and variables (shpark)   ------//

//------   Can Interface Header and Variables (JSShin)   ------//
#include "CAN_Manager.h"

CCAN_Manager m_canManager;
//------   End of Can Interface Header and Variables (JSShin)   ------//

//------ CDSL Controller code ------//
#include "controller.h"
ManualController m_manual_controller;
PDController m_PD_controller;
FLController m_FL_controller;
#include "ReferenceGenerator.h"
ReferenceGenerator m_reference_generator;
int loop_counter = 0;

//------ CDSL Controller code ------//


// USER Paramter //
#define DEBUG false

int RT_Timeout_Error_Flag = false;
int RT_Shutdown_Flag = false;

void signal_handler(int sig)
{
	printf("\n Ctrl+C Pressed. Shutdown this program! \r\n\n");

	RT_Shutdown_Flag = true; // RT Shutdown Flag

	m_canManager.Finalize(); // Finalize CAN

	munlockall(); // Unlock Memory

	exit(1);
}

static void *run_rtCycle(void *pParam)
{
	struct Period_Info PInfo;

	struct timespec clockCheck;
	unsigned long s_time, e_time, op_time; // monitor one loop time (ns)
	unsigned long robot_time[2];		   // monitor real robot operation time

	// set cycle time and initialize next period
	rt_posix_init_periodic(&PInfo, long(1) * 1000); // long(1) -> 1ms

	int MAX_DOF = 2;

	while (1)
	{
		clock_gettime(CLOCK_REALTIME, &clockCheck); // Start time
		s_time = clockCheck.tv_sec * 1000000000 + clockCheck.tv_nsec;

		// RX process
		// Read Encoder
		double jPos[2];
		m_canManager.Recv_PDO_Data();
		
		for (int i = 0; i < MAX_DOF; i++)
		{
			// Pulse (4096/rev = 2^12, 12bit Incremental Encoder)
			// Input Pulse/Pulse(1rev) * Radian(1rev) / Reduction Ratio
			jPos[i] = (((((double)m_canManager.m_Dev[i].m_ActPos)/4096.0) * (2.0 * 3.141592)) / m_canManager.m_Dev[i].m_Gear_Ratio); // Inc to Radian
			jPos[i] = jPos[i]*(-1.0); // Convert Direction
			// printf("[%d]Actual Position : %d \r\n\n",i,m_canManager.m_Dev[i].m_ActPos);
			
			// printf("[%d]rad Position : %d \r\n\n",i,jPos[i]);
		}
	
		//-------------CDSL_Control Field------------//
		//1. basic settings

		int control_torque[2];
		control_torque[0] = 0; control_torque[1] = 0;
		
		//---1-1. loop_counter
		loop_counter++;
		double current_time = loop_counter * 0.004;
		printf("\npassed time(ms): %f", current_time);
		// printf("\nfrom KIRO, passed time: %f\n", robot_time[2]);

		//---1-2. numerical difference calculation
		double theta_dot_d = 0.0;

		

		//2. control mode selection
		int controlmode = ctrl_pd; // 0: Manual, 1: PD, 2: FL
		switch (controlmode)
		{
		case ctrl_manual:
			{	
			control_torque[0] = m_manual_controller.calculateTau(0); // Example input torque
			control_torque[1] = m_manual_controller.calculateTau(0);
			printf("\ninput of manual controller is (%d, %d)\n", control_torque[0], control_torque[1]);
			printf("\ncurrent joint_1 is  : %f\n", jPos[0]);
		    printf("\ncurrent joint_2 is  : %f\n", jPos[1]);
			break;
			}
		
		case ctrl_pd:
			{
			double theta1_desired_d = m_reference_generator.get_position(current_time);
			printf("\ndiscrete time reference signal : %f\n", theta1_desired_d);
			printf("\ncurrent joint_1 is  : %f\n", jPos[0]);
		    printf("\ncurrent joint_2 is  : %f\n", jPos[1]);

			double joint_error_1 = theta1_desired_d - jPos[0];
			double joint_error_1_dot = 0.0; // currently it's P controller
			
			control_torque[0] = static_cast<int>(m_PD_controller.calculateTau(0, joint_error_1, joint_error_1_dot));
			control_torque[1] = 0; // Assuming no control for second joint in
			printf("\ninput of pd controller is (%d, %d)\n", control_torque[0], control_torque[1]);
			break;
			}

		case ctrl_fl:
			{
			control_torque[0] = 0;
			control_torque[0] = 0;
			printf("\ninput of FL controller is (%d, %d)\n", control_torque[0], control_torque[1]);
			break;
			}

		default:
			{
			control_torque[0] = 0;
			control_torque[1] = 0;
			printf("\n control mode is not defined, so torque is set to 0\n");
			break;
			}
		}


		//-------------CDSL_Control Field Ends-------------//

		// TX process
		// Torque Command
		for (int i = 0; i < MAX_DOF; i++)
		{
			m_canManager.m_Dev[i].m_PWR_S = 1; // Servo ON(1), Servo Off(0)
			m_canManager.m_Dev[i].m_CtrlWord = m_canManager.m_Dev[i].Power(m_canManager.m_Dev[i].m_PWR_S);
				
			switch(m_canManager.m_Dev[i].m_MoOp)
			{
				default:
				case CSP: 
				m_canManager.m_Dev[i].m_TargetPos = 0;
				m_canManager.m_Dev[i].m_TargetPos = 0;
				printf("[%d]Target Position : %d \r\n\n",i,m_canManager.m_Dev[i].m_TargetPos);
				break;
				case CST: // Thousand Per Rated Torque (Rated Torque : 52.8mNm = 1000, MAXON EC-i 40)
				m_canManager.m_Dev[i].m_TargetTrq = control_torque[i]; // Input Torque
				m_canManager.m_Dev[i].m_TargetTrq = control_torque[i]; // Input Torque
				printf("[%d]Target Torque : %d \r\n\n",i,m_canManager.m_Dev[i].m_TargetTrq);
				break;
			}
			m_canManager.Send_PDO_Data(i);
		}


		clock_gettime(CLOCK_REALTIME, &clockCheck); // end time
		e_time = clockCheck.tv_sec * 1000000000 + clockCheck.tv_nsec;
		op_time = e_time - s_time;
		
		//what
		// printf("\ncontrol loop calculation time: %d", op_time);
		// printf("\nperiod_ns: %d", PInfo.period_ns);
		//is it 

		if (op_time > PInfo.period_ns)
		{
			RT_Timeout_Error_Flag = true;
			printf("\e[31m [cannon cycle time over] [%ld] us \e[0m \n\n", op_time / 1000);
		}

		// wait next period
		rt_posix_wait_period(&PInfo);
	}
	return NULL;
}

int main(int nArgc, char *ppArgv[])
{
	printf("=================== \n");
	printf(">> Start program << \n");
	printf("=================== \n\n\n");

	signal(SIGINT, signal_handler); // ctrl+C
	signal(SIGTERM, signal_handler);

	m_canManager.m_Dev[0].m_Gear_Ratio = 4440;
	m_canManager.m_Dev[1].m_Gear_Ratio = 4440;

	int opMode;

	opMode = 1; // Position Mode(0) or Torque Mode(1)

	// Initialize CAN Controller
	m_canManager.Initialize(opMode);

	//------   create thread
	pthread_t run_rt_cannon_thread; // real time loop for cannon control

	int status; // thread status

	// lock memory
	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
	{
		printf("Memory lock all failed! : %m\n");
		exit(-2);
	}

	status = rt_posix_create(&run_rt_cannon_thread, PTHREAD_STACK_MIN, SCHED_RR, 99, PTHREAD_EXPLICIT_SCHED, run_rtCycle, NULL);

	//------   block while run
	while (1)
	{
		usleep(100000);
	}

	status = pthread_join(run_rt_cannon_thread, NULL);

	// unlock memory
	munlockall();

	printf("program finished\n");

	return 0;
}
