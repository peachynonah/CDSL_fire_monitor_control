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

int loop_counter;
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
			printf("[%d]Actual Position : %d \r\n\n",i,m_canManager.m_Dev[i].m_ActPos);
			
			printf("[%d]rad Position : %d \r\n\n",i,jPos[i]);
		}
	
		//-------------CDSL_Control Field

		//------1. Manual Controller
		int control_torque_manual[2];
		control_torque_manual[0] = m_manual_controller.calculateTau(850);
		control_torque_manual[1] = m_manual_controller.calculateTau(0);

		//------2. PD Controller

			//---2-1. loop_counter
			loop_counter++;
			double current_time = loop_counter * 0.004;
			// printf("Control loop count: %d\n", loop_counter);
			printf("\npassed time(ms): %f\n", current_time);
			printf("\nfrom KIRO, passed time: %f\n", robot_time[2]);
			double theta1_desired_d = m_reference_generator.get_position(current_time);
			printf("\ndiscrete time reference signal : %f\n", theta1_desired_d);
			printf("\ncurrent joint is  : %f\n", jPos[0]);
		
			//--- 2-2. Control value_PD
			int control_torque[2];
			double joint_error_1 = theta1_desired_d - jPos[0];
			double joint_error_1_dot = 0.0; // currently it's P controller
			control_torque[0] = 0.0;
			control_torque[1] = 0.0;

			double control_torque_test[2];
			control_torque_test[0] = m_PD_controller.calculateTau(0,joint_error_1, joint_error_1_dot);
			control_torque_test[1] = 0.0;
			printf("\ntorque input of pd controller is %f, %f)\n", control_torque_test[0], control_torque_test[1]);

			int control_torque_test_int[2];
			control_torque_test_int[0] = static_cast<int>(control_torque_test[0]);
			control_torque_test_int[1] = 0;
			printf("\nint input of pd controller is %d, %d)\n", control_torque_test_int[0], control_torque_test_int[1]);

		//-------------CDSL_Control Field


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
				m_canManager.m_Dev[i].m_TargetTrq = control_torque_test_int[i]; // Input Torque
				m_canManager.m_Dev[i].m_TargetTrq = control_torque_test_int[i]; // Input Torque
				printf("[%d]Target Torque : %d \r\n\n",i,m_canManager.m_Dev[i].m_TargetTrq);
				break;
			}
			m_canManager.Send_PDO_Data(i);
		}


		clock_gettime(CLOCK_REALTIME, &clockCheck); // end time
		e_time = clockCheck.tv_sec * 1000000000 + clockCheck.tv_nsec;
		op_time = e_time - s_time;
		
		//what
		printf("\ncontrol loop calculation time: %d", op_time);
		printf("\nperiod_ns: %d", PInfo.period_ns);
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
