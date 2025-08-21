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

//------   terminal(arrow keys) read header   ------//
#include <termios.h>

//------   RT header   ------//
#include "rt_posix.h"
#include <limits.h> // Limits
#include <sched.h>	// Scheduler

//------   end of cannon control header and variables (shpark)   ------//

//------   Can Interface Header and Variables (JSShin)   ------//
#include "CAN_Manager.h"

CCAN_Manager m_canManager;
//------   End of Can Interface Header and Variables (JSShin)   ------//


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

double torque_value[2] = {0.0, 0.0};
void arrow_keys()
{
    /* RAW + NON-BLOCK 모드 */
    struct RawTerm {
        termios orig;
        int     orig_flags;          // 원래 fcntl 플래그 보관
        RawTerm() {
            tcgetattr(STDIN_FILENO, &orig);
            termios raw = orig;
            raw.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);

            orig_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
            fcntl(STDIN_FILENO, F_SETFL, orig_flags | O_NONBLOCK);
        }
        ~RawTerm() {
            tcsetattr(STDIN_FILENO, TCSANOW, &orig);
            fcntl(STDIN_FILENO, F_SETFL, orig_flags);
        }
    } _raw;

    std::cout << "====== 방향키를 눌러 보세요 (q 로 종료) ======\n";

    while (true) {
        /* 입력이 없으면 토크 0 */
        torque_value[0] = 0.0;
		torque_value[1] = 0.0;

        char c;
        ssize_t n = read(STDIN_FILENO, &c, 1);

        if (n == -1 && errno == EAGAIN) {
            /* 아무 키도 안누르는 경우 → 토크 0 유지 */
			torque_value[0] = 0.0;
			torque_value[1] = 0.0;
        }
        else if (n == 1) {
            if (c == 'q') break;
            if (c != '\x1b') continue;

            char seq[2];
            if (read(STDIN_FILENO, seq, 2) != 2 || seq[0] != '[') continue;

            switch (seq[1]) {
                case 'A':  // ↑
                    std::cout << "UP\n";
                    torque_value[0] =  1.0;   // x축 +
                    torque_value[1] =  0.0;
                    break;
                case 'B':  // ↓
                    std::cout << "DOWN\n";
                    torque_value[0] = -1.0;   // x축 –
                    torque_value[1] =  0.0;
                    break;
                case 'C':  // →
                    std::cout << "RIGHT\n";
                    torque_value[0] =  0.0;
                    torque_value[1] =  1.0;   // y축 +
                    break;
                case 'D':  // ←
                    std::cout << "LEFT\n";
                    torque_value[0] =  0.0;
                    torque_value[1] = -1.0;   // y축 –
                    break;
            }
        }
        usleep(1000);          // CPU 사용 제한(1 ms)
    }
    torque_value[0] = 0.0;
	torque_value[1] = 0.0;
    std::cout << "============ Bye! =============\n";
}


double saturation(double value, double min, double max) {
	if (value < min) return min;
	if (value > max) return max;
	return value;
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
	double min = -10.0;
	double max = 10.0;
	
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
		}
	
		// double torque_value[] = {0.0, 0.0};
    	arrow_keys();

		//saturation
		for (int i = 0; i < MAX_DOF; i++)
		{
			torque_value[i] = saturation(torque_value[i], min, max);
		}


		//DOB

	
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
				m_canManager.m_Dev[i].m_TargetTrq = torque_value[i]; // Input Torque
				m_canManager.m_Dev[i].m_TargetTrq = torque_value[i]; // Input Torque
				printf("[%d]Target Torque : %d \r\n\n",i,m_canManager.m_Dev[i].m_TargetTrq);
				break;
			}
			m_canManager.Send_PDO_Data(i);
		}

		clock_gettime(CLOCK_REALTIME, &clockCheck); // end time
		e_time = clockCheck.tv_sec * 1000000000 + clockCheck.tv_nsec;
		op_time = e_time - s_time;
		
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
