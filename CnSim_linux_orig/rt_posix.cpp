#include "rt_posix.h"

int rt_posix_create(pthread_t* thread_id, size_t stacksize, int policy, int priority, int inheritsched, void* (*func)(void*), void* arg)
{
	pthread_attr_t attr; //thread attribute type
	struct sched_param thread_param; //scheduler parameter
	int status;

//------   initialize pthread attributes (default values)
	status = pthread_attr_init(&attr);
	if(status){
		printf("Initialize pthread attributes failed!\n"); return status;
	}

//------   set a specific stack size
	status = pthread_attr_setstacksize(&attr, stacksize);
	if(status){
		printf("Pthread : Set stack size failed!\n"); return status;
	}

//------   set scheduler policy
	status = pthread_attr_setschedpolicy(&attr, policy);
	if(status){
		printf("Pthread : Set scheduler policy failed!\n"); return status;
	}

//------   set priority of pthread
	thread_param.sched_priority = priority;
	status = pthread_attr_setschedparam(&attr, &thread_param);
	if(status){
		printf("Pthread : Set scheduler parameter failed!\n"); return status;
	}

//------   use scheduling parameters of attr
	status = pthread_attr_setinheritsched(&attr, inheritsched);
	if(status){
		printf("Pthread : Set inherit scheduler failed!\n");
	}

//------   create a pthread with specified attributes
	status = pthread_create(thread_id, &attr, func, arg); 
	if(status){
		printf("Pthread : Create pthread error[%d]\n", status);
	}

	return status;
}

void rt_posix_init_periodic(Period_Info* PInfo, long period_ns)
{
	PInfo->period_ns = 4000000; // 4ms
	//PInfo->period_ns = 4000000; // 4ms


	clock_gettime(CLOCK_REALTIME, &(PInfo->next_period));
}

void rt_posix_wait_period(struct Period_Info* PInfo)
{
	//increase next period
	PInfo->next_period.tv_nsec += PInfo->period_ns;

	if(PInfo->next_period.tv_nsec >= 1000000000){
		PInfo->next_period.tv_sec++;
		PInfo->next_period.tv_nsec -= 1000000000;
	}

	//wait next period
	clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &PInfo->next_period, NULL);
}