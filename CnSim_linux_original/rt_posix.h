#ifndef RT_POSIX_H
#define RT_POSIX_H

//------   system   ------//
#include <stdio.h>




//------   RT header   ------//
#include "rt_posix.h"
#include <pthread.h> // POSIX Thread

struct Period_Info {
	struct timespec next_period;
	long period_ns;
};

int rt_posix_create(pthread_t *thread_id, size_t stacksize, int policy, int priority, int inheritsched, void* (*func)(void*), void* arg);

//set period and initialize next period
void rt_posix_init_periodic(Period_Info* PInfo, long period_ns);

void rt_posix_wait_period(struct Period_Info* PInfo);

#endif