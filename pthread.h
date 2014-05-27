#ifndef _PTHREAD_H__
#define _PTHREAD_H__

#include <winsock2.h>

#define pthread_exit(n)
#define pthread_t void*
#define pthread_cond_t HANDLE
#define socklen_t int

typedef void* (*ptRoutine)(void* p);

int pthread_create(void** h, void* attr, ptRoutine, void* p);

#define pthread_close(h) CloseHandle(h)
#define pthread_kill(h) TerminateThread(h, -1)
#define pthread_join(a, b) WaitForSingleObject(a, 2000)
#define pthread_mutex_t LPCRITICAL_SECTION
int pthread_mutex_lock(pthread_mutex_t*);
int pthread_mutex_unlock(pthread_mutex_t*);
int pthread_mutex_destroy(pthread_mutex_t*);
int pthread_mutex_init(pthread_mutex_t*, void*);
int pthread_cond_init(pthread_cond_t*, void*);
int pthread_cond_signal(pthread_cond_t*);
int pthread_cond_destroy(pthread_cond_t*);
#define pthread_cancel(a)
#define pthread_self() (unsigned long)(GetCurrentThread())
#define pthread_exit(a)
#define pthread_detach(a)

#endif //_PTHREAD_H__