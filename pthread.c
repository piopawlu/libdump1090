#include "pthread.h"
#include "winint.h"
#include <winsock2.h>

typedef struct _helperStruct 
{ 
	void *param; 
	ptRoutine fcn; 
}helperStruct; 

uint32_t WINAPI helper_thread(void* p) 
{ 
	helperStruct* hs = (helperStruct*)p; 

	hs->fcn(hs->param); 

	free(hs); 
	return 0; 
} 

int pthread_create(void** h, void* attr, ptRoutine fcn, void* p) 
{ 
	HANDLE hThread = 0; 
	uint32_t threadId = 0; 
	helperStruct* hs = (helperStruct*)malloc(sizeof(helperStruct)); 

	if(!hs){ 
		return -1; 
	} 

	hs->param = p; 
	hs->fcn = fcn; 

	hThread = CreateThread(NULL, NULL, helper_thread, (void*)hs, NULL, &threadId); 
	if( !hThread ) { 
		free(hs); 
		return -1; 
	} else { 
		*h = hThread; 
		return 0; 
	} 
} 

int pthread_mutex_lock(pthread_mutex_t* m){
	EnterCriticalSection((LPCRITICAL_SECTION)(*m));
	return 0;
}

int pthread_mutex_unlock(pthread_mutex_t* m){
	LeaveCriticalSection((LPCRITICAL_SECTION)(*m));
	return 0;
}

int pthread_mutex_destroy(pthread_mutex_t* m)
{
	DeleteCriticalSection((LPCRITICAL_SECTION)(*m));
	free(*m);
	return 0;
}

int pthread_mutex_init(pthread_mutex_t* m, void* d)
{
	*((LPCRITICAL_SECTION*)m) = (CRITICAL_SECTION*)malloc(sizeof(CRITICAL_SECTION));
	InitializeCriticalSection((LPCRITICAL_SECTION)(*m));
	return 0;
}

int pthread_cond_init(pthread_cond_t* c, void* d)
{
	*c = CreateEventA(NULL, FALSE, FALSE, NULL);
	return 0;
}

int pthread_cond_signal(pthread_cond_t* c)
{
	SetEvent(*c);
	return 0;
}

int pthread_cond_destroy(pthread_cond_t* c)
{
	CloseHandle(*c);
	return 0;
}