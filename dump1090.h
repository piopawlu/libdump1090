//
//  dump1090.h
//  dump1090
//
//  Created by Piotr Pawluczuk on 14.05.2014.
//
//

#ifndef dump1090_dump1090_h
#define dump1090_dump1090_h


#ifdef WIN32
#if EXPORT==1
    #define DLLEXPORT __declspec(dllexport)
#else
    #define DLLEXPORT __declspec(dllimport)
#endif
    #define CALLTYPE __stdcall
	#define sleep(a) Sleep((a)*1000)
#else
    #define DLLEXPORT
    #define CALLTYPE
#endif

typedef int (*RAWCB)(uint8_t* data, uint32_t length, void* custom);

int DLLEXPORT CALLTYPE dump1090_setCallback(RAWCB fpCallback, void* custom_parameter);
int DLLEXPORT CALLTYPE dump1090_initialize(int argc, char** argv);
int DLLEXPORT CALLTYPE dump1090_start(int async);
int DLLEXPORT CALLTYPE dump1090_stop();
int DLLEXPORT CALLTYPE dump1090_read(int blocking);

#endif
