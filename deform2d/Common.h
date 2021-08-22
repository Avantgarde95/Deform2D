#pragma once

#ifdef _WIN32
    #ifdef DEFORM2D_EXPORTS
        #define DEFORM2D_API __declspec(dllexport)
    #else
        #define DEFORM2D_API __declspec(dllimport)
    #endif
#elif defined(__GNUC__)
    #define DEFORM2D_API __attribute__((visibility("default")))
#else
    #define DEFORM2D_API
#endif

struct Deform2D_Vector3 {
	float x;
	float y;
	float z;
};

typedef void(__stdcall* Deform2D_InversionFunction)(double* A, int size);
