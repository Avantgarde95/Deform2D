#pragma once

#ifdef DEFORM2D_EXPORTS
#define DEFORM2D_API __declspec(dllexport)
#else
#define DEFORM2D_API __declspec(dllimport)
#endif

struct Deform2D_Vector2 {
	float x;
	float y;
};
