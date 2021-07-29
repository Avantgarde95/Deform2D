#pragma once

#ifdef DEFORM2D_EXPORTS
#define DEFORM2D_EXPORT __declspec(dllexport)
#else
#define DEFORM2D_EXPORT __declspec(dllimport)
#endif

struct Deform2D_Vector2 {
	float x;
	float y;
};
