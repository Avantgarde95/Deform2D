#pragma once

#include "Common.h"

extern "C" {
	DEFORM2D_API void* Deform2D_CreateDeformer();

	DEFORM2D_API void Deform2D_DestroyDeformer(
		void* deformer
	);

	DEFORM2D_API void Deform2D_ForceValidation(
		void* deformer
	);

	DEFORM2D_API void Deform2D_SetDeformedHandle(
		void* deformer,
		unsigned int handleIndex,
		const Deform2D_Vector2* handle
	);

	DEFORM2D_API void Deform2D_RemoveHandle(
		void* deformer,
		unsigned int handleIndex
	);

	DEFORM2D_API void Deform2D_SetMesh(
		void* deformer,
		Deform2D_Vector2* vertices,
		unsigned int vertexCount,
		unsigned int* faces,
		unsigned int faceCount
	);

	DEFORM2D_API void Deform2D_GetDeformedMesh(
		void* deformer,
		Deform2D_Vector2* vertices,
		unsigned int vertexCount,
		bool isRigid
	);
}
