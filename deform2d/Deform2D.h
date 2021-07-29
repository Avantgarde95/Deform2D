#pragma once

#include "Common.h"
#include "RigidMeshDeformer2D.h"

#ifdef DEFORM2D_EXPORTS
#define DEFORM2D_API __declspec(dllexport)
#else
#define DEFORM2D_API __declspec(dllimport)
#endif

extern "C" {
	rmsmesh::RigidMeshDeformer2D* DEFORM2D_API Deform2D_CreateDeformer() {
		return new rmsmesh::RigidMeshDeformer2D();
	}

	void DEFORM2D_API Deform2D_DestroyDeformer(
		rmsmesh::RigidMeshDeformer2D* deformer
	) {
		delete deformer;
	}

	void DEFORM2D_API Deform2D_ForceValidation(
		rmsmesh::RigidMeshDeformer2D* deformer
	) {
		deformer->ForceValidation();
	}

	void DEFORM2D_API Deform2D_RemoveHandle(
		rmsmesh::RigidMeshDeformer2D* deformer,
		unsigned int handleIndex
	) {
		deformer->RemoveHandle(handleIndex);
	}

	void DEFORM2D_API Deform2D_SetMesh(
		rmsmesh::RigidMeshDeformer2D* deformer,
		Deform2D_Vector2* vertices,
		unsigned int vertexCount,
		unsigned int* faces,
		unsigned int faceCount
	) {
		deformer->SetMesh(vertices, vertexCount, faces, faceCount);
	}

	void DEFORM2D_API Deform2D_GetDeformedMesh(
		rmsmesh::RigidMeshDeformer2D* deformer,
		Deform2D_Vector2* vertices,
		unsigned int vertexCount,
		bool isRigid
	) {
		deformer->GetDeformedMesh(vertices, vertexCount, isRigid);
	}
}
