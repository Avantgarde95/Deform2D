#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "Deform2D.h"
#include "RigidMeshDeformer2D.h"

void* Deform2D_CreateDeformer() {
	return new rmsmesh::RigidMeshDeformer2D();
}

void Deform2D_DestroyDeformer(
	void* deformer
) {
	delete deformer;
}

void Deform2D_ForceValidation(
	void* deformer
) {
	static_cast<rmsmesh::RigidMeshDeformer2D*>(deformer)->ForceValidation();
}

void Deform2D_SetDeformedHandle(
	void* deformer,
	int handleIndex,
	const Deform2D_Vector3* handle
) {
	static_cast<rmsmesh::RigidMeshDeformer2D*>(deformer)->SetDeformedHandle(handleIndex, handle);
}

void Deform2D_RemoveHandle(
	void* deformer,
	int handleIndex
) {
	static_cast<rmsmesh::RigidMeshDeformer2D*>(deformer)->RemoveHandle(handleIndex);
}

void Deform2D_SetMesh(
	void* deformer,
	Deform2D_Vector3* vertices,
	int vertexCount,
	int* faces,
	int faceCount
) {
	// sizeof(signed) and sizeof(unsigned) are equal, so we can cast int* -> unsigned int* safely.
	// https://stackoverflow.com/questions/13169451/do-i-have-the-guarantee-that-sizeoftype-sizeofunsigned-type?rq=1
	static_cast<rmsmesh::RigidMeshDeformer2D*>(deformer)->SetMesh(vertices, vertexCount, (unsigned int*)faces, faceCount);
}

void Deform2D_GetDeformedMesh(
	void* deformer,
	Deform2D_Vector3* vertices,
	int vertexCount,
	bool isRigid
) {
	static_cast<rmsmesh::RigidMeshDeformer2D*>(deformer)->GetDeformedMesh(vertices, vertexCount, isRigid);
}

BOOL APIENTRY DllMain
(HMODULE hModule,
	DWORD  ul_reason_for_call,
	LPVOID lpReserved
) {
	switch (ul_reason_for_call) {
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}

	return TRUE;
}
