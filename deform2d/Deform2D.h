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
        int handleIndex,
        const Deform2D_Vector3* handle
    );

    DEFORM2D_API void Deform2D_RemoveHandle(
        void* deformer,
        int handleIndex
    );

    DEFORM2D_API void Deform2D_SetMesh(
        void* deformer,
        Deform2D_Vector3* vertices,
        int vertexCount,
        int* faces,
        int faceCount
    );

    DEFORM2D_API void Deform2D_GetDeformedMesh(
        void* deformer,
        Deform2D_Vector3* vertices,
        int vertexCount,
        bool isRigid
    );

    DEFORM2D_API void Deform2D_SetExternalSolver(
        void* deformer,
        Deform2D_SolverComputeFunction computeFunction,
        Deform2D_SolverSolveFunction solveFunction
    );
}
