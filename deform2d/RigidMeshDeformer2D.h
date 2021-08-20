#ifndef __RMSIMPLICIT_RIGID_MESH_DEFORMER_2D_H__
#define __RMSIMPLICIT_RIGID_MESH_DEFORMER_2D_H__

#include <Eigen/Eigen>

#include "Common.h"

#include <map>
#include <set>
#include <limits>

namespace rmsmesh {

class RigidMeshDeformer2D
{
public:
	RigidMeshDeformer2D( );
	~RigidMeshDeformer2D() {};


	void ForceValidation() { ValidateSetup(); }
	void RemoveHandle( unsigned int nHandle );

/*
 * interface stuff
 */
	//unsigned int GetNumHandles();

	//const Eigen::Vector2f & GetInitialHandle(unsigned int nHandle); 
	//const Eigen::Vector2f & GetDeformedHandle( unsigned int nHandle );

	//! nHandle is vertex ID
	void SetDeformedHandle( unsigned int nHandle, const Deform2D_Vector3* vHandle );

	//void TransformPoint( Eigen::Vector2f & vTransform );
	void UnTransformPoint( Deform2D_Vector3 & vTransform );

/*
 * mesh handling
 */
	void SetMesh(
		Deform2D_Vector3* vertices,
		unsigned int vertexCount,
		unsigned int* faces,
		unsigned int faceCount
	);

	void GetDeformedMesh(
		Deform2D_Vector3* vertices,
		unsigned int vertexCount,
		bool isRigid
	);

/*
 * debug
 */
	const Eigen::Vector2f* GetTriangleVerts( unsigned int nTriangle ) { return m_vTriangles[nTriangle].vScaled; }
protected:

	struct Vertex {
		Eigen::Vector2f vPosition;
	};

public:
	struct Triangle {
		unsigned int nVerts[3];

		// definition of each vertex in triangle-local coordinate system
		Eigen::Vector2f vTriCoords[3];

		// un-scaled triangle
		Eigen::Vector2f vScaled[3];

		// pre-computed matrices for triangle scaling step
		Eigen::MatrixXd mF, mC;
	};

protected:
	std::vector<Vertex> m_vInitialVerts;
	std::vector<Vertex> m_vDeformedVerts;
	
	std::vector<Triangle> m_vTriangles;


	struct Constraint {
		unsigned int nVertex;
		Eigen::Vector2f vConstrainedPos;

		Constraint() { nVertex = 0; vConstrainedPos = { 0, 0 }; }
		Constraint( unsigned int nVert, const Eigen::Vector2f & vPos ) { nVertex = nVert; vConstrainedPos = vPos; }

		bool operator<(const Constraint & c2) const
			{ return nVertex < c2.nVertex; }
	};

	std::set<Constraint> m_vConstraints;
	void UpdateConstraint( Constraint & cons );


	bool m_bSetupValid;
	void InvalidateSetup() { m_bSetupValid = false; }
	void ValidateSetup();


	Eigen::MatrixXd m_mFirstMatrix;
	std::vector<unsigned int> m_vVertexMap;
	Eigen::MatrixXd m_mHXPrime, m_mHYPrime;
	Eigen::FullPivLU<Eigen::MatrixXd> m_mHXPrimeSolver, m_mHYPrimeSolver;
	Eigen::MatrixXd m_mDX, m_mDY;

	void PrecomputeOrientationMatrix();
	void PrecomputeScalingMatrices( unsigned int nTriangle );
	void PrecomputeFittingMatrices();

	void ValidateDeformedMesh( bool bRigid );
	void UpdateScaledTriangle( unsigned int nTriangle );
	void ApplyFittingStep();

	Eigen::Vector2f GetInitialVert( unsigned int nVert ) 
	  { return Eigen::Vector2f( m_vInitialVerts[ nVert ].vPosition.x(), m_vInitialVerts[ nVert ].vPosition.y() ); }
};


} // namespace rmsimplicit


#endif // __RMSIMPLICIT_RIGID_MESH_DEFORMER_2D_H__
